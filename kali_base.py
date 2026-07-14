import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial
import struct
import threading
import math
import time

# --- HARDWARE CONFIGURATION ---
SERIAL_PORT = '/dev/ttyUSB1'
BAUD_RATE = 115200

# --- KINEMATICS CONSTANTS ---
WHEEL_BASE = 0.116       # Distance between wheels in meters
WHEEL_RADIUS = 0.0335    # 67mm diameter / 2
MAX_SPEED_MPS = 0.21     # Max speed in meters per second
TICKS_PER_REV = 400.0    # Magnetic ticks * gear ratio
M_PER_TICK = (2.0 * math.pi * WHEEL_RADIUS) / TICKS_PER_REV

START_BYTE = 0x02
END_BYTE = 0x03

def get_quaternion_from_yaw(yaw):
    """Simple Euler to Quaternion conversion for 2D rotation"""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

class MonkBaseNode(Node):
    def __init__(self):
        super().__init__('monk_base_controller')
        
        # 1. Setup Serial Connection
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
            self.get_logger().info(f"Connected to ESP32 on {SERIAL_PORT}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            raise e

        # 2. ROS 2 Publishers & Subscribers
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # 3. Telemetry & Odometry State Variables
        self.left_ticks = 0
        self.right_ticks = 0
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.last_time = self.get_clock().now()
        self.running = True

        # 4. Start Threads and Timers
        self.telemetry_thread = threading.Thread(target=self.receive_telemetry, daemon=True)
        self.telemetry_thread.start()

        # Run the odometry loop at 20Hz (0.05 seconds)
        self.odom_timer = self.create_timer(0.05, self.publish_odometry)

        self.get_logger().info("Monk OS Base Node is ACTIVE (with Odometry).")

    def cmd_vel_callback(self, msg: Twist):
        """Translates Twist (m/s) to Differential Drive PWM"""
        linear_v = msg.linear.x
        angular_w = msg.angular.z

        v_left = linear_v - (angular_w * WHEEL_BASE / 2.0)
        v_right = linear_v + (angular_w * WHEEL_BASE / 2.0)

        left_pwm = int((v_left / MAX_SPEED_MPS) * 127)
        right_pwm = int((v_right / MAX_SPEED_MPS) * 127)

        self.send_velocity_cmd(left_pwm, right_pwm)

    def send_velocity_cmd(self, left_pwm: int, right_pwm: int):
        left_pwm = max(-127, min(127, left_pwm))
        right_pwm = max(-127, min(127, right_pwm))
        checksum = (left_pwm & 0xFF) ^ (right_pwm & 0xFF)
        packet = struct.pack('<B b b B B B', START_BYTE, left_pwm, right_pwm, 0x00, checksum, END_BYTE)
        self.ser.write(packet)

    def receive_telemetry(self):
        while self.running:
            if self.ser.in_waiting >= 8:
                if self.ser.read(1)[0] == START_BYTE:
                    data = self.ser.read(7)
                    if len(data) == 7 and data[6] == END_BYTE:
                        l_low, l_high, r_low, r_high, unused, checksum = data[0:6]
                        if (l_low ^ r_low) == checksum:
                            self.left_ticks = struct.unpack('<h', bytes([l_low, l_high]))[0]
                            self.right_ticks = struct.unpack('<h', bytes([r_low, r_high]))[0]
            else:
                time.sleep(0.001)  # yield CPU when no telemetry data is pending

    def publish_odometry(self):
        """Calculates X, Y, and Theta from ticks and publishes it to ROS 2"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        # 1. Calculate delta ticks
        delta_l_ticks = self.left_ticks - self.prev_left_ticks
        delta_r_ticks = self.right_ticks - self.prev_right_ticks

        self.prev_left_ticks = self.left_ticks
        self.prev_right_ticks = self.right_ticks

        # 2. Convert ticks to meters traveled
        dist_l = delta_l_ticks * M_PER_TICK
        dist_r = delta_r_ticks * M_PER_TICK
        dist_center = (dist_l + dist_r) / 2.0

        # 3. Calculate heading change (theta)
        delta_theta = (dist_r - dist_l) / WHEEL_BASE

        # 4. Calculate X and Y coordinates
        # Using simple Euler integration
        self.x += dist_center * math.cos(self.theta + (delta_theta / 2.0))
        self.y += dist_center * math.sin(self.theta + (delta_theta / 2.0))
        self.theta += delta_theta

        # Calculate actual velocities for the message
        if dt > 0:
            vx = dist_center / dt
            vth = delta_theta / dt
        else:
            vx = 0.0
            vth = 0.0

        q = get_quaternion_from_yaw(self.theta)

        # 5. Publish the Transform (tf) - Vital for Navigation/LiDAR
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        # 6. Publish the Odometry Message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth
        self.odom_pub.publish(odom)

        self.last_time = current_time

    def destroy_node(self):
        self.running = False
        self.send_velocity_cmd(0, 0) 
        self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MonkBaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down cleanly...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
