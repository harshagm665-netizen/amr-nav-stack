import serial
import time
import struct
import threading

# Configuration
SERIAL_PORT = '/dev/ttyUSB1'  # Change this to match your setup!
BAUD_RATE = 115200

START_BYTE = 0x02
END_BYTE = 0x03

class RobotBridge:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.running = True
        
        # Telemetry variables
        self.left_ticks = 0
        self.right_ticks = 0

    def send_velocity_cmd(self, left_pwm: int, right_pwm: int):
        """Sends a 6-byte packet to the ESP32"""
        # Clamp PWM values to -127 / +127 as expected by your ESP32
        left_pwm = max(-200, min(200, left_pwm))
        right_pwm = max(-200, min(200, right_pwm))

        # Calculate checksum (left ^ right)
        # Using ctypes-like masking to handle negative numbers in bitwise XOR safely
        checksum = (left_pwm & 0xFF) ^ (right_pwm & 0xFF)

        # Pack the bytes: Start, Left, Right, Unused(0), Checksum, End
        # 'b b b b b b' format means 6 signed/unsigned chars
        packet = struct.pack('<B b b B B B', 
                             START_BYTE, 
                             left_pwm, 
                             right_pwm, 
                             0x00, 
                             checksum, 
                             END_BYTE)
        
        self.ser.write(packet)

    def receive_telemetry(self):
        """Reads the 8-byte packet from the ESP32 in a loop"""
        while self.running:
            if self.ser.in_waiting >= 8:
                # Read until we hit a START_BYTE
                if self.ser.read(1)[0] == START_BYTE:
                    # Read the remaining 7 bytes
                    data = self.ser.read(7)
                    if len(data) == 7 and data[6] == END_BYTE:
                        # Extract bytes
                        l_low, l_high, r_low, r_high, unused, checksum = data[0:6]
                        
                        # Verify checksum (tx[1] ^ tx[3] in your C++ code)
                        calc_checksum = l_low ^ r_low
                        
                        if calc_checksum == checksum:
                            # Unpack the 16-bit little-endian integers
                            self.left_ticks = struct.unpack('<h', bytes([l_low, l_high]))[0]
                            self.right_ticks = struct.unpack('<h', bytes([r_low, r_high]))[0]
                            print(f"Telemetry -> Left Ticks: {self.left_ticks} | Right Ticks: {self.right_ticks}")
                        else:
                            print("Checksum error in received telemetry!")

    def close(self):
        self.running = False
        self.send_velocity_cmd(0, 0) # Stop motors
        self.ser.close()

if __name__ == "__main__":
    robot = RobotBridge(SERIAL_PORT, BAUD_RATE)
    
    # Start the telemetry listener in a background thread
    listener_thread = threading.Thread(target=robot.receive_telemetry, daemon=True)
    listener_thread.start()

    print("Robot Bridge Active. Press Ctrl+C to stop.")
    try:
        while True:
            # Command the robot to drive forward slowly (PWM = 30 out of 127)
            robot.send_velocity_cmd(30, 30)
            time.sleep(0.1) # Command loop running at 10Hz
            
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        robot.close()
