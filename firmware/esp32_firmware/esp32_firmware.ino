#include <Arduino.h>
#include <driver/pcnt.h>

// ---------- HARDWARE PIN ASSIGNMENTS ----------
#define LEFT_ENA   25
#define LEFT_IN1   32
#define LEFT_IN2   33
#define LEFT_ENC_A 27 
#define LEFT_ENC_B 26 

#define RIGHT_ENA   18
#define RIGHT_IN1   19
#define RIGHT_IN2   21
#define RIGHT_ENC_A 23
#define RIGHT_ENC_B 22

#define START_BYTE  0x02
#define END_BYTE    0x03
#define PACKET_SIZE 6

// ---------- ROBOT KINEMATICS & PID ----------
const float MAX_TICKS_PER_SEC = 400.0; // 60 RPM * 400 ticks/rev
const float DT = 0.02; // 20ms loop time (50Hz)

// PID Tuning Parameters
float Kp = 0.8;
float Ki = 2.5;

float left_integral = 0;
float right_integral = 0;

pcnt_unit_t pcnt_left = PCNT_UNIT_0;
pcnt_unit_t pcnt_right = PCNT_UNIT_1;

SemaphoreHandle_t xVelocityMutex;
volatile int8_t global_pwm_left = 0;
volatile int8_t global_pwm_right = 0;
volatile unsigned long last_heartbeat_time = 0;
const unsigned long WATCHDOG_TIMEOUT_MS = 150;

volatile int32_t absolute_left_ticks = 0;
volatile int32_t absolute_right_ticks = 0;

void setupEncoder(pcnt_unit_t unit, int pulsePin, int ctrlPin) {
    pinMode(pulsePin, INPUT_PULLUP);
    pinMode(ctrlPin, INPUT_PULLUP);
    pcnt_config_t cfg = {
        .pulse_gpio_num = pulsePin, .ctrl_gpio_num  = ctrlPin,
        .lctrl_mode = PCNT_MODE_REVERSE, .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC, .neg_mode = PCNT_COUNT_DEC,
        .counter_h_lim = 32767, .counter_l_lim = -32768,
        .unit = unit, .channel = PCNT_CHANNEL_0,
    };
    pcnt_unit_config(&cfg);
    pcnt_set_filter_value(unit, 100); 
    pcnt_filter_enable(unit);
    pcnt_counter_pause(unit); pcnt_counter_clear(unit); pcnt_counter_resume(unit);
}

int16_t getEncoderDelta(pcnt_unit_t unit) {
    int16_t count;
    pcnt_get_counter_value(unit, &count);
    pcnt_counter_clear(unit);
    return count;
}

void writeMotorHardware(bool is_left, int output_pwm) {
    int in1 = is_left ? LEFT_IN1 : RIGHT_IN1;
    int in2 = is_left ? LEFT_IN2 : RIGHT_IN2;
    int pwmChannel = is_left ? 0 : 1;
    
    // Clamp the PID output to max hardware PWM
    if (output_pwm > 255) output_pwm = 255;
    if (output_pwm < -255) output_pwm = -255;

    // Deadband for completely stopping
    if (abs(output_pwm) < 15) {
        digitalWrite(in1, LOW); digitalWrite(in2, LOW);
        ledcWrite(pwmChannel, 0);
        return;
    }

    int speed = abs(output_pwm); 

    // --- SOFTWARE DIRECTION INVERSION ---
    // Since the physical wires are wrapped, we flip the HIGH/LOW logic here.
    // A positive output_pwm now sets IN1 LOW and IN2 HIGH to drive physically forward.
    if (output_pwm > 0) {
        digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
    } else {
        digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
    }
    ledcWrite(pwmChannel, speed);
}

void runControlLoop(void * pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz Loop

    for(;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // 1. Get Encoder Deltas (Ticks since last loop)
        int16_t delta_left = getEncoderDelta(pcnt_left);
        int16_t delta_right = getEncoderDelta(pcnt_right);
        absolute_left_ticks += delta_left;
        absolute_right_ticks += delta_right;

        // Calculate actual speed in Ticks per Second
        float current_speed_left = (float)delta_left / DT;
        float current_speed_right = (float)delta_right / DT;

        // 2. Safety Watchdog
        if (millis() - last_heartbeat_time > WATCHDOG_TIMEOUT_MS) {
            writeMotorHardware(true, 0); writeMotorHardware(false, 0);
            left_integral = 0; right_integral = 0; // Reset windup when stopped
            // We do nothing else here, allowing it to fall through to telemetry
        } 
        else {
            // 3. Read Commanded Target
            int8_t local_cmd_left = 0;
            int8_t local_cmd_right = 0;
            if (xSemaphoreTake(xVelocityMutex, (TickType_t)5) == pdTRUE) {
                local_cmd_left = global_pwm_left;
                local_cmd_right = global_pwm_right;
                xSemaphoreGive(xVelocityMutex);
            }

            // Convert the -127 to +127 command into a Target Speed (Ticks per Second)
            float target_speed_left = ((float)local_cmd_left / 127.0) * MAX_TICKS_PER_SEC;
            float target_speed_right = ((float)local_cmd_right / 127.0) * MAX_TICKS_PER_SEC;

            // If commanded to stop, reset integrals to prevent sudden lurches
            if (local_cmd_left == 0 && local_cmd_right == 0) {
                left_integral = 0; right_integral = 0;
                writeMotorHardware(true, 0); writeMotorHardware(false, 0);
            } else {
                // 4. Calculate PID for Left Motor
                float error_left = target_speed_left - current_speed_left;
                left_integral += error_left * DT;
                float output_left = (Kp * error_left) + (Ki * left_integral);

                // 5. Calculate PID for Right Motor
                float error_right = target_speed_right - current_speed_right;
                right_integral += error_right * DT;
                float output_right = (Kp * error_right) + (Ki * right_integral);

                // 6. Drive Motors
                writeMotorHardware(true, (int)output_left); 
                writeMotorHardware(false, (int)output_right);
            }
        }

        // 7. Transmit Uplink (Always runs, keeping the Pi updated)
        uint8_t tx[8];
        tx[0] = START_BYTE;
        tx[1] = (uint8_t)(absolute_left_ticks & 0xFF);
        tx[2] = (uint8_t)((absolute_left_ticks >> 8) & 0xFF);
        tx[3] = (uint8_t)(absolute_right_ticks & 0xFF);
        tx[4] = (uint8_t)((absolute_right_ticks >> 8) & 0xFF);
        tx[5] = 0x00; 
        tx[6] = tx[1] ^ tx[3]; 
        tx[7] = END_BYTE;
        Serial.write(tx, 8);
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(LEFT_IN1, OUTPUT); pinMode(LEFT_IN2, OUTPUT); pinMode(LEFT_ENA, OUTPUT);
    pinMode(RIGHT_IN1, OUTPUT); pinMode(RIGHT_IN2, OUTPUT); pinMode(RIGHT_ENA, OUTPUT);
    
    ledcSetup(0, 1000, 8); ledcAttachPin(LEFT_ENA, 0);
    ledcSetup(1, 1000, 8); ledcAttachPin(RIGHT_ENA, 1);
    
    setupEncoder(pcnt_left, LEFT_ENC_A, LEFT_ENC_B); 
    setupEncoder(pcnt_right, RIGHT_ENC_A, RIGHT_ENC_B);
    
    xVelocityMutex = xSemaphoreCreateMutex();
    last_heartbeat_time = millis();
    xTaskCreatePinnedToCore(runControlLoop, "ControlLoop", 3072, NULL, 2, NULL, 1);
}

void loop() {
    uint8_t rx[PACKET_SIZE];
    if (Serial.available() >= PACKET_SIZE) {
        if (Serial.read() == START_BYTE) {
            rx[0] = START_BYTE;
            for (int i = 1; i < PACKET_SIZE; i++) rx[i] = Serial.read();
            if (rx[PACKET_SIZE - 1] == END_BYTE) {
                if ((rx[1] ^ rx[2]) == rx[4]) {
                    if (xSemaphoreTake(xVelocityMutex, portMAX_DELAY) == pdTRUE) {
                        global_pwm_left = (int8_t)rx[1];
                        global_pwm_right = (int8_t)rx[2];
                        xSemaphoreGive(xVelocityMutex);
                    }
                    last_heartbeat_time = millis();
                }
            }
        }
    }
}
