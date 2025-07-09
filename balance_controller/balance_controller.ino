#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


// PIN DEFINITIONS
#define MOTOR_L_ENA 14
#define MOTOR_L_IN1 2
#define MOTOR_L_IN2 0
#define MOTOR_R_ENB 4
#define MOTOR_R_IN3 15
#define MOTOR_R_IN4 16
#define MPU_SDA 13
#define MPU_SCL 12

Adafruit_MPU6050 mpu;

// PID Parameters
float Kp = 40.0;//35
float Ki = 180.0;//190
float Kd = 2.0;

float prev_error = 0;
float integral = 0;
float pitch = 0;
float setpoint = 0.34;


// Timing variables
unsigned long last_loop = 0;

void updatePID(float error, float dt);
void controlMotors(float output);
void stopMotors();

void setup() {
    delay(5000);
    
    Serial.begin(115200);
    Serial.println(F("Starting robot..."));
    
    // Initialize I2C with specific pins
    Wire.begin(MPU_SDA, MPU_SCL);
    Wire.setClock(400000); // Fast I2C
    
    // MPU initialization
    Serial.println(F("Initializing MPU6050..."));
    
    int mpu_attempts = 0;
    bool mpu_success = false;
    
    while (mpu_attempts < 5 && !mpu_success) {
        Serial.print(F("MPU attempt "));
        Serial.println(mpu_attempts + 1);
        
        if (mpu.begin()) {
            mpu_success = true;
            Serial.println(F("MPU6050 found!"));
            
            // Configure MPU settings
            mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
            mpu.setGyroRange(MPU6050_RANGE_250_DEG);
            mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
            
            // Test reading
            sensors_event_t a, g, temp;
            if (mpu.getEvent(&a, &g, &temp)) {
                Serial.print(F("Test reading - Accel Y: "));
                Serial.print(a.acceleration.y);
                Serial.print(F(" Gyro Y: "));
                Serial.println(g.gyro.y);
            } else {
                Serial.println(F("Failed to read MPU data"));
                mpu_success = false;
            }
        } else {
            Serial.println(F("MPU6050 not found"));
            delay(500);
        }
        
        mpu_attempts++;
        yield();
    }
    
    if (!mpu_success) {
        Serial.println(F("CRITICAL: MPU6050 initialization failed!"));
        Serial.println(F("Check wiring:"));
        Serial.println(F("SDA -> D7 (GPIO13)"));
        Serial.println(F("SCL -> D6 (GPIO12)"));
        Serial.println(F("VCC -> 3.3V"));
        Serial.println(F("GND -> GND"));
        // Don't continue without MPU
        while(1) {
            delay(1000);
            yield();
        }
    }
    
    // Motor pins - batch setup
    pinMode(MOTOR_L_ENA, OUTPUT);
    pinMode(MOTOR_L_IN1, OUTPUT);
    pinMode(MOTOR_L_IN2, OUTPUT);
    pinMode(MOTOR_R_ENB, OUTPUT);
    pinMode(MOTOR_R_IN3, OUTPUT);
    pinMode(MOTOR_R_IN4, OUTPUT);
    
    analogWriteRange(255);
    stopMotors();
    
    
    last_loop = millis();
   
    
    Serial.println(F("Setup complete"));
}

void loop() {
      // Serial input tuning interface
  if (Serial.available() > 0) {
    char cmd = Serial.read(); // Read the command character ('p', 'i', or 'd')

    // Wait a brief moment for the numerical value to arrive in the buffer
    delay(2); 

    // Check which variable to update
    if (cmd == 'p' || cmd == 'P') {
      Kp = Serial.parseFloat();
    } else if (cmd == 'i' || cmd == 'I') {
      Ki = Serial.parseFloat();
    } else if (cmd == 'd' || cmd == 'D') {
      Kd = Serial.parseFloat();
    }

    // Ensure values remain non-negative
    if (Kp < 0) Kp = 0;
    if (Ki < 0) Ki = 0;
    if (Kd < 0) Kd = 0;

    printValues(); // Print the updated values

    // Clear any remaining characters from the input buffer
    while(Serial.available() > 0) {
      Serial.read();
    }
  }

    yield();
    unsigned long now = millis();
  
    if ((now - last_loop) >= 10) {
        float dt = (now - last_loop) / 1000.0;
        last_loop = now;

        sensors_event_t a, g, temp;
        if (mpu.getEvent(&a, &g, &temp)) {
            // Complementary filter
            float accel_angle = atan2(a.acceleration.y, a.acceleration.z) * 57.296;
            float gyro_rate = g.gyro.y * 57.296;

            pitch = 0.98 * (pitch + gyro_rate * dt) + 0.02 * accel_angle;

            Serial.println(pitch);

            // Constrain pitch
            if (pitch > 50) pitch = 50;
            if (pitch < -50) pitch = -50;

            // PID control
            float error = setpoint - pitch;
            updatePID(error, dt);

            if (abs(pitch) < 35) {
                float output = Kp * error + Ki * integral + Kd  * (error - prev_error) / dt;
                controlMotors(output);
            } else {
                stopMotors();
            }

            prev_error = error;
        } else {
            Serial.println(F("Failed to read MPU data in loop"));
            static int fail_count = 0;
            if (fail_count++ > 10) {
                Serial.println(F("Too many MPU read failures - check connections"));
                fail_count = 0;
            }
        }

        yield();
    }
}

void printValues() {
  Serial.print("Kp: "); Serial.print(Kp, 4);
  Serial.print("  Ki: "); Serial.print(Ki, 4);
  Serial.print("  Kd: "); Serial.print(Kd, 4);
  Serial.println();
}

// motor control
void controlMotors(float output) {
    // Constrain output
    if (output > 255) output = 255;
    if (output < -255) output = -255;
    
    if (output > 15) { // Forward
        digitalWrite(MOTOR_L_IN1, LOW);
        digitalWrite(MOTOR_L_IN2, HIGH);
        digitalWrite(MOTOR_R_IN3, LOW);
        digitalWrite(MOTOR_R_IN4, HIGH);
        analogWrite(MOTOR_L_ENA, (int)output);
        analogWrite(MOTOR_R_ENB, (int)output);
    } else if (output < -15) { // Backward
        digitalWrite(MOTOR_L_IN1, HIGH);
        digitalWrite(MOTOR_L_IN2, LOW);
        digitalWrite(MOTOR_R_IN3, HIGH);
        digitalWrite(MOTOR_R_IN4, LOW);
        analogWrite(MOTOR_L_ENA, (int)(-output));
        analogWrite(MOTOR_R_ENB, (int)(-output));
    } else {
        stopMotors();
    }
}

void stopMotors() {
    digitalWrite(MOTOR_L_IN1, LOW);
    digitalWrite(MOTOR_L_IN2, LOW);
    digitalWrite(MOTOR_R_IN3, LOW);
    digitalWrite(MOTOR_R_IN4, LOW);
    analogWrite(MOTOR_L_ENA, 0);
    analogWrite(MOTOR_R_ENB, 0);
}
void updatePID(float error, float dt) {
    integral += error * dt;

    // Prevent integral windup
    if (integral > 10) integral = 10;
    if (integral < -10) integral = -10;
}
