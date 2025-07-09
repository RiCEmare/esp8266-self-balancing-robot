/*
 * MPU6050 Setpoint Finder using Adafruit Library (No DMP)
 * SDA -> D7 (GPIO13), SCL -> D6 (GPIO12)
 * ESP8266 with Software I2C
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define MPU_SDA 13  // D2 on NodeMCU (GPIO4)
#define MPU_SCL 12  // D1 on NodeMCU (GPIO5)

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  delay(6000);
  Serial.println("\nInitializing MPU6050 with Adafruit library...");

  // Use Software I2C
  Wire.begin(MPU_SDA, MPU_SCL);
  Wire.setClock(400000); // Optional, set I2C clock to 400kHz

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  // Set accelerometer and gyroscope ranges
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("MPU6050 ready. Hold the device steady.");
  Serial.println("Observe the 'Pitch' (approximate from accel) value below.\n");
}

void loop() {
  // Get sensor data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Estimate pitch (approx) from accelerometer
  float pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;

  Serial.print("Pitch: ");
  Serial.println(pitch);

  delay(100);  // Sample rate ~10Hz
}
