#include <Wire.h>
#define MPU_ADDR 0x68

// MPU6050 Registers
#define REG_PWR_MGMT_1 0x6B
#define REG_ACCEL_XOUT_H 0x3B
#define REG_GYRO_XOUT_H  0x43

// Sensitivities
const float ACCEL_SENS = 16384.0;
const float GYRO_SENS  = 131.0;

// Complementary filter constants
const float alpha = 0.98;
const float beta = 1.0 - alpha;

// Variables
float roll = 0, pitch = 0, yaw = 0;
float gx_bias = 0, gy_bias = 0, gz_bias = 0;
unsigned long lastMicros;

int16_t read16(int reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);
  int16_t hi = Wire.read();
  int16_t lo = Wire.read();
  return (hi << 8) | lo;
}

void readMPU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  int16_t rawAX = read16(REG_ACCEL_XOUT_H);
  int16_t rawAY = read16(REG_ACCEL_XOUT_H + 2);
  int16_t rawAZ = read16(REG_ACCEL_XOUT_H + 4);
  int16_t rawGX = read16(REG_GYRO_XOUT_H);
  int16_t rawGY = read16(REG_GYRO_XOUT_H + 2);
  int16_t rawGZ = read16(REG_GYRO_XOUT_H + 4);

  ax = rawAX / ACCEL_SENS;
  ay = rawAY / ACCEL_SENS;
  az = rawAZ / ACCEL_SENS;
  gx = rawGX / GYRO_SENS - gx_bias;
  gy = rawGY / GYRO_SENS - gy_bias;
  gz = rawGZ / GYRO_SENS - gz_bias;
}

void calibrateGyro(int samples = 1000) {
  Serial.println("Calibrating gyro... Keep MPU6050 still!");
  delay(1000);
  
  long sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < samples; i++) {
    sumX += read16(REG_GYRO_XOUT_H);
    sumY += read16(REG_GYRO_XOUT_H + 2);
    sumZ += read16(REG_GYRO_XOUT_H + 4);
    if (i % 100 == 0) Serial.print(".");
    delay(5);
  }
  Serial.println("\nCalibration complete!");
  
  gx_bias = (sumX / (float)samples) / GYRO_SENS;
  gy_bias = (sumY / (float)samples) / GYRO_SENS;
  gz_bias = (sumZ / (float)samples) / GYRO_SENS;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Initialize MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_PWR_MGMT_1);
  Wire.write(0x00); // Wake up the sensor
  Wire.endTransmission(true);
  delay(100);
  
  calibrateGyro(800);
  
  // Initialize angles from accelerometer
  float ax, ay, az, gx, gy, gz;
  readMPU(ax, ay, az, gx, gy, gz);
  roll  = atan2(ay, az) * 180.0 / PI;
  pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0 / PI;
  yaw = 0;
  
  lastMicros = micros();
  Serial.println("MPU6050 Ready! Sending data...");
}

void loop() {
  float ax, ay, az, gx, gy, gz;
  readMPU(ax, ay, az, gx, gy, gz);
  
  unsigned long now = micros();
  float dt = (now - lastMicros) / 1000000.0;
  lastMicros = now;
  
  // Calculate angles from accelerometer (for roll and pitch)
  float acc_roll  = atan2(ay, az) * 180.0 / PI;
  float acc_pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0 / PI;
  
  // Integrate gyroscope data
  float roll_gyro  = roll  + gx * dt;
  float pitch_gyro = pitch + gy * dt;
  float yaw_gyro   = yaw   + gz * dt;
  
  // Complementary filter
  roll  = alpha * roll_gyro  + beta * acc_roll;
  pitch = alpha * pitch_gyro + beta * acc_pitch;
  yaw   = yaw_gyro; // Yaw from gyro only (no magnetometer)
  
  // Ensure angles stay within -180 to 180 degrees
  if (roll > 180) roll -= 360;
  if (roll < -180) roll += 360;
  if (pitch > 180) pitch -= 360;
  if (pitch < -180) pitch += 360;
  if (yaw > 180) yaw -= 360;
  if (yaw < -180) yaw += 360;
  
  // Send data as CSV
  Serial.print(pitch, 2);
  Serial.print(",");
  Serial.print(roll, 2);
  Serial.print(",");
  Serial.println(yaw, 2);
  
  delay(15); // ~66Hz update rate
}