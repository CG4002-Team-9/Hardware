#include <Wire.h>
#include <MPU6050.h>

#define MPU6050_INT_PIN 2    // Interrupt pin for MPU6050

const int samplingRate = 50;       // 20 Hz = 50 ms

volatile bool motionDetected = false;    // changes to true if mpu ISR triggered
unsigned long lastSampleTime = 0;        // last sample time of mpu
MPU6050 mpu;                             // MPU6050 object

void setup() {
  // Initialize Serial communication
  Serial.begin(115200);

  // Initialize MPU6050
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection())
  {
    Serial.println("MPU6050 connection failed");
    while (1)
      ;
  }
  
  // Set MPU6050 interrupt
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4); // Set accelerometer range to 4 g
  mpu.setDHPFMode(MPU6050_DHPF_0P63);             // high pass filter 0.63hz            bandpass filter from 0.63hz to 20hz
  mpu.setDLPFMode(MPU6050_DLPF_BW_20);            // low pass filter 20hz
  mpu.setMotionDetectionThreshold(15);
  mpu.setMotionDetectionDuration(5);
  mpu.setIntMotionEnabled(true);
  pinMode(MPU6050_INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(MPU6050_INT_PIN), mpuISR, RISING);
}

void loop() {
  // mpu ISR triggered
  if (motionDetected) {
    if (millis() - lastSampleTime >= samplingRate)
    {
      lastSampleTime = millis();

      // Read accelerometer and gyroscope data
      int16_t ax, ay, az;
      int16_t gx, gy, gz;
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

      Serial.print("Accel: ");
      Serial.print(ax); Serial.print(", ");
      Serial.print(ay); Serial.print(", ");
      Serial.print(az); Serial.print(" | ");
      Serial.print("Gyro: ");
      Serial.print(gx); Serial.print(", ");
      Serial.print(gy); Serial.println(gz);
      motionDetected = false;
    }
  }
}

void mpuISR() {
  motionDetected = true;
}
