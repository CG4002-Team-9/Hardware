#include <Wire.h>
#include <MPU6050.h>
#include <IRremote.hpp>

// Pin assignments
#define MPU6050_INT_PIN 2    // Interrupt pin for MPU6050
#define IR_RECEIVER_PIN 3    // IR receiver pin
#define BUTTON_PIN 4         // Button pin
#define BUZZER_PIN 5         // Buzzer pin
#define sCommand 0x59

// Constants
const int MAX_BULLETS = 6;        // Maximum number of bullets
const int LOW_FREQ = 2500;         // Low frequency for buzzer (in Hz)
const int HIGH_FREQ = 4500;       // High frequency for buzzer (in Hz)
const int QUEUE_SIZE = 4;          // queue size for buzzer sounds
const int samplingRate = 50;       // 20 Hz = 50 ms

// Global variables
volatile int bulletCount = MAX_BULLETS;  // Current number of bullets
volatile bool motionDetected = false;    // changes to true if mpu ISR triggered
unsigned long lastSampleTime = 0;        // last sample time of mpu
const unsigned long debounceDelay = 50;  // 50 ms debounce time
unsigned long lastDebounceTime = 0;      // Store the last debounce time
bool buttonState = HIGH;                 // Current state of the button
bool lastButtonState = HIGH;             // Previous state of the button
MPU6050 mpu;                             // MPU6050 object
unsigned long hitData = 0x00;

// Buzzer queue variables
struct BuzzerSound {
  int frequency;
  int duration;
};

BuzzerSound buzzerQueue[QUEUE_SIZE];
int queueStart = 0;  // Points to the start of the queue
int queueEnd = 0;    // Points to the end of the queue
unsigned long lastBuzzerTime = 0;  // Time of the last buzzer sound

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
  
  // Initialize button and buzzer pins
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  
  // Initialize IR receiver
  IrReceiver.begin(IR_RECEIVER_PIN, ENABLE_LED_FEEDBACK);

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
      Serial.print(gy); Serial.print(", ");
      Serial.println(gz);
      motionDetected = false;
    }
  }

  // button check
  int reading = digitalRead(BUTTON_PIN);
  // If the button state has changed, reset the debounce timer
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading == LOW && buttonState == HIGH) {
      // Button is pressed, only fire once per press
      buttonState = LOW;
      handleButtonPress();
    } else if (reading == HIGH && buttonState == LOW) {
      // Button is released
      buttonState = HIGH;  // Reset the button state
    }
  }
  // Save the current reading as the last button state
  lastButtonState = reading;

  // Check for incoming player state updates from the server
  //if (Serial.available() >= sizeof(PlayerState)) {
    //PlayerState playerState;
    //Serial.readBytes((char*)&playerState, sizeof(PlayerState));

    // Update the bullet count if it has changed
    //if (playerState.bullet > bulletCount) {
      //playReloadSound();
    //}
    //bulletCount = playerState.bullet;
  //}

  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); 
    if (command == "reload") {
      bulletCount = 6;
      playReloadSound();
    }
  }

  // Buzzer playing routine using queue
  if (millis() - lastBuzzerTime > 500) {
    if (queueStart != queueEnd) {
      // There is a sound to play in the queue
      tone(BUZZER_PIN, buzzerQueue[queueStart].frequency, buzzerQueue[queueStart].duration);
      lastBuzzerTime = millis();  // Update the time of the last buzzer sound

      // Move to the next sound in the queue
      queueStart = (queueStart + 1) % QUEUE_SIZE;
    }
  }

}
void mpuISR() {
  motionDetected = true;
}

void handleButtonPress() {
  if (bulletCount == 0) {
    // Out of bullets
    Serial.println("out of bullets");
    addToBuzzerQueue(LOW_FREQ, 200); // Low-high sound pattern for 500ms
    addToBuzzerQueue(HIGH_FREQ, 200);
  }
  else {
    // bullets > 0
    int freq = 2500 + 400*(bulletCount-1);
    //if (IrReceiver.decode() && IrReceiver.decodedIRData.command == sCommand) {
    if (IrReceiver.decode()) {
      // successful shot, freq based on remaining bulletCount. Sound once
      Serial.println("Hit");
      addToBuzzerQueue(freq, 200);
      Serial.println(IrReceiver.decodedIRData.address, HEX); // Print "old" raw data
      //Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX); // Print "old" raw data
      IrReceiver.printIRResultShort(&Serial); // Print complete received data in one line
      IrReceiver.printIRSendUsage(&Serial);   // Print the statement required to send this data
      IrReceiver.resume();
    }
    else {
      // shot missed/stray sigbal. Freq based on reamining bulletCount. Sound twice
      Serial.println("Miss");
      addToBuzzerQueue(freq, 200);
      addToBuzzerQueue(freq, 200);
    }
    bulletCount--;
  }
}

void playReloadSound() {
  // Play low-high-low sound pattern for reload indication
  addToBuzzerQueue(LOW_FREQ, 200);
  addToBuzzerQueue(HIGH_FREQ, 200);
  addToBuzzerQueue(LOW_FREQ, 200);
}

void addToBuzzerQueue(int frequency, int duration) {
  // Add a sound to the buzzer queue
  buzzerQueue[queueEnd].frequency = frequency;
  buzzerQueue[queueEnd].duration = duration;
  queueEnd = (queueEnd + 1) % QUEUE_SIZE;
  if (queueEnd == queueStart) {
    // Queue overflow, discard the oldest entry
    queueStart = (queueStart + 1) % QUEUE_SIZE;
  }
}
