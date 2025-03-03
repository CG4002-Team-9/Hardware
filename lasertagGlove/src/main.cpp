#include <Arduino.h>
#include <IRremote.hpp>
#include <MPU6050.h>
#include <ArduinoQueue.h>
#include <Tone.h>
#include <EEPROM.h>
#include "CRC8.h"

// Define I/O pins
#define DECODE_NEC // Enable NEC protocol. This is the protocol used for the IR receiver
#define IMU_INTERRUPT_PIN 2
#define BUTTON_PIN 3
#define IR_RECEIVE_PIN 4 // receiver is the gun
#define BUZZER_PIN 5

// Define constants
#define DEBOUNCE_DELAY 200
#define MPU_SAMPLING_RATE 40 // in Hz
#define MOTION_DETECTED_DELAY 800
#define IR_SEARCH_TIMEOUT 200
#define NUM_SAMPLES 65

// BLE constants
#define SYN 'S'
#define SYNACK 'C'
#define ACK 'A'
#define UPDATE 'U'
#define DATA 'D'
#define SHOOT 'G'
#define INVALID_PACKET 'X'
#define NOT_WAITING_FOR_ACK -1
#define ACK_TIMEOUT 200
#define PACKET_SIZE 15

// Define global variables
struct Player
{
  uint8_t address;
  uint8_t bullets = 0;
} myPlayer;

typedef struct Sound
{
  uint16_t note;
  uint16_t duration;
} Sound;

struct mpuCalibration
{
  int16_t ax_offset;
  int16_t ay_offset;
  int16_t az_offset;
  int16_t gx_offset;
  int16_t gy_offset;
  int16_t gz_offset;
} mpuCal;

struct AckPacket
{
  char packetType = ACK;
  uint8_t seq = 0;
  byte padding[PACKET_SIZE - 3] = {0};
  uint8_t crc;
} ackPacket;

struct SynAckPacket
{
  char packetType = SYNACK;
  uint8_t seq = 0;
  byte padding[PACKET_SIZE - 3] = {0};
  uint8_t crc;
} synAckPacket;

struct ShootPacket
{
  char packetType = SHOOT;
  uint8_t seq = 0;
  uint8_t hit = 0;
  byte padding[PACKET_SIZE - 4] = {0};
  uint8_t crc;
} shootPacket;

struct DataPacket
{
  char packetType = DATA;
  uint8_t seq;
  int16_t accX;
  int16_t accY;
  int16_t accZ;
  int16_t gyrX;
  int16_t gyrY;
  int16_t gyrZ;
  uint8_t crc;
} dataPacket;

struct AckTracker
{
  int16_t synAck = -1;
  int16_t shootAck = -1;
} ackTracker;

ArduinoQueue<Sound> soundQueue(20);
Tone buzzer;
MPU6050 mpu;

const unsigned long SAMPLING_DELAY = 1000 / MPU_SAMPLING_RATE;
uint8_t mpuSamples = 0;

bool isButtonPressed = false;
bool isMotionDetected = false;
bool isFindingIR = false;

CRC8 crc;
uint8_t shootSeq = 0;
int bulletAddr = 0;
bool isHandshaked = false;
uint8_t updatePacketSeq = 99;

// millis variables
unsigned long lastDebounceTime = 0;
unsigned long lastSoundTime = 0;
uint16_t NOTE_DELAY = 0;
unsigned long lastSampleTime = 0;
unsigned long lastMotionDetectedTime = 0;
unsigned long irStartTime = 0;

// function declarations
void motionDetected();
void buttonPressed();
void playMotionDetected();
void playMotionEnded();
void playSuccessfulShot();
void playSuccessfulReload();
void playShootBullet(uint8_t bullets);
void playEmptyGun();
void sendShotDataToServer(bool hitDetected);
void sendIMUDataToServer(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz);
void playConnectionEstablished();

void sendACK(uint8_t seq);
void sendSYNACK();
char handleRxPacket();
void waitAck(int ms);
void handshake(uint8_t seq);

void setup()
{
  Serial.begin(115200);

  // write to EEPROM the player address
  // EEPROM.write(0, 0x02);             // only do this once, then comment out
  myPlayer.address = EEPROM.read(0); // read the address from EEPROM

  // save to EEPROM the calibration values
  // mpuCal.ax_offset = -1560;
  // mpuCal.ay_offset = -215;
  // mpuCal.az_offset = 166;
  // mpuCal.gx_offset = 28;
  // mpuCal.gy_offset = -119;
  // mpuCal.gz_offset = 18;
  // EEPROM.put(1, mpuCal); // only do this once, then comment out

  // read from EEPROM and set the calibration values
  mpuCal = EEPROM.get(1, mpuCal);
  mpu.setXAccelOffset(mpuCal.ax_offset);
  mpu.setYAccelOffset(mpuCal.ay_offset);
  mpu.setZAccelOffset(mpuCal.az_offset);
  mpu.setXGyroOffset(mpuCal.gx_offset);
  mpu.setYGyroOffset(mpuCal.gy_offset);
  mpu.setZGyroOffset(mpuCal.gz_offset);

  // Serial.print(F("Player address: 0x"));
  // Serial.println(myPlayer.address, HEX);

  buzzer.begin(BUZZER_PIN);

  mpu.initialize();
  if (!mpu.testConnection())
  {
    //  Serial.println(F("MPU6050 connection failed"));
    while (1)
      ;
  }

  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);

  mpu.setDHPFMode(MPU6050_DHPF_0P63);
  mpu.setDLPFMode(MPU6050_DLPF_BW_20);

  mpu.setMotionDetectionThreshold(50);
  mpu.setMotionDetectionDuration(5);
  mpu.setIntMotionEnabled(true);

  pinMode(IMU_INTERRUPT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(IMU_INTERRUPT_PIN), motionDetected, RISING);

  pinMode(BUZZER_PIN, OUTPUT);

  // interrupt for button press both rising and falling
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonPressed, FALLING);

  pinMode(IR_RECEIVE_PIN, INPUT);
  IrReceiver.begin(IR_RECEIVE_PIN);
  // Serial.println(F("Setup complete"));
}

void loop()
{
  if (Serial.available() >= PACKET_SIZE && !isMotionDetected)
  {
    handleRxPacket();
  }

  // buttonpress subroutine
  if (isButtonPressed)
  {
    if (myPlayer.bullets > 0)
    {
      playShootBullet(myPlayer.bullets);
      myPlayer.bullets--;

      isFindingIR = true;
    }
    else
    {
      playEmptyGun();
      sendShotDataToServer(false);
    }
    isButtonPressed = false;
  }

  // sound playing subroutine
  if (millis() - lastSoundTime > NOTE_DELAY)
  {
    if (soundQueue.itemCount() > 0)
    {
      Sound sound = soundQueue.dequeue();
      buzzer.play(sound.note, sound.duration);
      NOTE_DELAY = sound.duration;
    }
    else if (soundQueue.itemCount() == 0)
    {
      IrReceiver.restartTimer();
    }
    lastSoundTime = millis();
  }

  // ir data collect subroutine
  if (isFindingIR)
  {
    IrReceiver.start();
    irStartTime = millis();
    bool hitDetected = false;
    while (millis() - irStartTime < IR_SEARCH_TIMEOUT)
    {
      if (IrReceiver.decode())
      {
        // if received data is valid with NEC2, enqueue the address
        if (IrReceiver.decodedIRData.protocol != UNKNOWN && IrReceiver.decodedIRData.address != myPlayer.address)
        {
          hitDetected = true;
          break;
        }
        IrReceiver.resume();
      }
    }
    IrReceiver.stop();

    sendShotDataToServer(hitDetected);
    isFindingIR = false;
  }

  // mpu data collect subroutine
  if (isMotionDetected && (millis() - lastSampleTime > SAMPLING_DELAY))
  {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    sendIMUDataToServer(ax, ay, az, gx, gy, gz);

    lastSampleTime = millis();
    if (++mpuSamples >= NUM_SAMPLES)
    {
      mpuSamples = 0;
      isMotionDetected = false;
      lastMotionDetectedTime = millis();
      playMotionEnded();
    }
  }
}

// function definitions
void motionDetected()
{
  if (!isMotionDetected && millis() - lastMotionDetectedTime > MOTION_DETECTED_DELAY)
  {
    isMotionDetected = true;
    playMotionDetected();
  }
}

void buttonPressed()
{
  if (millis() - lastDebounceTime > DEBOUNCE_DELAY && !isFindingIR && !isButtonPressed)
  {
    isButtonPressed = true;
    lastDebounceTime = millis();
  }
  
  if (isMotionDetected) {
    mpuSamples = 0;
    isMotionDetected = false;
    lastMotionDetectedTime = millis();
  }
}

void playSuccessfulShot()
{
  // play a sound to indicate successful shot
  Sound sound;
  sound.note = 0;
  sound.duration = 100;
  soundQueue.enqueue(sound);
  sound.duration = 80;
  sound.note = NOTE_AS5;
  soundQueue.enqueue(sound);
  sound.note = NOTE_B5;
  soundQueue.enqueue(sound);
}

void playSuccessfulReload()
{
  // play a sound to indicate successful reload
  Sound sound;
  sound.duration = 50;
  sound.note = NOTE_C6;
  soundQueue.enqueue(sound);
  sound.note = NOTE_CS6;
  soundQueue.enqueue(sound);
  sound.note = NOTE_D6;
  soundQueue.enqueue(sound);
  sound.note = NOTE_DS6;
  soundQueue.enqueue(sound);
}

void playShootBullet(uint8_t bullets)
{
  // play a sound based on the number of bullets left
  Sound sound;

  switch (bullets)
  {
  case 1:
    sound.note = NOTE_C4;
    break;
  case 2:
    sound.note = NOTE_CS4;
    break;
  case 3:
    sound.note = NOTE_D4;
    break;
  case 4:
    sound.note = NOTE_DS4;
    break;
  case 5:
    sound.note = NOTE_E4;
    break;
  case 6:
    sound.note = NOTE_F4;
    break;
  default:
    break;
  }
  sound.duration = 150;
  soundQueue.enqueue(sound);
}

void playEmptyGun()
{
  // play a sound to indicate empty gun
  Sound sound;
  sound.duration = 50;
  sound.note = NOTE_A3;
  soundQueue.enqueue(sound);
  sound.note = NOTE_G3;
  soundQueue.enqueue(sound);
  sound.note = NOTE_F3;
  soundQueue.enqueue(sound);
}

void playMotionDetected()
{
  // play a sound to indicate motion detected
  Sound sound;
  sound.duration = 255;
  sound.note = NOTE_AS6;
  soundQueue.enqueue(sound);
}

void playMotionEnded()
{
  // play a sound to indicate motion ended
  Sound sound;
  sound.duration = 255;
  sound.note = NOTE_A6;
  soundQueue.enqueue(sound);
}

void playConnectionEstablished()
{
  // play a sound to indicate connection established
  Sound sound;
  sound.duration = 200;
  sound.note = NOTE_C5;
  soundQueue.enqueue(sound);
  sound.duration = 120;
  sound.note = NOTE_A4;
  soundQueue.enqueue(sound);
  sound.note = NOTE_F4;
  soundQueue.enqueue(sound);
  sound.note = NOTE_E5;
  soundQueue.enqueue(sound);
}

void sendShotDataToServer(bool hitDetected)
{
  // sends gun action to server
  shootPacket.seq = ++shootSeq;
  shootPacket.hit = uint8_t(hitDetected);
  crc.reset();
  crc.add((byte *)&shootPacket, sizeof(shootPacket) - 1);
  shootPacket.crc = crc.calc();
  do
  {
    Serial.write((byte *)&shootPacket, sizeof(shootPacket));
    ackTracker.shootAck = shootPacket.seq;
    waitAck(ACK_TIMEOUT);
  } while (ackTracker.shootAck != NOT_WAITING_FOR_ACK);
}

void sendIMUDataToServer(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz)
{
  // sends imu data to server
  dataPacket.seq = mpuSamples;
  dataPacket.accX = ax;
  dataPacket.accY = ay;
  dataPacket.accZ = az;
  dataPacket.gyrX = gx;
  dataPacket.gyrY = gy;
  dataPacket.gyrZ = gz;
  crc.reset();
  crc.add((byte *)&dataPacket, sizeof(dataPacket) - 1);
  dataPacket.crc = crc.calc();
  Serial.write((byte *)&dataPacket, sizeof(dataPacket));
}

void sendACK(uint8_t seq)
{
  // send ACK packet to server
  ackPacket.seq = seq;
  crc.reset();
  crc.add((byte *)&ackPacket, sizeof(ackPacket) - 1);
  ackPacket.crc = crc.calc();
  Serial.write((byte *)&ackPacket, sizeof(ackPacket));
}

void sendSYNACK()
{
  // send SYNACK packet to server
  crc.reset();
  crc.add((byte *)&synAckPacket, sizeof(synAckPacket) - 1);
  synAckPacket.crc = crc.calc();
  Serial.write((byte *)&synAckPacket, sizeof(synAckPacket));
}

void waitAck(int ms)
{
  // wait for ACK packet from server
  for (int i = 0; i < ms; i++)
  {
    if (Serial.available() >= PACKET_SIZE)
    {
      char packetTypeRx = handleRxPacket();
      if (packetTypeRx == ACK || packetTypeRx == SYNACK)
      {
        return;
      }
    }
    delay(1);
  }
}

void handshake(uint8_t seq)
{
  // handshake with server
  isHandshaked = false;
  playConnectionEstablished();
  sendSYNACK();
  // do {
  //    sendSYNACK();
  //    ackTracker.synAck = seq;
  //    waitAck(ACK_TIMEOUT);
  // } while (ackTracker.synAck != NOT_WAITING_FOR_ACK);

  isHandshaked = true;
  shootSeq = seq;
}

char handleRxPacket()
{
  // handle packets received from server
  char buffer[PACKET_SIZE];
  Serial.readBytes(buffer, PACKET_SIZE);

  uint8_t crcReceived = buffer[PACKET_SIZE - 1];
  crc.reset();
  crc.add(buffer, PACKET_SIZE - 1);
  if (!(crc.calc() == crcReceived))
  {
    Serial.readString(); // clear the buffer just in case
    return INVALID_PACKET;
  }

  char packetType = buffer[0];
  uint8_t seqReceived = buffer[1];

  switch (packetType)
  {
  case UPDATE:
    sendACK(seqReceived);
    if (updatePacketSeq != seqReceived)
    {
      updatePacketSeq = buffer[1];
      bool isReload = buffer[6];
      uint8_t newBullets = buffer[5];
      if (isReload && myPlayer.bullets == 0 && newBullets == 6)
      {
        playSuccessfulReload();
      }
      myPlayer.bullets = newBullets;

      // EEPROM.update(bulletAddr, buffer[4]);
    }
    break;

  case SYN:
    handshake(seqReceived);
    break;

  case SYNACK:
    ackTracker.synAck = NOT_WAITING_FOR_ACK;
    break;

  case ACK:
    if (ackTracker.shootAck == seqReceived)
    {
      ackTracker.shootAck = NOT_WAITING_FOR_ACK;
    }
    break;

  default:
    Serial.readString(); // clear the buffer just in case
    return INVALID_PACKET;
    break;
  }
  return packetType;
}
