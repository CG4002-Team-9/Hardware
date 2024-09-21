#include <Arduino.h>
#include <MPU6050.h>
#include <Tone.h>
#include <ArduinoQueue.h>
#include <EEPROM.h>
#include "CRC8.h"

// Define I/O pins
#define MPU_INTERRUPT_PIN 2
#define BUZZER_PIN 5

// Define constants
#define KICK_DELAY 2000  // Delay in ms for kick detection
#define NUM_SAMPLES 30
#define PLAYER_ADDRESS_EEPROM 0  // EEPROM address to store/retrieve player address

#define SYN 'S'
#define SYNACK 'C'
#define ACK 'A'
#define KICK 'K'
#define INVALID_PACKET 'X'
#define NOT_WAITING_FOR_ACK -1
#define ACK_TIMEOUT 200

// Define global variables
struct Player {
  uint8_t address;
} myPlayer;

typedef struct Sound {
  uint16_t note;
  uint8_t duration;
} Sound;

struct AckPacket {
  char packetType = ACK;
  uint8_t seq = 0;
  byte padding[17] = { 0 };
  uint8_t crc;
} ackPacket;

struct KickPacket {
  char packetType = KICK;
  uint8_t seq = 0;
  byte padding[17] = { 0 };
  uint8_t crc;
} kickPacket;

struct SynAckPacket {
  char packetType = SYNACK;
  uint8_t seq = 0;
  byte padding[17] = { 0 };
  uint8_t crc;
} synAckPacket;

struct AckTracker {
  int16_t synAck = -1;
  int16_t kickAck = -1;
} ackTracker;

Tone buzzer;
MPU6050 mpu;
ArduinoQueue<Sound> soundQueue(10);
CRC8 crc;

uint8_t kickSeq = 0;
bool isHandshaked = false;

unsigned long lastSoundTime = 0;
uint16_t NOTE_DELAY = 0;
bool isKickDetected = false;
unsigned long lastSampleTime = 0;

void playKickDetected();
void setupMPU();
void kickDetected();
void sendSoccerToServer();  // Placeholder for sending event to server
void playSoundsFromQueue();
void loadPlayer();

// BLE
void getKickPacket();
void sendKICK();
void sendSYNACK();
void waitAck(int ms);
void handshake(uint8_t seq);
char handleRxPacket();

  void setup() {
  Serial.begin(115200);

  // Load player information (address) from EEPROM
  loadPlayer();

  buzzer.begin(BUZZER_PIN);

  setupMPU();

  Serial.println(F("Leg monitor setup complete"));
}

void loop() {
  if (Serial.available() >= 20) {
    handleRxPacket();
  }

  // Sound playing subroutine
  playSoundsFromQueue();

  // Handle MPU data for kick detection
  if (isKickDetected) {
    playKickDetected();
    sendSoccerToServer();  // Send kick event to server
    isKickDetected = false;
  }
}

// Load player data from EEPROM
void loadPlayer() {
  // Set default player address in EEPROM if necessary
  // EEPROM.write(PLAYER_ADDRESS_EEPROM, 0x02); // Uncomment this line to set a default address, run once, and comment it again

  // Read player address from EEPROM
  myPlayer.address = EEPROM.read(PLAYER_ADDRESS_EEPROM);
  // Serial.print(F("Player address loaded from EEPROM: 0x"));
  // Serial.println(myPlayer.address, HEX);
}

// Initialize MPU and interrupt
void setupMPU() {
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println(F("MPU6050 connection failed"));
    while (1)
      ;
  }

  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);

  mpu.setDHPFMode(MPU6050_DHPF_5);
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);

  mpu.setMotionDetectionThreshold(120);
  mpu.setMotionDetectionDuration(5);
  mpu.setIntMotionEnabled(true);

  pinMode(MPU_INTERRUPT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), kickDetected, RISING);
}

void kickDetected() {
  // Set flag for kick detection
  if (!isKickDetected && millis() - lastSampleTime > KICK_DELAY) {
    isKickDetected = true;
    lastSampleTime = millis();
    // Serial.println(F("Kick detected!"));
  }
}

void playKickDetected() {
  // Enqueue sound for kick detected
  Sound sound;
  sound.duration = 100;
  sound.note = NOTE_C4;
  soundQueue.enqueue(sound);
  sound.note = NOTE_D4;
  soundQueue.enqueue(sound);
  sound.note = NOTE_E4;
  soundQueue.enqueue(sound);
}

// Queue-based sound playing
void playSoundsFromQueue() {
  if (millis() - lastSoundTime > NOTE_DELAY) {
    if (soundQueue.itemCount() > 0) {
      Sound sound = soundQueue.dequeue();
      buzzer.play(sound.note, sound.duration);
      NOTE_DELAY = sound.duration;
    }
    lastSoundTime = millis();
  }
}

// TODO: Implement external data updates (e.g., from Bluetooth or server)
void sendSoccerToServer() {
  // Placeholder function to send kick event to server
  // You can implement actual sending logic here
  // Serial.print(F("Kick event sent to server for player address: 0x"));
  // Serial.println(myPlayer.address, HEX);
  getKickPacket();
  do {
    sendKICK();
    ackTracker.kickAck = kickPacket.seq;
    waitAck(ACK_TIMEOUT);
  } while (ackTracker.kickAck != NOT_WAITING_FOR_ACK);
}

void getKickPacket() {
  kickPacket.seq = ++kickSeq;
  crc.reset();
  crc.add((byte *)&kickPacket, sizeof(kickPacket) - 1);
  kickPacket.crc = crc.calc();
}

void sendKICK() {
  Serial.write((byte *)&kickPacket, sizeof(kickPacket));
}

void sendSYNACK() {
  crc.reset();
  crc.add((byte *)&synAckPacket, sizeof(synAckPacket) - 1);
  synAckPacket.crc = crc.calc();
  Serial.write((byte *)&synAckPacket, sizeof(synAckPacket));
}

void waitAck(int ms) {
  for (int i = 0; i < ms; i++) {
    if (Serial.available() >= 20) {
      char packetTypeRx = handleRxPacket();
      if (packetTypeRx == SYNACK) {
        return;
      }
    }
    delay(1);
  }
}

void handshake(uint8_t seq) {
  isHandshaked = false;
  do {
    sendSYNACK();
    ackTracker.synAck = seq;
    waitAck(ACK_TIMEOUT);
  } while (ackTracker.synAck != NOT_WAITING_FOR_ACK);

  isHandshaked = true;
}

char handleRxPacket() {
  char buffer[20];
  Serial.readBytes(buffer, 20);

  uint8_t crcReceived = buffer[19];
  crc.reset();
  crc.add(buffer, 19);
  if (!(crc.calc() == crcReceived)) {
    return INVALID_PACKET;
  }

  char packetType = buffer[0];
  uint8_t seqReceived = buffer[1];
  switch (packetType) {
    case SYN:
      handshake(seqReceived);
      break;

    case SYNACK:
      ackTracker.synAck = NOT_WAITING_FOR_ACK;
      break;

    case ACK:
      if (ackTracker.kickAck == seqReceived) {
        ackTracker.kickAck = NOT_WAITING_FOR_ACK;
      }
      break;

    default:
      break;
  }
  return packetType;
}
