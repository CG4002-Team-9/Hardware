#include <Arduino.h>
#include <IRremote.hpp>
#include <Tone.h>
#include <EEPROM.h>
#include <ArduinoQueue.h>
#include "CRC8.h"

// Define I/O pins
#define IR_EMIT_PIN 3  // IR emitter pin
#define BUZZER_PIN 5

// Define constants
#define IR_SEND_INTERVAL 50   // Interval in ms to send IR signal
#define PLAYER_ADDRESS 0x02   // Example player address, can be changed
#define HP_MAX 100            // Maximum HP
#define SHIELD_HP_MAX 50      // Maximum Shield HP
#define NOTE_DELAY_DEFAULT 0  // Default note delay

// BLE
#define SYN 'S'
#define SYNACK 'C'
#define ACK 'A'
#define UPDATE 'U'
#define INVALID_PACKET 'X'
#define NOT_WAITING_FOR_ACK -1
#define ACK_TIMEOUT 200

// Define global variables
struct Player {
  uint8_t address;
  uint8_t hp;
  uint8_t shield_hp;
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

ArduinoQueue<Sound> soundQueue(10);
Tone buzzer;
unsigned long lastIRSendTime = 0;
unsigned long lastSoundTime = 0;
uint16_t NOTE_DELAY = NOTE_DELAY_DEFAULT;

// BLE
CRC8 crc;
bool isHandshaked = false;
uint8_t updatePacketSeq = 99;

void playHitDetected();
void playRespawn();
void setupIR();
void sendIRData();
void receive_data();  // Placeholder for external data updates
void playSoundsFromQueue();

void sendSYNACK();
void waitAck(int ms);
void handshake(uint8_t);
char handleRxPacket();

void setup() {
  Serial.begin(115200);

  // Set up the player address from EEPROM
  // EEPROM.write(0, PLAYER_ADDRESS);   // Uncomment once to store the address in EEPROM
  myPlayer.address = EEPROM.read(0);  // Read the address from EEPROM

  // Initialize player HP and Shield HP
  myPlayer.hp = HP_MAX;
  myPlayer.shield_hp = SHIELD_HP_MAX;

  // Serial.print(F("Player address: 0x"));
  // Serial.println(myPlayer.address, HEX);
  // Serial.print(F("Initial HP: "));
  // Serial.println(myPlayer.hp);
  // Serial.print(F("Initial Shield HP: "));
  // Serial.println(myPlayer.shield_hp);

  buzzer.begin(BUZZER_PIN);

  // Initialize the IR emitter
  setupIR();

  //Serial.println(F("Vest setup complete"));
}

void loop() {
  // Non-blocking IR signal sending
  if (millis() - lastIRSendTime >= IR_SEND_INTERVAL) {
    sendIRData();
    lastIRSendTime = millis();
  }

  // External data update (HP and Shield HP)
  if (Serial.available() >= 20) {
    handleRxPacket();
  }

  // Play sounds from the queue
  playSoundsFromQueue();
}

void setupIR() {
  // Initialize the IR sender without the default led blinker
  IrSender.begin(IR_EMIT_PIN);
  IrSender.IRLedOff();  // Turn off the IR LED blinker
}

void sendIRData() {
  // Sending the player address using NEC2 protocol with 0 repeats
  IrSender.sendNEC(myPlayer.address, 0, 0);  // Send the player address via IR with 0 repeats
  // Serial.print(F("Sent IR data with player address (NEC2, 0 repeats): 0x"));
  // Serial.println(myPlayer.address, HEX);
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

void playHitDetected() {
  // Enqueue sounds when the player is hit
  Sound sound;
  sound.duration = 100;
  sound.note = NOTE_G4;
  soundQueue.enqueue(sound);
  sound.note = NOTE_A4;
  soundQueue.enqueue(sound);
  sound.note = NOTE_B4;
  soundQueue.enqueue(sound);
}

void playRespawn() {
  // Enqueue sounds when the player respawns
  Sound sound;
  sound.duration = 80;
  sound.note = NOTE_C5;
  soundQueue.enqueue(sound);
  sound.note = NOTE_D5;
  soundQueue.enqueue(sound);
  sound.note = NOTE_E5;
  soundQueue.enqueue(sound);
}

// TODO: Implement external data updates (e.g., from Bluetooth or server)
void receive_data(char *buffer) {
  // Placeholder function to update HP and Shield HP externally
  updatePacketSeq = buffer[1];
  myPlayer.hp = buffer[2];
  myPlayer.shield_hp = buffer[3];

  // For now, just print the values (you can replace this with actual data updates)
  // Serial.print(F("Current HP: "));
  // Serial.println(myPlayer.hp);
  // Serial.print(F("Current Shield HP: "));
  // Serial.println(myPlayer.shield_hp);

  // For demo, let's assume the player got hit, and play the hit sound
  if (myPlayer.hp < HP_MAX) {
    playHitDetected();
  }
}

void sendACK(uint8_t seq) {
  ackPacket.seq = seq;
  crc.reset();
  crc.add((byte *)&ackPacket, sizeof(ackPacket) - 1);
  ackPacket.crc = crc.calc();
  Serial.write((byte *)&ackPacket, sizeof(ackPacket));
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
      if (packetTypeRx == ACK || packetTypeRx == SYNACK) {
        return;
      }
    }
    delay(1);
  }
}

void handshake(uint8_t seq) {
  isHandshaked = false;
  sendSYNACK();
  // do {
  //   sendSYNACK();
  //   ackTracker.synAck = seq;
  //   waitAck(ACK_TIMEOUT);
  // } while (ackTracker.synAck != NOT_WAITING_FOR_ACK);

  isHandshaked = true;
}

char handleRxPacket() {
  char buffer[20];
  Serial.readBytes(buffer, 20);

  uint8_t crcReceived = buffer[19];
  crc.reset();
  crc.add(buffer, 19);
  if (!(crc.calc() == crcReceived)) {
    //Serial.readString(); // clear the buffer just in case
    return INVALID_PACKET;
  }

  char packetType = buffer[0];
  uint8_t seqReceived = buffer[1];

  switch (packetType) {
    case UPDATE:
      sendACK(seqReceived);
      if (updatePacketSeq != seqReceived) {
        receive_data(buffer);
      }
      break;

    case SYN:
      handshake(seqReceived);
      break;

    case SYNACK:
      ackTracker.synAck = NOT_WAITING_FOR_ACK;
      break;

    default:
      break;
  }
  return packetType;
}