#include <Arduino.h>
#include <IRremote.hpp>
#include <Tone.h>
#include <EEPROM.h>
#include <ArduinoQueue.h>
#include "CRC8.h"

// Define I/O pins
#define IR_EMIT_PIN 4 // IR emitter pin
#define BUZZER_PIN 5

// Define constants
#define IR_SEND_INTERVAL 50  // Interval in ms to send IR signal
#define PLAYER_ADDRESS 0x02  // Example player address, can be changed
#define HP_MAX 100           // Maximum HP
#define SHIELD_HP_MAX 30     // Maximum Shield HP
#define NOTE_DELAY_DEFAULT 0 // Default note delay

// BLE Constants
#define SYN 'S'
#define SYNACK 'C'
#define ACK 'A'
#define UPDATE 'U'
#define INVALID_PACKET 'X'
#define NOT_WAITING_FOR_ACK -1
#define ACK_TIMEOUT 200
#define PACKET_SIZE 15

// Define global variables
struct Player
{
  uint8_t address;
  uint8_t hp = 100;
  uint8_t shield_hp = 0;
} myPlayer;

typedef struct Sound
{
  uint16_t note;
  uint16_t duration;
} Sound;

struct AckPacket
{
  char packetType = ACK;
  uint8_t seq = 0;
  byte padding[12] = {0};
  uint8_t crc;
} ackPacket;

struct SynAckPacket
{
  char packetType = SYNACK;
  uint8_t seq = 0;
  byte padding[12] = {0};
  uint8_t crc;
} synAckPacket;

struct AckTracker
{
  int16_t synAck = -1;
  int16_t kickAck = -1;
} ackTracker;

ArduinoQueue<Sound> soundQueue(20);
Tone buzzer;
unsigned long lastIRSendTime = 0;
unsigned long lastSoundTime = 0;
uint16_t NOTE_DELAY = NOTE_DELAY_DEFAULT;

CRC8 crc;
bool isHandshaked = false;
uint8_t updatePacketSeq = 99;

void playPlayerRespawn();
void playShieldRecharged();
void playShieldHit(uint8_t new_shield_hp);
void playShieldDestroyed();
void playPlayerHit(uint8_t new_hp);
void setupIR();
void sendIRData();
void receive_data();
void playSoundsFromQueue();
void playConnectionEstablished();

void sendSYNACK();
void waitAck(int ms);
void handshake(uint8_t);
char handleRxPacket();

void setup()
{
  Serial.begin(115200);

  // Set up the player address from EEPROM
  // EEPROM.write(0, PLAYER_ADDRESS);   // Uncomment once to store the address in EEPROM
  myPlayer.address = EEPROM.read(0); // Read the address from EEPROM

  buzzer.begin(BUZZER_PIN);

  // Initialize the IR emitter
  setupIR();
}

void loop()
{
  // Non-blocking IR signal sending
  if (millis() - lastIRSendTime >= IR_SEND_INTERVAL)
  {
    sendIRData();
    lastIRSendTime = millis();
  }

  // External data update (HP and Shield HP)
  if (Serial.available() >= PACKET_SIZE)
  {
    handleRxPacket();
  }

  playSoundsFromQueue();
}

void setupIR()
{
  // Initialize the IR sender without the default led blinker
  IrSender.begin(IR_EMIT_PIN);
  IrSender.IRLedOff(); // Turn off the IR LED blinker
}

void sendIRData()
{
  // Sending the player address using NEC2 protocol with 0 repeats
  IrSender.sendNEC(myPlayer.address, 0, 0); // Send the player address via IR with 0 repeats
  // Serial.print(F("Sent IR data with player address (NEC2, 0 repeats): 0x"));
  // Serial.println(myPlayer.address, HEX);
}

void playSoundsFromQueue()
{
  // Play sounds from the queue
  if (millis() - lastSoundTime > NOTE_DELAY)
  {
    if (soundQueue.itemCount() > 0)
    {
      Sound sound = soundQueue.dequeue();
      buzzer.play(sound.note, sound.duration);
      NOTE_DELAY = sound.duration;
    }
    lastSoundTime = millis();
  }
}

void receive_data(char *buffer)
{
  // Update the player HP and Shield HP based on the received data
  updatePacketSeq = buffer[1];
  uint8_t new_hp = buffer[2];
  uint8_t new_shield_hp = buffer[3];
  uint8_t soundType = buffer[4];

  if (soundType == 2 && myPlayer.shield_hp == 0 && new_shield_hp == SHIELD_HP_MAX)
  {
    playShieldRecharged();
  }
  else if (soundType == 1)
  {
    // few cases to play sound
    // 1. when shield is hit, but it is not destroyed
    // 2. when shield is destroyed
    // 3. when player is hit
    // 4. when player is respawned
    // shield could be destroyed and player could be hit at the same time
    if (myPlayer.shield_hp > 0 && new_shield_hp > 0 && myPlayer.shield_hp > new_shield_hp)
    {
      playShieldHit(new_shield_hp);
    }
    else if (myPlayer.shield_hp > 0 && new_shield_hp == 0)
    {
      playShieldDestroyed();
    }
    if (myPlayer.shield_hp == 0 && new_shield_hp == 0 && new_hp < myPlayer.hp)
    {
      playPlayerHit(new_hp);
    }
    else if (myPlayer.hp < new_hp)
    {
      playPlayerRespawn();
      if (new_hp < HP_MAX)
      {
        playPlayerHit(new_hp);
      }
    }
  }

  myPlayer.hp = new_hp;
  myPlayer.shield_hp = new_shield_hp;
}

void playPlayerRespawn()
{
  // play respawn sound
  Sound sound;

  // death sound
  sound.duration = 200;
  sound.note = NOTE_G2;
  soundQueue.enqueue(sound);
  soundQueue.enqueue(sound);
  soundQueue.enqueue(sound);

  // respawn sound
  sound.duration = 100;
  sound.note = NOTE_C4;
  soundQueue.enqueue(sound);
  sound.note = NOTE_E4;
  soundQueue.enqueue(sound);
  sound.note = NOTE_G4;
  soundQueue.enqueue(sound);
}

void playShieldRecharged()
{
  // play shield recharge sound
  Sound sound;
  sound.duration = 70;
  uint16_t notes[] = {NOTE_C5, NOTE_E5, NOTE_G5, NOTE_C6};

  for (int i = 0; i < 4; i++)
  {
    sound.note = notes[i];
    soundQueue.enqueue(sound);
  }
}

void playShieldHit(uint8_t new_shield_hp)
{
  // play shield hit sound based on the new shield HP
  Sound sound;
  sound.duration = 70;

  switch (new_shield_hp)
  {
  case 25:
    sound.note = NOTE_G4;
    soundQueue.enqueue(sound);
    sound.note = NOTE_GS4;
    soundQueue.enqueue(sound);
    sound.note = NOTE_G4;
    soundQueue.enqueue(sound);
    break;
  case 20:
    sound.note = NOTE_A4;
    soundQueue.enqueue(sound);
    sound.note = NOTE_AS4;
    soundQueue.enqueue(sound);
    sound.note = NOTE_A4;
    soundQueue.enqueue(sound);
    break;
  case 15:
    sound.note = NOTE_B4;
    soundQueue.enqueue(sound);
    sound.note = NOTE_C5;
    soundQueue.enqueue(sound);
    sound.note = NOTE_B4;
    soundQueue.enqueue(sound);
    break;
  case 10:
    sound.note = NOTE_C5;
    soundQueue.enqueue(sound);
    sound.note = NOTE_CS5;
    soundQueue.enqueue(sound);
    sound.note = NOTE_C5;
    soundQueue.enqueue(sound);
    break;
  case 5:
    sound.note = NOTE_D5;
    soundQueue.enqueue(sound);
    sound.note = NOTE_DS5;
    soundQueue.enqueue(sound);
    sound.note = NOTE_D5;
    soundQueue.enqueue(sound);
    break;
  default:
    break;
  }
}

void playShieldDestroyed()
{
  // play shield destroyed sound
  Sound sound;
  sound.duration = 60;
  uint16_t notes[] = {NOTE_G5, NOTE_FS5, NOTE_F5, NOTE_E5, NOTE_DS5, NOTE_D5, NOTE_CS5, NOTE_C5};

  for (int i = 0; i < 8; i++)
  {
    sound.note = notes[i];
    soundQueue.enqueue(sound);
  }
}

void playPlayerHit(uint8_t new_hp)
{
  // play player hit sound based on the new HP
  Sound sound;
  sound.duration = 100;

  switch (new_hp)
  {
  case 95:
    sound.note = NOTE_E5;
    soundQueue.enqueue(sound);
    sound.note = NOTE_D5;
    soundQueue.enqueue(sound);
    break;
  case 90:
    sound.note = NOTE_D5;
    soundQueue.enqueue(sound);
    sound.note = NOTE_CS5;
    soundQueue.enqueue(sound);
    break;
  case 85:
    sound.note = NOTE_C5;
    soundQueue.enqueue(sound);
    sound.note = NOTE_B4;
    soundQueue.enqueue(sound);
    break;
  case 80:
    sound.note = NOTE_B4;
    soundQueue.enqueue(sound);
    sound.note = NOTE_A4;
    soundQueue.enqueue(sound);
    break;
  case 75:
    sound.note = NOTE_A4;
    soundQueue.enqueue(sound);
    sound.note = NOTE_GS4;
    soundQueue.enqueue(sound);
    break;
  case 70:
    sound.note = NOTE_G4;
    soundQueue.enqueue(sound);
    sound.note = NOTE_FS4;
    soundQueue.enqueue(sound);
    break;
  case 65:
    sound.note = NOTE_F4;
    soundQueue.enqueue(sound);
    sound.note = NOTE_E4;
    soundQueue.enqueue(sound);
    break;
  case 60:
    sound.note = NOTE_E4;
    soundQueue.enqueue(sound);
    sound.note = NOTE_DS4;
    soundQueue.enqueue(sound);
    break;
  case 55:
    sound.note = NOTE_D4;
    soundQueue.enqueue(sound);
    sound.note = NOTE_CS4;
    soundQueue.enqueue(sound);
    break;
  case 50:
    sound.note = NOTE_C4;
    soundQueue.enqueue(sound);
    sound.note = NOTE_B3;
    soundQueue.enqueue(sound);
    break;
  case 45:
    sound.note = NOTE_B3;
    soundQueue.enqueue(sound);
    sound.note = NOTE_A3;
    soundQueue.enqueue(sound);
    break;
  case 40:
    sound.note = NOTE_A3;
    soundQueue.enqueue(sound);
    sound.note = NOTE_GS3;
    soundQueue.enqueue(sound);
    break;
  case 35:
    sound.note = NOTE_G3;
    soundQueue.enqueue(sound);
    sound.note = NOTE_FS3;
    soundQueue.enqueue(sound);
    break;
  case 30:
    sound.note = NOTE_F3;
    soundQueue.enqueue(sound);
    sound.note = NOTE_E3;
    soundQueue.enqueue(sound);
    break;
  case 25:
    sound.note = NOTE_E3;
    soundQueue.enqueue(sound);
    sound.note = NOTE_DS3;
    soundQueue.enqueue(sound);
    break;
  case 20:
    sound.note = NOTE_D3;
    soundQueue.enqueue(sound);
    sound.note = NOTE_CS3;
    soundQueue.enqueue(sound);
    break;
  case 15:
    sound.note = NOTE_C3;
    soundQueue.enqueue(sound);
    sound.note = NOTE_B3;
    soundQueue.enqueue(sound);
    break;
  case 10:
    sound.note = NOTE_B3;
    soundQueue.enqueue(sound);
    sound.note = NOTE_A3;
    soundQueue.enqueue(sound);
    break;
  case 5:
    sound.note = NOTE_A3;
    soundQueue.enqueue(sound);
    sound.note = NOTE_GS3;
    soundQueue.enqueue(sound);
    break;
  default:
    break;
  }
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
  //   sendSYNACK();
  //   ackTracker.synAck = seq;
  //   waitAck(ACK_TIMEOUT);
  // } while (ackTracker.synAck != NOT_WAITING_FOR_ACK);

  isHandshaked = true;
}

char handleRxPacket()
{
  // handle received packet from server
  char buffer[PACKET_SIZE];
  Serial.readBytes(buffer, PACKET_SIZE);
  uint8_t crcReceived = buffer[PACKET_SIZE - 1];
  crc.reset();
  crc.add(buffer, PACKET_SIZE - 1);
  if (!(crc.calc() == crcReceived))
  {
    Serial.readString();

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
    Serial.readString();
    return INVALID_PACKET;
    break;
  }
  return packetType;
}
