#include <Arduino.h>
#include <MPU6050.h>
#include <Tone.h>
#include <ArduinoQueue.h>
#include <EEPROM.h>
#include "CRC8.h"

// Define I/O pins
#define IMU_INTERRUPT_PIN 2
#define BUTTON_PIN 3
#define BUZZER_PIN 5

// Define constants
#define KICK_DELAY 800 // Delay in ms for kick detection
#define NUM_SAMPLES 30
#define PLAYER_ADDRESS_EEPROM 0 // EEPROM address to store/retrieve player address
#define MPU_SAMPLING_RATE 40    // in Hz
#define MOTION_DETECTED_DELAY 1000
#define NUM_SAMPLES 45
#define IMU_THRESHOLD 80
#define IMU_THRESHOLD_DURATION 5

#define SYN 'S'
#define SYNACK 'C'
#define ACK 'A'
#define DATA 'D'
#define INVALID_PACKET 'X'
#define NOT_WAITING_FOR_ACK -1
#define ACK_TIMEOUT 200
#define PACKET_SIZE 15

// Define global variables
struct Player
{
    uint8_t address;
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
    char packetType = ACK;
    uint8_t seq = 0;
    byte padding[PACKET_SIZE - 3] = {0};
    uint8_t crc;
} ackPacket;

struct KickPacket
{
    char packetType = KICK;
    uint8_t seq = 0;
    byte padding[12] = {0};
    uint8_t crc;
} kickPacket;

struct SynAckPacket
{
    char packetType = SYNACK;
    uint8_t seq = 0;
    byte padding[12] = {0};
    uint8_t crc;
    char packetType = SYNACK;
    uint8_t seq = 0;
    byte padding[PACKET_SIZE - 3] = {0};
    uint8_t crc;
} synAckPacket;

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
    int16_t kickAck = -1;
    int16_t synAck = -1;
} ackTracker;

ArduinoQueue<Sound> soundQueue(20);
Tone buzzer;
MPU6050 mpu;

const unsigned long SAMPLING_DELAY = 1000 / MPU_SAMPLING_RATE;
uint8_t mpuSamples = 0;
bool isMotionDetected = false;

// BLE variables
CRC8 crc;
bool isHandshaked = false;
bool isSendingIMU = false;

unsigned long lastSoundTime = 0;
uint16_t NOTE_DELAY = 0;
unsigned long lastSampleTime = 0;
unsigned long lastMotionDetectedTime = 0;

void setupMPU();
void motionDetected();
void playMotionDetected();
void playMotionEnded();
void sendIMUDataToServer(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz);
void playConnectionEstablished();

// BLE Function declarations
void sendSYNACK();
char handleRxPacket();
void waitAck(int ms);
void handshake(uint8_t seq);

void setup()
{
    Serial.begin(115200);
    // Serial.write('+');
    // Serial.write('+');
    // Serial.write('+');
    // delay(500);
    // Serial.print("AT+RESTART\r\n");

    // Load player information (address) from EEPROM
    loadPlayer();

    buzzer.begin(BUZZER_PIN);

    setupMPU();

    // Serial.println(F("Leg monitor setup complete"));
}

void loop()
{
    if (Serial.available() >= PACKET_SIZE)
    {
        handleRxPacket();
    }

    // Sound playing subroutine
    playSoundsFromQueue();

    // Handle MPU data for kick detection
    if (isKickDetected)
    {
        playKickDetected();
        sendSoccerToServer(); // Send kick event to server
        isKickDetected = false;
    }
}

// Load player data from EEPROM
void loadPlayer()
{
    // Set default player address in EEPROM if necessary
    // EEPROM.write(PLAYER_ADDRESS_EEPROM, 0x02); // Uncomment this line to set a default address, run once, and comment it again

    // Read player address from EEPROM
    myPlayer.address = EEPROM.read(PLAYER_ADDRESS_EEPROM);
    // Serial.print(F("Player address loaded from EEPROM: 0x"));
    // Serial.println(myPlayer.address, HEX);

    mpuCal = EEPROM.get(1, mpuCal);
    mpu.setXAccelOffset(mpuCal.ax_offset);
    mpu.setYAccelOffset(mpuCal.ay_offset);
    mpu.setZAccelOffset(mpuCal.az_offset);
    mpu.setXGyroOffset(mpuCal.gx_offset);
    mpu.setYGyroOffset(mpuCal.gy_offset);
    mpu.setZGyroOffset(mpuCal.gz_offset);
}

// Initialize MPU and interrupt
void setupMPU()
{
    mpu.initialize();
    if (!mpu.testConnection())
    {
        Serial.println(F("MPU6050 connection failed"));
        while (1)
            ;
    }
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

    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);

    mpu.setDHPFMode(MPU6050_DHPF_5);
    mpu.setDLPFMode(MPU6050_DLPF_BW_42);

    mpu.setMotionDetectionThreshold(120);
    mpu.setMotionDetectionDuration(5);
    mpu.setIntMotionEnabled(true);
    mpu.setMotionDetectionThreshold(IMU_THRESHOLD);
    mpu.setMotionDetectionDuration(IMU_THRESHOLD_DURATION);
    mpu.setIntMotionEnabled(true);

    pinMode(MPU_INTERRUPT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), kickDetected, RISING);
    pinMode(IMU_INTERRUPT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(IMU_INTERRUPT_PIN), motionDetected, RISING);

    pinMode(BUZZER_PIN, OUTPUT);
    // Serial.println(F("Setup complete"));
}

void kickDetected()
{
    // Set flag for kick detection
    if (!isKickDetected && millis() - lastSampleTime > KICK_DELAY)
    {
        isKickDetected = true;
        lastSampleTime = millis();
        // Serial.println(F("Kick detected!"));
    }
    void loop()
    {
        if (Serial.available() >= PACKET_SIZE && !isSendingIMU)
        {
            handleRxPacket();
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
            lastSoundTime = millis();
        }

        // mpu data collect subroutine
        if (isMotionDetected && (millis() - lastSampleTime > SAMPLING_DELAY))
        {
            int16_t ax, ay, az;
            int16_t gx, gy, gz;
            mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

            isSendingIMU = true;
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

    // put function definitions here:

    void motionDetected()
    {
        if (!isMotionDetected && millis() - lastMotionDetectedTime > MOTION_DETECTED_DELAY)
        {
            isMotionDetected = true;
            playMotionDetected();
        }
    }

    void playMotionDetected()
    {
        // Enqueue sound for kick detected
        Sound sound;
        sound.duration = 100;
        sound.note = NOTE_C4;
        soundQueue.enqueue(sound);
        sound.note = NOTE_D4;
        soundQueue.enqueue(sound);
        sound.note = NOTE_E4;
        soundQueue.enqueue(sound);
        // play a sound to indicate motion detected
        Sound sound;
        sound.duration = 255;
        sound.note = NOTE_GS5;
        soundQueue.enqueue(sound);
    }

    void playMotionEnded()
    {
        // play a sound to indicate motion ended
        Sound sound;
        sound.duration = 255;
        sound.note = NOTE_G5;
        soundQueue.enqueue(sound);
        isSendingIMU = false;
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

    // Queue-based sound playing
    void playSoundsFromQueue()
    {
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

    // TODO: Implement external data updates (e.g., from Bluetooth or server)
    void sendSoccerToServer() void sendIMUDataToServer(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz)
    {
        // Placeholder function to send kick event to server
        // You can implement actual sending logic here
        // Serial.print(F("Kick event sent to server for player address: 0x"));
        // Serial.println(myPlayer.address, HEX);
        getKickPacket();
        do
        {
            Serial.write((byte *)&kickPacket, sizeof(kickPacket));
            ackTracker.kickAck = kickPacket.seq;
            waitAck(ACK_TIMEOUT);
        } while (ackTracker.kickAck != NOT_WAITING_FOR_ACK);
    }

    void getKickPacket()
    {
        kickPacket.seq = ++kickSeq;
        crc.reset();
        crc.add((byte *)&kickPacket, sizeof(kickPacket) - 1);
        kickPacket.crc = crc.calc();
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

    void sendSYNACK()
    {
        crc.reset();
        crc.add((byte *)&synAckPacket, sizeof(synAckPacket) - 1);
        synAckPacket.crc = crc.calc();
        Serial.write((byte *)&synAckPacket, sizeof(synAckPacket));
    }

    void waitAck(int ms)
    {
        for (int i = 0; i < ms; i++)
        {
            if (Serial.available() >= PACKET_SIZE)
            {
                char packetTypeRx = handleRxPacket();
                if (packetTypeRx == SYNACK)
                {
                    return;
                }
            }
            delay(1);
        }
    }

    void handshake(uint8_t seq)
    {
        isHandshaked = false;
        playConnectionEstablished();
        sendSYNACK();
        // do {
        //   sendSYNACK();
        //   ackTracker.synAck = seq;
        //   waitAck(ACK_TIMEOUT);
        // } while (ackTracker.synAck != NOT_WAITING_FOR_ACK);

        isHandshaked = true;
        kickSeq = seq;
        isHandshaked = true;
    }

    char handleRxPacket()
    {
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
        case SYN:
            handshake(seqReceived);
            break;

        case SYNACK:
            ackTracker.synAck = NOT_WAITING_FOR_ACK;
            break;

        case ACK:
            if (ackTracker.kickAck == seqReceived)
            {
                ackTracker.kickAck = NOT_WAITING_FOR_ACK;
            }
            break;

        default:
            Serial.readString(); // clear the buffer just in case
            return INVALID_PACKET;
            break;
        }
        return packetType;
    default:
        Serial.readString(); // clear the buffer just in case
        return INVALID_PACKET;
        break;
    }
    return packetType;
}
