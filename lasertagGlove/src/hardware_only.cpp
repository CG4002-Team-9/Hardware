// #include <Arduino.h>
// #include <IRremote.hpp>
// #include <MPU6050.h>
// #include <ArduinoQueue.h>
// #include <Tone.h>
// #include <EEPROM.h>

// // Define I/O pins
// #define DECODE_NEC // Enable NEC protocol. This is the protocol used for the IR receiver
// #define IMU_INTERRUPT_PIN 2
// #define BUTTON_PIN 3
// #define IR_RECEIVE_PIN 4 // receiver is the gun
// #define BUZZER_PIN 5

// // Define constants
// #define DEBOUNCE_DELAY 200
// #define MPU_SAMPLING_RATE 40 // in Hz
// #define IR_SEARCH_TIMEOUT 200
// #define NUM_SAMPLES 30

// // Define global variables

// struct Player
// {
//     uint8_t address;
//     uint8_t bullets = 6;
// } myPlayer;

// typedef struct Sound
// {
//     uint16_t note;
//     uint8_t duration;
// } Sound;

// struct mpuCalibration
// {
//     int16_t ax_offset;
//     int16_t ay_offset;
//     int16_t az_offset;
//     int16_t gx_offset;
//     int16_t gy_offset;
//     int16_t gz_offset;
// } mpuCal;

// ArduinoQueue<Sound> soundQueue(10);

// Tone buzzer;

// MPU6050 mpu;
// const unsigned long SAMPLING_DELAY = 1000 / MPU_SAMPLING_RATE;
// uint8_t mpuSamples = 0;

// bool isButtonPressed = false;
// bool isMotionDetected = false;
// bool isFindingIR = false;

// // millis variables
// unsigned long lastDebounceTime = 0;
// unsigned long lastSoundTime = 0;
// uint16_t NOTE_DELAY = 0;
// unsigned long lastSampleTime = 0;
// unsigned long irStartTime = 0;

// // put function declarations here:
// void motionDetected();
// void buttonPressed();
// void playMotionDetected();
// void playMotionEnded();
// void playSuccessfulShot();
// void playSuccessfulReload();
// void playShootBullet(uint8_t bullets);
// void playEmptyGun();
// void sendShotDataToServer(bool hitDetected, uint8_t playerHit); // could queue into a buffer which sends in a subroutine
// void sendIMUDataToServer(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz);

// void setup()
// {
//     Serial.begin(115200);
//     // write to EEPROM the player address
//     // EEPROM.write(0, 0x01);             // only do this once, then comment out
//     myPlayer.address = EEPROM.read(0); // read the address from EEPROM

//     // save to EEPROM the calibration values
//     // mpuCal.ax_offset = -1633;
//     // mpuCal.ay_offset = -253;
//     // mpuCal.az_offset = 144;
//     // mpuCal.gx_offset = 47;
//     // mpuCal.gy_offset = -110;
//     // mpuCal.gz_offset = 15;
//     // EEPROM.put(1, mpuCal); // only do this once, then comment out

//     // read from EEPROM and set the calibration values
//     mpuCal = EEPROM.get(1, mpuCal);
//     mpu.setXAccelOffset(mpuCal.ax_offset);
//     mpu.setYAccelOffset(mpuCal.ay_offset);
//     mpu.setZAccelOffset(mpuCal.az_offset);
//     mpu.setXGyroOffset(mpuCal.gx_offset);
//     mpu.setYGyroOffset(mpuCal.gy_offset);
//     mpu.setZGyroOffset(mpuCal.gz_offset);

//     Serial.print(F("Player address: 0x"));
//     Serial.println(myPlayer.address, HEX);

//     buzzer.begin(BUZZER_PIN);

//     mpu.initialize();
//     if (!mpu.testConnection())
//     {
//         Serial.println(F("MPU6050 connection failed"));
//         while (1)
//             ;
//     }

//     mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
//     mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);

//     mpu.setDHPFMode(MPU6050_DHPF_0P63);
//     mpu.setDLPFMode(MPU6050_DLPF_BW_20);

//     mpu.setMotionDetectionThreshold(50);
//     mpu.setMotionDetectionDuration(5);
//     mpu.setIntMotionEnabled(true);

//     pinMode(IMU_INTERRUPT_PIN, INPUT);
//     attachInterrupt(digitalPinToInterrupt(IMU_INTERRUPT_PIN), motionDetected, RISING);

//     pinMode(BUZZER_PIN, OUTPUT);

//     // interrupt for button press both rising and falling
//     pinMode(BUTTON_PIN, INPUT_PULLUP);
//     attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonPressed, FALLING);

//     pinMode(IR_RECEIVE_PIN, INPUT);
//     IrReceiver.begin(IR_RECEIVE_PIN);
//     playSuccessfulReload();
//     Serial.println(F("Setup complete"));
// }

// void loop()
// {
//     // buttonpress subroutine
//     if (isButtonPressed)
//     {
//         if (myPlayer.bullets > 0)
//         {
//             playShootBullet(myPlayer.bullets);
//             myPlayer.bullets--;

//             isFindingIR = true;

//             Serial.print(F("Bullet fired. Bullets left: "));
//             Serial.println(myPlayer.bullets);
//         }
//         else
//         {
//             playEmptyGun();
//             Serial.println(F("Attempted to fire but the gun is empty."));
//         }
//         isButtonPressed = false;
//     }

//     // sound playing subroutine
//     if (millis() - lastSoundTime > NOTE_DELAY)
//     {
//         if (soundQueue.itemCount() > 0)
//         {
//             Sound sound = soundQueue.dequeue();
//             buzzer.play(sound.note, sound.duration);
//             NOTE_DELAY = sound.duration;
//         }
//         else if (soundQueue.itemCount() == 0)
//         {
//             IrReceiver.restartTimer();
//         }
//         lastSoundTime = millis();
//     }

//     // ir data collect subroutine
//     if (isFindingIR)
//     {
//         IrReceiver.start();
//         irStartTime = millis();
//         uint8_t playerHit = 0;
//         bool hitDetected = false;
//         while (millis() - irStartTime < IR_SEARCH_TIMEOUT)
//         {
//             if (IrReceiver.decode())
//             {
//                 // print all the data of the received IR signal
//                 IrReceiver.printIRResultShort(&Serial, true);

//                 // if received data is valid with NEC2, enqueue the address
//                 if (IrReceiver.decodedIRData.protocol != UNKNOWN && IrReceiver.decodedIRData.address != myPlayer.address)
//                 {
//                     hitDetected = true;
//                     playerHit = IrReceiver.decodedIRData.address;
//                     break;
//                 }
//                 IrReceiver.resume();
//             }
//         }

//         if (hitDetected)
//         {
//             Serial.print(F("Hit detected on player: 0x"));
//             Serial.println(playerHit, HEX);
//             playSuccessfulShot();
//         }
//         else
//         {
//             Serial.println(F("No hit detected."));
//         }

//         sendShotDataToServer(hitDetected, playerHit);
//         isFindingIR = false;
//         IrReceiver.stop();
//     }

//     // mpu data collect subroutine
//     if (isMotionDetected && millis() - lastSampleTime > SAMPLING_DELAY)
//     {
//         int16_t ax, ay, az;
//         int16_t gx, gy, gz;
//         mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

//         Serial.print("a/g:\t");
//         Serial.print(ax);
//         Serial.print("\t");
//         Serial.print(ay);
//         Serial.print("\t");
//         Serial.print(az);
//         Serial.print("\t");
//         Serial.print(gx);
//         Serial.print("\t");
//         Serial.print(gy);
//         Serial.print("\t");
//         Serial.println(gz);

//         lastSampleTime = millis();
//         if (mpuSamples++ >= NUM_SAMPLES)
//         {
//             sendIMUDataToServer(ax, ay, az, gx, gy, gz);
//             mpuSamples = 0;
//             isMotionDetected = false;
//             playMotionEnded();
//         }
//     }
// }

// // put function definitions here:

// void motionDetected()
// {
//     if (!isMotionDetected)
//     {
//         isMotionDetected = true;
//         playMotionDetected();
//     }
// }

// void buttonPressed()
// {
//     if (millis() - lastDebounceTime > DEBOUNCE_DELAY && !isFindingIR && !isButtonPressed)
//     {
//         isButtonPressed = true;
//         lastDebounceTime = millis();
//     }
// }

// void playSuccessfulShot()
// {
//     // play two ascending sounds
//     Sound sound;
//     sound.note = 0;
//     sound.duration = 100;
//     soundQueue.enqueue(sound);
//     sound.duration = 80;
//     sound.note = NOTE_AS5;
//     soundQueue.enqueue(sound);
//     sound.note = NOTE_B5;
//     soundQueue.enqueue(sound);
// }

// void playSuccessfulReload()
// {
//     // play four ascending sounds
//     Sound sound;
//     sound.duration = 50;
//     sound.note = NOTE_C6;
//     soundQueue.enqueue(sound);
//     sound.note = NOTE_CS6;
//     soundQueue.enqueue(sound);
//     sound.note = NOTE_D6;
//     soundQueue.enqueue(sound);
//     sound.note = NOTE_DS6;
//     soundQueue.enqueue(sound);
// }

// void playShootBullet(uint8_t bullets)
// {
//     // play a sound based on the number of bullets left
//     Sound sound;

//     switch (bullets)
//     {
//     case 1:
//         sound.note = NOTE_C4;
//         break;
//     case 2:
//         sound.note = NOTE_CS4;
//         break;
//     case 3:
//         sound.note = NOTE_D4;
//         break;
//     case 4:
//         sound.note = NOTE_DS4;
//         break;
//     case 5:
//         sound.note = NOTE_E4;
//         break;
//     case 6:
//         sound.note = NOTE_F4;
//         break;
//     default:
//         break;
//     }
//     sound.duration = 150;
//     soundQueue.enqueue(sound);
// }

// void playEmptyGun()
// {
//     // play three descending low sounds
//     Sound sound;
//     sound.duration = 50;
//     sound.note = NOTE_A3;
//     soundQueue.enqueue(sound);
//     sound.note = NOTE_G3;
//     soundQueue.enqueue(sound);
//     sound.note = NOTE_F3;
//     soundQueue.enqueue(sound);
// }

// void playMotionDetected()
// {
//     // play a sound to indicate motion detected
//     Sound sound;
//     sound.duration = 255;
//     sound.note = NOTE_AS6;
//     soundQueue.enqueue(sound);
// }

// void playMotionEnded()
// {
//     // play a sound to indicate motion ended
//     Sound sound;
//     sound.duration = 255;
//     sound.note = NOTE_A6;
//     soundQueue.enqueue(sound);
// }

// void sendShotDataToServer(bool hitDetected, uint8_t playerHit)
// {
//     // TODO: Implement this function
// }

// void sendIMUDataToServer(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz)
// {
//     // TODO: Implement this function
// }
