// #include <Arduino.h>
// #include <IRremote.hpp>
// #include <Tone.h>
// #include <EEPROM.h>
// #include <ArduinoQueue.h>

// // Define I/O pins
// #define IR_EMIT_PIN 3 // IR emitter pin
// #define BUZZER_PIN 5

// // Define constants
// #define IR_SEND_INTERVAL 50  // Interval in ms to send IR signal
// #define PLAYER_ADDRESS 0x02  // Example player address, can be changed
// #define HP_MAX 100           // Maximum HP
// #define SHIELD_HP_MAX 50     // Maximum Shield HP
// #define NOTE_DELAY_DEFAULT 0 // Default note delay

// // Define global variables
// struct Player
// {
//   uint8_t address;
//   uint8_t hp;
//   uint8_t shield_hp;
// } myPlayer;

// typedef struct Sound
// {
//   uint16_t note;
//   uint8_t duration;
// } Sound;

// ArduinoQueue<Sound> soundQueue(10);
// Tone buzzer;
// unsigned long lastIRSendTime = 0;
// unsigned long lastSoundTime = 0;
// uint16_t NOTE_DELAY = NOTE_DELAY_DEFAULT;

// void playHitDetected();
// void playRespawn();
// void setupIR();
// void sendIRData();
// void receive_data(); // Placeholder for external data updates
// void playSoundsFromQueue();

// void setup()
// {
//   Serial.begin(115200);

//   // Set up the player address from EEPROM
//   EEPROM.write(0, PLAYER_ADDRESS);   // Uncomment once to store the address in EEPROM
//   myPlayer.address = EEPROM.read(0); // Read the address from EEPROM

//   // Initialize player HP and Shield HP
//   myPlayer.hp = HP_MAX;
//   myPlayer.shield_hp = SHIELD_HP_MAX;

//   Serial.print(F("Player address: 0x"));
//   Serial.println(myPlayer.address, HEX);
//   Serial.print(F("Initial HP: "));
//   Serial.println(myPlayer.hp);
//   Serial.print(F("Initial Shield HP: "));
//   Serial.println(myPlayer.shield_hp);

//   buzzer.begin(BUZZER_PIN);

//   // Initialize the IR emitter
//   setupIR();

//   Serial.println(F("Vest setup complete"));
// }

// void loop()
// {
//   // Non-blocking IR signal sending
//   if (millis() - lastIRSendTime >= IR_SEND_INTERVAL)
//   {
//     sendIRData();
//     lastIRSendTime = millis();
//   }

//   // External data update (HP and Shield HP)
//   // receive_data(); // Placeholder for updating HP and shield_hp

//   // Play sounds from the queue
//   playSoundsFromQueue();
// }

// void setupIR()
// {
//   // Initialize the IR sender without the default led blinker
//   IrSender.begin(IR_EMIT_PIN);
//   IrSender.IRLedOff(); // Turn off the IR LED blinker
// }

// void sendIRData()
// {
//   // Sending the player address using NEC2 protocol with 0 repeats
//   IrSender.sendNEC(myPlayer.address, 0, 0); // Send the player address via IR with 0 repeats
//   // Serial.print(F("Sent IR data with player address (NEC2, 0 repeats): 0x"));
//   // Serial.println(myPlayer.address, HEX);
// }

// // Queue-based sound playing
// void playSoundsFromQueue()
// {
//   if (millis() - lastSoundTime > NOTE_DELAY)
//   {
//     if (soundQueue.itemCount() > 0)
//     {
//       Sound sound = soundQueue.dequeue();
//       buzzer.play(sound.note, sound.duration);
//       NOTE_DELAY = sound.duration;
//     }
//     lastSoundTime = millis();
//   }
// }

// void playHitDetected(int hp)
// {
//     Sound sound;
//     sound.duration = 100;

//     switch (hp)
//     {
//         case 95:
//             sound.note = NOTE_A5;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_B5;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_C6;
//             soundQueue.enqueue(sound);
//             break;
//         case 90:
//             sound.note = NOTE_G5;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_A5;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_B5;
//             soundQueue.enqueue(sound);
//             break;
//         case 85:
//             sound.note = NOTE_F5;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_G5;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_A5;
//             soundQueue.enqueue(sound);
//             break;
//         case 80:
//             sound.note = NOTE_E5;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_F5;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_G5;
//             soundQueue.enqueue(sound);
//             break;
//         case 75:
//             sound.note = NOTE_D5;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_E5;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_F5;
//             soundQueue.enqueue(sound);
//             break;
//         case 70:
//             sound.note = NOTE_C5;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_D5;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_E5;
//             soundQueue.enqueue(sound);
//             break;
//         case 65:
//             sound.note = NOTE_B4;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_C5;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_D5;
//             soundQueue.enqueue(sound);
//             break;
//         case 60:
//             sound.note = NOTE_A4;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_B4;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_C5;
//             soundQueue.enqueue(sound);
//             break;
//         case 55:
//             sound.note = NOTE_G4;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_A4;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_B4;
//             soundQueue.enqueue(sound);
//             break;
//         case 50:
//             sound.note = NOTE_F4;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_G4;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_A4;
//             soundQueue.enqueue(sound);
//             break;
//         case 45:
//             sound.note = NOTE_E4;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_F4;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_G4;
//             soundQueue.enqueue(sound);
//             break;
//         case 40:
//             sound.note = NOTE_D4;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_E4;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_F4;
//             soundQueue.enqueue(sound);
//             break;
//         case 35:
//             sound.note = NOTE_C4;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_D4;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_E4;
//             soundQueue.enqueue(sound);
//             break;
//         case 30:
//             sound.note = NOTE_B3;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_C4;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_D4;
//             soundQueue.enqueue(sound);
//             break;
//         case 25:
//             sound.note = NOTE_A3;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_B3;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_C4;
//             soundQueue.enqueue(sound);
//             break;
//         case 20:
//             sound.note = NOTE_G3;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_A3;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_B3;
//             soundQueue.enqueue(sound);
//             break;
//         case 15:
//             sound.note = NOTE_F3;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_G3;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_A3;
//             soundQueue.enqueue(sound);
//             break;
//         case 10:
//             sound.note = NOTE_E3;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_F3;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_G3;
//             soundQueue.enqueue(sound);
//             break;
//         case 5:
//             sound.note = NOTE_D3;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_E3;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_F3;
//             soundQueue.enqueue(sound);
//             break;
//         case 0:
//             sound.note = NOTE_C3;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_D3;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_E3;
//             soundQueue.enqueue(sound);
//             break;
//         default:
//             // Optionally handle unexpected hp values here
//             break;
//     }
// }

// void playShieldHitDetected(int shield_hp)
// {
//     Sound sound;
//     sound.duration = 100;

//     switch (shield_hp)
//     {
//         case 45:
//             sound.note = NOTE_G4;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_F4;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_E4;
//             soundQueue.enqueue(sound);
//             break;
//         case 40:
//             sound.note = NOTE_F4;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_E4;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_D4;
//             soundQueue.enqueue(sound);
//             break;
//         case 35:
//             sound.note = NOTE_E4;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_D4;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_C4;
//             soundQueue.enqueue(sound);
//             break;
//         case 30:
//             sound.note = NOTE_D4;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_C4;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_B3;
//             soundQueue.enqueue(sound);
//             break;
//         case 25:
//             sound.note = NOTE_C4;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_B3;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_A3;
//             soundQueue.enqueue(sound);
//             break;
//         case 20:
//             sound.note = NOTE_B3;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_A3;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_G3;
//             soundQueue.enqueue(sound);
//             break;
//         case 15:
//             sound.note = NOTE_A3;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_G3;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_F3;
//             soundQueue.enqueue(sound);
//             break;
//         case 10:
//             sound.note = NOTE_G3;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_F3;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_E3;
//             soundQueue.enqueue(sound);
//             break;
//         case 5:
//             sound.note = NOTE_F3;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_E3;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_D3;
//             soundQueue.enqueue(sound);
//             break;
//         case 0:
//             sound.note = NOTE_E3;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_D3;
//             soundQueue.enqueue(sound);
//             sound.note = NOTE_C3;
//             soundQueue.enqueue(sound);
//             break;
//         default:
//             break;
//     }
// }

// void playRespawn()
// {
//   // Enqueue sounds when the player respawns
//   Sound sound;
//   sound.duration = 80;
//   sound.note = NOTE_C4;
//   soundQueue.enqueue(sound);
//   sound.note = NOTE_C5;
//   soundQueue.enqueue(sound);
//   sound.note = NOTE_C6;
//   soundQueue.enqueue(sound);
// }

// // TODO: Implement external data updates (e.g., from Bluetooth or server)
// void receive_data()
// {
//   // Placeholder function to update HP and Shield HP externally
//   // Example:
//   int prev_hp = myPlayer.hp;
//   int prev_shield_hp = myPlayer.shield_hp;
//   myPlayer.hp = 95;
//   myPlayer.shield_hp = 50;

//   // For now, just print the values (you can replace this with actual data updates)
//   Serial.print(F("Current HP: "));
//   Serial.println(myPlayer.hp);
//   Serial.print(F("Current Shield HP: "));
//   Serial.println(myPlayer.shield_hp);

//   // if player hp increased, they got revived
//   if (myPlayer.hp > prev_hp)
//   {
//     playRespawn();
//   }
//   else if (myPlayer.hp < prev_hp)
//   {
//     playHitDetected(myPlayer.hp);
//   }
//   // in situation where player runs out of shield and loses hp from the attack, hp loss feedback is prioritized
//   else if (myPlayer.shield_hp < prev_shield_hp)
//   {
//     playShieldHitDetected(myPlayer.shield_hp);
//   }
// }
