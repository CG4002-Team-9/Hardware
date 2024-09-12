#include <IRremote.hpp>  // Include the IRSend library

// Pin assignments
#define IR_EMITTER_PIN 3  // IR emitter pin
#define BUZZER_PIN 5      // Buzzer pin

// Constants for IR emission
const int IR_EMISSION_FREQ = 38000; // 38kHz is common for IR emitter

// Buzzer queue variables
const int QUEUE_SIZE = 2; // Buzzer queue size

struct BuzzerSound {
  int frequency;
  int duration;
};

BuzzerSound buzzerQueue[QUEUE_SIZE];
int queueStart = 0;
int queueEnd = 0;
unsigned long lastBuzzerTime = 0;  // Time of the last buzzer sound

// Variables for server data
int audio_num = 0;  // Indicate which sound byte to play

// Initialize IRSend object
IRsend irsend;

unsigned long lastIRSendTime = 0;

void setup() {
  // Initialize Serial communication
  Serial.begin(115200);

  // Initialize pins
  pinMode(BUZZER_PIN, OUTPUT);

  // Initialize IR emitter
  irsend.begin(IR_EMITTER_PIN);  // Initialize the IR emitter
}

void loop() {
  // Check for incoming data from the server
  //if (Serial.available() >= sizeof(PlayerState)) {
    //PlayerState playerState;
    //Serial.readBytes((char*)&playerState, sizeof(PlayerState));

    // Extract audio file index from the struct
    //audio_num = playerState.audio;

    // Play the buzzer sound corresponding to audio_num
    //playBuzzer(audio_num);
  //}

  irsend.sendNEC(0x00, 0x11, 0);

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

void playBuzzer(int audio_num) {
  // case 1-20 when player is not shielded. case 21-40 when player is shielded.
  switch (audio_num) {
    case 1:  // hp > 95%
      addToBuzzerQueue(5000, 200);
      break;
    case 2:  // 90-95% hp
      addToBuzzerQueue(4850, 200);
      break;
    case 3:  // 85-90% hp
      addToBuzzerQueue(4700, 200);
      break;
    case 4:  // 80-85% hp
      addToBuzzerQueue(4550, 200);
      break;
    case 5:  // 75-80% hp
      addToBuzzerQueue(4400, 200);
      break;
    case 6:  // 70-75% hp
      addToBuzzerQueue(4250, 200);
      break;
    case 7:  // 65-70% hp
      addToBuzzerQueue(4100, 200);
      break;
    case 8:  // 60-65% hp
      addToBuzzerQueue(3950, 200);
      break;
    case 9:  // 55-60% hp
      addToBuzzerQueue(3800, 200);
      break;
    case 10: // 50-55% hp
      addToBuzzerQueue(3650, 200);
      break;
    case 11: // 45-50% hp
      addToBuzzerQueue(3500, 200);
      break;
    case 12: // 40-45% hp
      addToBuzzerQueue(3350, 200);
      break;
    case 13: // 35-40% hp
      addToBuzzerQueue(3200, 200);
      break;
    case 14: // 30-35% hp
      addToBuzzerQueue(3050, 200);
      break;
    case 15: // 25-30% hp
      addToBuzzerQueue(2900, 200);
      break;
    case 16: // 20-25% hp
      addToBuzzerQueue(2750, 200);
      break;
    case 17: // 15-20% hp
      addToBuzzerQueue(2600, 200);
      break;
    case 18: // 10-15% hp
      addToBuzzerQueue(2450, 200);
      break;
    case 19: // 5-10% hp
      addToBuzzerQueue(2300, 200);
      break;
    case 20: // 0-5% hp
      addToBuzzerQueue(2150, 200);
      break;
    case 21: // sp > 95%
      addToBuzzerQueue(5000, 200);
      addToBuzzerQueue(5000, 200);
      break;
    case 22: // 90-95% sp
      addToBuzzerQueue(4850, 200);
      addToBuzzerQueue(4850, 200);
      break;
    case 23: // 85-90% sp
      addToBuzzerQueue(4700, 200);
      addToBuzzerQueue(4700, 200);
      break;
    case 24: // 80-85% sp
      addToBuzzerQueue(4550, 200);
      addToBuzzerQueue(4550, 200);
      break;
    case 25: // 75-80% sp
      addToBuzzerQueue(4400, 200);
      addToBuzzerQueue(4400, 200);
      break;
    case 26: // 70-75% sp
      addToBuzzerQueue(4250, 200);
      addToBuzzerQueue(4250, 200);
      break;
    case 27: // 65-70% sp
      addToBuzzerQueue(4100, 200);
      addToBuzzerQueue(4100, 200);
      break;
    case 28: // 60-65% sp
      addToBuzzerQueue(3950, 200);
      addToBuzzerQueue(3950, 200);
      break;
    case 29: // 55-60% sp
      addToBuzzerQueue(3800, 200);
      addToBuzzerQueue(3800, 200);
      break;
    case 30: // 50-55% sp
      addToBuzzerQueue(3650, 200);
      addToBuzzerQueue(3650, 200);
      break;
    case 31: // 45-50% sp
      addToBuzzerQueue(3500, 200);
      addToBuzzerQueue(3500, 200);
      break;
    case 32: // 40-45% sp
      addToBuzzerQueue(3350, 200);
      addToBuzzerQueue(3350, 200);
      break;
    case 33: // 35-40% sp
      addToBuzzerQueue(3200, 200);
      addToBuzzerQueue(3200, 200);
      break;
    case 34: // 30-35% sp
      addToBuzzerQueue(3050, 200);
      addToBuzzerQueue(3050, 200);
      break;
    case 35: // 25-30% sp
      addToBuzzerQueue(2900, 200);
      addToBuzzerQueue(2900, 200);
      break;
    case 36: // 20-25% sp
      addToBuzzerQueue(2750, 200);
      addToBuzzerQueue(2750, 200);
      break;
    case 37: // 15-20% sp
      addToBuzzerQueue(2600, 200);
      addToBuzzerQueue(2600, 200);
      break;
    case 38: // 10-15% sp
      addToBuzzerQueue(2450, 200);
      addToBuzzerQueue(2450, 200);
      break;
    case 39: // 5-10% sp
      addToBuzzerQueue(2300, 200);
      addToBuzzerQueue(2300, 200);
      break;
    case 40: // 0-5% sp
      addToBuzzerQueue(2150, 200);
      addToBuzzerQueue(2150, 200);
      break;
    default:
      break;
  }
}
