#include <IRremote.hpp>  // Include the IRSend library

// Pin assignments
#define IR_EMITTER_PIN 3  // IR emitter pin
#define BUZZER_PIN 5      // Buzzer pin

// Constants for IR emission
const int IR_EMISSION_FREQ = 38000; // 38kHz is common for IR emitter

// Buzzer queue variables
const int QUEUE_SIZE = 3; // Buzzer queue size

struct BuzzerSound {
  int frequency;
  int duration;
};

BuzzerSound buzzerQueue[QUEUE_SIZE];
int queueStart = 0;
int queueEnd = 0;
unsigned long lastBuzzerTime = 0;  // Time of the last buzzer sound

int hp = 100;
int sp = 30;

// Initialize IRSend object
IRsend irsend;

unsigned long lastIRSendTime = 0;

void setup() {
  // Initialize Serial communication
  Serial.begin(115200);

  // Initialize pins
  pinMode(BUZZER_PIN, OUTPUT);

  // Initialize IR emitter
  irsend.begin(IR_EMITTER_PIN);

}

void loop() {
  // Check for incoming data from the server
  //if (Serial.available() >= sizeof(PlayerState)) {
    //PlayerState playerState;
    //Serial.readBytes((char*)&playerState, sizeof(PlayerState));

    // refresh hp and sp on board with latest values
    //int prev_sp = sp;
    //int prev_hp = hp;
    //hp = playerState.hp;
    //sp = playerState.sp;
    //if (hp > prev_hp) {
      //playReviveBuzzer();
    //} else if (sp < prev_sp) {
      // player got hit while shielding
      //playShieldBuzzer(sp);
    //} else if (hp < prev_hp) {
      // player hit while not shielding
      //playHealthBuzzer(hp);
    //}
  //}
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); // Read the incoming command

    int prev_sp = sp;
    int prev_hp = hp;
    // Process the serial command
    if (command == "send shielded hit") {
      sp = 25;
    }
    else if (command == "send unshielded hit") {
      hp = 95;
    }
    else if (command == "revive") {
      hp = 100;
      sp = 30;
    }
    
    if (hp > prev_hp) {
      playReviveBuzzer();
    } else if (sp < prev_sp) {
      // player got hit while shielding
      playShieldBuzzer(sp);
    } else if (hp < prev_hp) {
      // player hit while not shielding
      playHealthBuzzer(hp);
    }
  }

  irsend.sendNEC(0x00, 0x59, 0);

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

void playHealthBuzzer(int hp) {
  int freq = 2000 + hp * 30; //scale sound to remaining hp
  addToBuzzerQueue(freq, 200);
}

void playShieldBuzzer(int sp) {
  int freq = 2000 + sp * 30; //scale sound to remaining sp
  addToBuzzerQueue(freq, 200);
  addToBuzzerQueue(freq, 200);
}

void playReviveBuzzer() {
  addToBuzzerQueue(2000,200);
  addToBuzzerQueue(5000,200);
}
