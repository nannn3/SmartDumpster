#include "Config.h"

// #define ENCODER_LOOP
#define MAX_TICKS 5120

int encoder_count = 0;
int current_state = 0;
int last_read = 0;

typedef enum {
  state_1 = 0,
  state_2 = 1,
  state_3 = 2,
  state_4 = 3
} conveyor_states;

int Encoder_init(void) {
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  //Reading interrupt for only pin A, since UNO only has two capable interrupt pins and we want to save for potential other uses.
  attachInterrupt(digitalPinToInterrupt(pinA), readEncoder, CHANGE);
  return true;
}

int GetPosition(void) {
  return encoder_count;
}

void ResetPosition() {
  encoder_count = 0;
  current_state = 0;
}

int encoder_to_degrees(int value) {
  int degrees = value * 0.0703125;
  return degrees;
}

void readEncoder() {
  int A = digitalRead(pinA);
  int B = digitalRead(pinB);
  static int lastA = 0;
  static int lastB = 0;

  // Determine direction based on A and B
  if (lastA == 0 && A == 1) {
    if (B == 0) encoder_count++;  // Moving forward
    else encoder_count--;         // Moving backward
  } else if (lastA == 1 && A == 0) {
    if (B == 0) encoder_count--;  // Moving backward
    else encoder_count++;         // Moving forward
  }

  lastA = A;
  lastB = B;

  // handle overflow/underflow or reset conditions here
  if (encoder_count >= MAX_TICKS || encoder_count <= -MAX_TICKS) ResetPosition();
}

#ifdef ENCODER_LOOP
void setup() {
  Serial.begin(115200);

  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  //Reading interrupt for only pin A, since UNO only has two capable interrupt pins and we want to save for potential other uses.
  attachInterrupt(digitalPinToInterrupt(pinA), readEncoder, CHANGE);
}

void loop() {
  int current_count = GetPosition();
  if (last_read != current_count) {
    Serial.print("Count: ");
    Serial.println(current_count);
    last_read = current_count;
  }
  delay(100);
}
#endif
