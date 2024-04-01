
#define MAX_TICKS 5120

int encoder_count = 0;
const int pinA = 2;      // Encoder A output
const int pinB = 7;      // Encoder B output
const int pinIndex = 4;  // Index output, if used
int current_state = 0;
int last_read = 0;

typedef enum {
  state_1 = 0,
  state_2 = 1,
  state_3 = 2,
  state_4 = 3
} encoder_states;

void setup() {
  Serial.begin(115200);

  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  // Only attach an interrupt to pinA
  attachInterrupt(digitalPinToInterrupt(pinA), readEncoder, CHANGE);
}
// void setup() {
//   Serial.begin(115200);

//   pinMode(pinA, INPUT);
//   pinMode(pinB, INPUT);
//   attachInterrupt(digitalPinToInterrupt(pinA), readEncoder, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(pinB), readEncoder, CHANGE);
// }

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

  // Optionally handle overflow/underflow or reset conditions here
  // Example: Reset position if exceeds MAX_TICKS or goes below -MAX_TICKS
  if (encoder_count >= MAX_TICKS || encoder_count <= -MAX_TICKS) ResetPosition();
}
// void readEncoder() {
//   int A = digitalRead(pinA);
//   int B = digitalRead(pinB);
//   if (encoder_count == MAX_TICKS || encoder_count == -MAX_TICKS) {
//     ResetPosition();
//   }

//   //anything else that needs to happen goes here
//   switch (current_state) {
//     case state_1:
//       if ((A == 1) && (B == 0)) {
//         current_state = state_2;
//         encoder_count = (encoder_count - 1) % MAX_TICKS;
//       } else if ((A == 0) && (B == 1)) {
//         current_state = state_4;
//         encoder_count = (encoder_count + 1) % MAX_TICKS;
//       }
//       break;
//     case state_2:
//       if ((A == 0) && (B == 0)) {
//         current_state = state_3;
//       } else if ((A == 1) && (B == 1)) {
//         current_state = state_1;
//       }
//       break;
//     case state_3:
//       if ((A == 0) && (B == 1)) {
//         current_state = state_4;
//       } else if ((A == 1) && (B == 0)) {
//         current_state = state_2;
//       }
//       break;
//     case state_4:
//       if ((A == 1) && (B == 1)) {
//         current_state = state_1;
//       } else if ((A == 0) && (B == 0)) {
//         current_state = state_3;
//       }
//       break;
//   }
// }

void loop() {
  int current_count = GetPosition();
  if (last_read != current_count) {
    Serial.print("Count: ");
    Serial.println(current_count);
    last_read = current_count;
  }
  delay(100);
}
