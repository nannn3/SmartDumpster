#include "Config.h"

// #define MOTOR_LOOP

int Motor_init(void){
    // Initialize the L298N control pins as outputs
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);
  pinMode(motorEnA, OUTPUT);
  return true;
}

void motorForward(int speed) {
  analogWrite(motorEnA, speed);  // Set motor speed
  digitalWrite(motorIn1, HIGH);  // Set motor direction
  digitalWrite(motorIn2, LOW);
}

void motorReverse(int speed) {
  analogWrite(motorEnA, speed);  // Set motor speed
  digitalWrite(motorIn1, LOW);   // Set motor direction
  digitalWrite(motorIn2, HIGH);
}

void motorStop() {
  digitalWrite(motorEnA, LOW);  // Disable the motor
  digitalWrite(motorIn1, LOW);  // Set motor direction
  digitalWrite(motorIn2, LOW); // Both low means motor won't move
}

#ifdef MOTOR_LOOP
void setup() {
  // Initialize the L298N control pins as outputs
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);
  pinMode(motorEnA, OUTPUT);
}

void loop() {
  // Spin the motor forward at full speed
  motorForward(127); // Half speed
  delay(2000);

  // Stop the motor
  motorStop();
  delay(1000);

  // Spin the motor in reverse at half speed
  motorReverse(127); // Half speed
  delay(2000);

  // Stop the motor
  motorStop();
  delay(1000);
}
#endif MOTOR_LOOP