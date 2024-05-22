#include "Config.h"

// #define MOTOR_LOOP

// initializes pins for the motor
int Motor_init(void){
    // Initialize the L298N control pins as outputs
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);
  pinMode(motorEnA, OUTPUT);
  return true;
}

// sets given motor speed, and sets input A high and B low to go forward
void motorForward(int speed) {
  digitalWrite(motorEnA, HIGH);  // Set motor speed
  // analogWrite(motorEnA, speed);  // Set motor speed
  digitalWrite(motorIn1, HIGH);  // Set motor direction
  digitalWrite(motorIn2, LOW);
}

// sets given motor speed, and sets input A low and B high to go backward
void motorReverse(int speed) {
  analogWrite(motorEnA, speed);  // Set motor speed
  digitalWrite(motorIn1, LOW);   // Set motor direction
  digitalWrite(motorIn2, HIGH);
}

// stops motor
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
  // Spin the motor forward
  motorForward(halfSpeed);
  // delay(2000);

  // Stop the motor
  // motorStop();
  // delay(1000);

  // Spin the motor in reverse at half speed
  // motorReverse(halfSpeed);
  // delay(2000);

  // Stop the motor
  // motorStop();
  // delay(1000);
}
#endif MOTOR_LOOP