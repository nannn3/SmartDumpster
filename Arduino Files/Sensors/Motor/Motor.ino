// Define Arduino pin numbers connected to the L298N module
const int motorIn1 = 12;  // IN1 on the L298N
const int motorIn2 = 9;   // IN2 on the L298N
const int motorEnA = 11;  // ENA on the L298N for PWM speed control

void setup() {
  // Initialize the L298N control pins as outputs
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);
  pinMode(motorEnA, OUTPUT);
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
