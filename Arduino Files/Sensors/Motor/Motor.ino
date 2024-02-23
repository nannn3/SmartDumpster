// Motor control pins
const int motorPin1 = 2;  // Motor direction pin 1
const int motorPin2 = 4; // Motor direction pin 2
const int motorSpeedPin = 9; // Motor speed control pin

// Define variables
int motorSpeed = 0; // Motor speed value (0 to 255)

void setup() {
  // Set motor control pins as outputs
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorSpeedPin, OUTPUT);
}

// Function to set motor speed
// Speed sets the speed of the motor
// Motor_In_1 sets IN1 either high or low
// Motor_In_2 sets IN2 either high or low
// Setting one motor in high and the other low changes which direction
// motors are going.
void setMotorSpeed(int speed, int Motor_In_1, int Motor_In_2) {
  // Set motor direction pins, HIGH or LOW
  digitalWrite(motorPin1, Motor_In_1);
  digitalWrite(motorPin2, Motor_In_2);
  
  // Set motor speed
  analogWrite(motorSpeedPin, speed);
}

// Function to read motor speed
// When this would be necessary, Idk, maybe testing?
int readMotorSpeed() {
  return motorSpeed; // Return the current motor speed
}

// Function to stop the motor
// Can be used to reset the motor... though will likely want to slowly
// bring motor back to 0--or add a snubbing diode to help with
// motor inductive kick back causing high voltage
void stopMotor() {
  // Set motor speed to 0
  analogWrite(motorSpeedPin, 0);
}

void loop() {
  // Set motor speed
  setMotorSpeed(128); // Set motor speed to approximately half-speed

  // Read motor speed
  int speed = readMotorSpeed(); // Read motor speed (optional)
  Serial.print("Motor speed: ");
  Serial.println(speed); // Print motor speed to serial monitor (optional)

  // Stop the motor after 2 seconds
  delay(2000);
  stopMotor();
}
