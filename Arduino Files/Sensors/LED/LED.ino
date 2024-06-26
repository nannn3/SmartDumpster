// Define the PWM pins for each color channel
const int redPin = 3;    // PWM pin for red LED
const int greenPin = 5;  // PWM pin for green LED
const int bluePin = 6;   // PWM pin for blue LED

// Note: ~ on the Arduino board means PWM cable pins

void setup() {
  // Set PWM pins as outputs
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
}

// Function to set the RGB color, max PWM value is 255 (?)
void setColor(int redValue, int greenValue, int blueValue) {
  analogWrite(redPin, redValue);
  analogWrite(greenPin, greenValue);
  analogWrite(bluePin, blueValue);
}

void loop() {
  // Example: Fade through different colors
  for (int i = 255; i >= 0; i--) {
    // Fade in red
    setColor(i, 255, 255);
    delay(10);
  }
  for (int i = 0; i <= 255; i++) {
    // Fade out red
    setColor(i, 255, 255);
    delay(10);  // Function from Arduino core library, reads miliseconds, currently 0.01 second delay
  }
  for (int i = 255; i >= 0; i--) {
    // Fade in green
    setColor(255, i, 255);
    delay(10);
  }
  for (int i = 0; i <= 255; i++) {
    // Fade out green
    setColor(255, i, 255);
    delay(10);
  }
  for (int i = 255; i >= 0; i--) {
    // Fade in blue
    setColor(255, 255, i);
    delay(10);
  }
  for (int i = 0; i <= 255; i++) {
    // Fade out blue
    setColor(255, 255, i);
    delay(10);
  }
}
