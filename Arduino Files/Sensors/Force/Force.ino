// Define the Arduino analog pin connected to the sensor output
const int sensorPin1 = A0;
const int sensorPin2 = A1;
const int sensorPin3 = A2;

// Define the supply voltage and fixed resistor value
const float Vr = 5.0;    // Assuming a 5V supply voltage
const float R1 = 10000;  // 47k ohm fixed resistor

// #include "Config.h"

#define FORCE_LOOP

// moving average for force sensor
const int numReadings = 256;  // Number of readings to average
float readings[numReadings];   // Array to store the readings
int readIndex = 0;           // Index of the current reading
float total = 0;               // Running total of the readings
float average = 0;             // The average of the readings

// state machine sensor value readings
float prevFor1 = 0;
float newFor1 = 0;

float MovingAverage(float prev_For1) {
  total = 0;
  readings[readIndex] = prev_For1;  // Read from the sensor
  for (int i = 0; i < numReadings; i++) {
    total += readings[i];
  }                                           // Add the newest reading to the total
  readIndex = (readIndex + 1) % numReadings;  // Advance to the next position in the array

  // Calculate the average:
  average = total / numReadings;
  return average;
}

// Example conversion function - needs actual implementation
float convertToForce(float V2) {
  // float force = (0.0107 * V2) - 0.0594;
  float force = (0.0105 * V2) - 0.959;
  // float force = (0.00868 * V2) - 0.518;
  return force;
}

float readForceSensor(float forcePin) {
  float voltage = analogRead(forcePin);
  return voltage;
}

#ifdef FORCE_LOOP
void setup() {
  // Start serial communication
  Serial.begin(115200);
}

void loop() {
  // // Read the sensor voltage
  // float V2_1 = analogRead(sensorPin1);  // reads 0 - 1023
  // float V2_2 = analogRead(sensorPin2);

  // // Convert Rs to force in lbs
  newFor1 = readForceSensor(sensorPin1);
  // new_For1 = convertToForce(readForceSensor(sensorPin1));
  if (prevFor1 != newFor1) {
    prevFor1 = newFor1;
    float number = MovingAverage(prevFor1);
    Serial.println(number);
      }
  // Serial.println(force1);
  // Serial.println(force1);
  // Delay for a bit to avoid spamming
}
#endif
