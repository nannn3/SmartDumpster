#ifndef Config_H
#define	Config_H

const int pinA = 2;      // Encoder A output
const int pinB = 7;      // Encoder B output
const int pinIndex = 4;  // Index output, if used

// Define the Arduino analog pin connected to the force sensor output
const int sensorPin1 = A0;
const int sensorPin2 = A1;

// Define the PWM pins for each color channel
const int redPin = 3;    // PWM pin for red LED
const int greenPin = 5;  // PWM pin for green LED
const int bluePin = 6;   // PWM pin for blue LED

// Define Arduino pin numbers connected to the L298N module
const int motorIn1 = 9;  // IN1 on the L298N
const int motorIn2 = 10;   // IN2 on the L298N
const int motorEnA = 11;  // ENA on the L298N for PWM speed control

#endif	/* Config_H */