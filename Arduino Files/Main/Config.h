#include <VL53L0X.h>

#ifndef Config_H
#define Config_H

/* Important info
// red LED color when error occurs
// orange LED color when contaminant bin is full
// yellow LED color when bio bin is full
// white LED color when conveyor belt is stopped but system is live and awaiting commands, or arm is sorting
// green LED color when conveyor belt is actively moving

// SDA is pin A4
// SCL is pin A5
*/

#define maxSpeed 255
#define halfSpeed 127

typedef enum {
  START,         // starts conveyor belt state machine
  STOP,          // stops conveyor belt system and sends back to start state
  fod,           // camera detects foreign object debris
  failedtoinit,  // inits failed to initialize
  cfi            // computer vision failed to initialize
} Op_Codes;      // codes a user/computer vision can send to operate the conveyor belt state machine

// struct for error types
typedef enum {
  error_clear,
  error_FOD,
  error_FailedToInit,
  error_FailedEncoderInit,
  error_FailedLEDInit,
  error_FailedMotorInit,
  error_FailedTOFInit
} error_ops;

// struct for conveyor belt state machine
typedef enum {
  start,         // start and idle state
  sorting,       // active sorting state
  bio_bin_full,
  cont_bin_full,
  error
} conveyor_states;

// struct for error state machine
typedef enum {
  clear,         // idle state
  searching,     // state to check error type, redundant
  failedToInitialize,
  FOD
} error_states;

const int pinA = 2;      // Encoder A output
const int pinB = 7;      // Encoder B output
const int pinIndex = 4;  // Index output, if used

// Define the Arduino analog pin connected to the force sensor output
const int sensorPin1 = A0; // force sensor 1
const int sensorPin2 = A1; // force sensor 2

// Define the PWM pins for each color channel
const int redPin = 3;    // PWM pin for red LED
const int greenPin = 5;  // PWM pin for green LED
const int bluePin = 6;   // PWM pin for blue LED

// Define Arduino pin numbers connected to the L298N module
const int motorIn1 = 9;   // IN1 on the L298N
const int motorIn2 = 10;  // IN2 on the L298N
const int motorEnA = 11;  // ENA on the L298N for PWM speed control

// Initializing ToF sensor
VL53L0X sensor1;          // instance of ToF sensor1
VL53L0X sensor2;          // instance of ToF sensor2
int xshut1 = 8;           // xshut for sensor1
int xshut2 = 12;          // xshut for sensor2

// Force Sensor Functions
// a function that takes a raw reading value from an analog pin
float readForceSensor(float forcePin);
// a function that takes an adc value from the analog pin and converts it to force
float convertToForce(float V2);

// Encoder Sensor Functions
// a function to initialize encoder pins and the interrupt
int Encoder_init(void);
// get the position of the encoder in a given moment
int GetPosition(void);
// resets the position of the encoder and sets to 0
void ResetPosition();
// converts encoder readings to degrees of a circle
int encoder_to_degrees(int value);
// reads the A, B, and Index of encoder and calculates the position of the encoder based of the three values
void readEncoder();

// LED functions
// initializes the LED pins
int LED_init(void);
// Function to set the RGB color, max PWM value is 255 (?)
// sets PWM signals for red, green and blue pins
void setColor(int redValue, int greenValue, int blueValue);

// Motor functions
// initializes pins for the motor
int Motor_init(void);
// sets given motor speed, and sets input A high and B low to go forward
void motorForward(int speed);
// sets given motor speed, and sets input A low and B high to go backward
void motorReverse(int speed);
// stops motor
void motorStop();

// ToF Sensor Functions
// initializes pins, instances, and sets the address for the sensors. Initializes the I2C
int ToF_init(void);
// returns distance in cm
uint16_t readDistance(VL53L0X &sensor);

#endif 