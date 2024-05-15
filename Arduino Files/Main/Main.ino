#include "Config.h"

// red LED color when error occurs
// orange LED color when contaminant bin is full
// yellow LED color when arm is actively sorting
// green LED color when conveyor belt is actively moving

// #define MAIN_LOOP
#define START 1           // used to tell the program to start the operations of the conveyor belt
#define STOP 0            // used to tell the program to stop the operations of the conveyor belt
#define ToF_THRESHOLD 10  // change this value to show desired minimum desitance of ToF reading in mm
#define F_THRESHOLD 700   // change this value to show max weight allowable for user to lift, 500 = raw value adc, 5 lbs

int operation_flag = STOP;     // initial state of the operation should be stop
int currentConveyorState = 0;  // initialize conveyor belt state machine to start
int error_flag = false;        // starting error state
int currentErrorState = 0;     // initialize error state machine to clear
int error_type = 0;

// moving average for force sensor
const int numReadings = 10;  // Number of readings to average
int readings[numReadings];   // Array to store the readings
int readIndex = 0;           // Index of the current reading
int total = 0;               // Running total of the readings
int average = 0;             // The average of the readings

// place holders
int ToF_reading = 0;
int force_reading = 0;

// state machine sensor value readings
int prev_ToF1 = 0;
int prev_For1 = 0;
int new_ToF1 = 0;
int new_For1 = 0;

// await command for start machine
int wait = 0;

// struct for conveyor belt state machine
typedef enum {
  start,
  sorting,
  bin_full,
  error
} conveyor_states;

// struct for error state machine
typedef enum {
  clear,
  searching,
  failedToInitialize,
  FOD
} error_states;

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

int ClearMovingAverage() {
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;  // Initialize all the readings to 0
  }
}

int Main_init(void) {
  Serial.begin(115200);
  Encoder_init();
  LED_init();
  Motor_init();
  ToF_init();
  if (Encoder_init() != true) {
    error_flag = true;
    error_type = error_FailedEncoderInit;
    operation_flag = STOP;
    return false;
  }
  if (LED_init() != true) {
    error_flag = true;
    error_type = error_FailedLEDInit;
    operation_flag = STOP;
    return false;
  }
  if (Motor_init() != true) {
    error_flag = true;
    error_type = error_FailedMotorInit;
    operation_flag = STOP;
    return false;
  }
  if (ToF_init() != true) {
    error_flag = true;
    error_type = error_FailedTOFInit;
    operation_flag = STOP;
    return false;
  }
  return true;
}

int checkOperationStatus(void) {
  if (Serial.available() > 0) {
    // read the incoming byte
    String command = Serial.readStringUntil('\n');  // read the command until the newline
    command.trim();                                 //remove the whitespace and newline

    // set the flag high or low
    if (command == "start" | command == "Start" | command == "START") {
      operation_flag = START;
      // echo the command back to the serial monitor
      Serial.print("Command received: ");
      Serial.println(command);
    } else if (command == "stop" | command == "Stop" | command == "STOP") {
      operation_flag = STOP;
      // echo the command back to the serial monitor
      Serial.print("Command received: ");
      Serial.println(command);
    } else if (command == "fod" | command == "Fod" | command == "FOD" | command == "FoD") {
      operation_flag = STOP;
      error_flag = true;
      error_type = error_FOD;
      // echo the command back to the serial monitor
      Serial.print("Command received: ");
      Serial.println(command);
    } else if (command == "FailedToInitialize" | command == "failedtoinitialize" | command == "FAILEDTOINITIALIZE" | command == "Failedtoinitialze") {
      operation_flag = STOP;
      error_flag = true;
      error_type = error_FailedToInit;
      // echo the command back to the serial monitor
      Serial.print("Command received: ");
      Serial.println(command);
    } else if (command == "Clear" | command == "clear" | command == "CLEAR") {
      operation_flag = STOP;
      error_flag = false;
      error_type = error_clear;
      // echo the command back to the serial monitor
      Serial.print("Command received: ");
      Serial.println(command);
    } else {
      // print message for error input
      Serial.println("Error: Invalid Response!");
    }
  }
}

void runErrorStateMachine() {
  switch (currentErrorState) {
    case clear:
      if (error_flag = true) {
        currentErrorState = searching;
      } else {
        currentErrorState = clear;
      }
      break;
    case searching:
      if (error_type == "FOD") {
        Serial.println("Clear FOD in Active Area!");
        currentErrorState = FOD;
      } else if (error_type == "FailedToInitialize") {
        Serial.println("Failed to Initialize!");
        currentErrorState = failedToInitialize;
      }
      break;
    case failedToInitialize:
      // check to see if all sensors, arm, and camera initialize completely
      if (Main_init() == true) {
        error_flag = false;
        currentErrorState = clear;
        error_type = 0;
      } else {
        currentErrorState = failedToInitialize;
      }
      break;
    case FOD:
      if (error_type != error_clear) {
        error_flag = true;
        currentErrorState = FOD;
      } else {
        error_flag = false;
        currentErrorState = clear;
        error_type = 0;
      }
      break;
  }
}

void runConveyorStateMachine() {
  // Serial.println("TOF");
  // Serial.println(new_ToF1);
  // Serial.println("force1");
  // Serial.println(new_For1);
  // Serial.println("force");start
  
  // Serial.println(average);
  switch (currentConveyorState) {
    case start:
      if (operation_flag == START) {
        currentConveyorState = sorting;
        ClearMovingAverage();
      } else {
        currentConveyorState = start;
        if (wait != 1) {
          Serial.println("Awaiting START command.");
          wait = 1;
        }
      }
      break;
    case sorting:
      // check if user suddenly stopped operations due to issues such as errors
      if (operation_flag == STOP) {
        if (error_flag == true) {
          currentConveyorState = error;
          motorStop();
          setColor(0, 255, 255);
        } else {
          currentConveyorState = start;
          wait = 0;
          motorStop();
          setColor(0, 0, 0);
        }
        break;
      }
      new_ToF1 = readDistance(sensor1);
      new_For1 = readForceSensor(sensorPin1);
      // new_For1 = convertToForce(readForceSensor(sensorPin1));
      if ((prev_ToF1 != new_ToF1) | (prev_For1 != new_For1)) {
        prev_ToF1 = new_ToF1;
        prev_For1 = new_For1;
        readings[readIndex] = prev_For1;  // Read from the sensor
        for (int i = 0; i < numReadings; i++) {
          total += readings[i];
        }                                           // Add the newest reading to the total
        readIndex = (readIndex + 1) % numReadings;  // Advance to the next position in the array

        // Calculate the average:
        average = total / numReadings;
        total = 0;
        delay(500);
        // check if both ToF and force sensor are detecting an empty bin
        if (new_ToF1 > ToF_THRESHOLD && average < F_THRESHOLD) {
          currentConveyorState = sorting;

          // set motor speed for conveyor belt
          motorForward(255);  // half speed is 127, full speed is 255

          // set LED to green for active sorting
          setColor(255, 0, 255);  // since the LED is a common anode, 255 is off and 0 is on. This is inverse if LED is common cathode
        }
        // check if either ToF or force sensor detect an empty bin
        else if (new_ToF1< ToF_THRESHOLD | average > F_THRESHOLD) {
          currentConveyorState = bin_full;

          // set motor speed for conveyor belt
          motorStop();

          // set LED to orange to notify user bin is full
          setColor(0, 90, 255);  // orange is typically red = 255, green = 165, blue = 0 with common cathode.
        }
      }
      break;
    case bin_full:
      // check if user suddenly stopped operations due to issues such as errors
      if (operation_flag == STOP) {
        if (error_flag == true) {
          currentConveyorState = error;
          motorStop();
          setColor(0, 255, 255);  // set color back to red to show error
        } else {
          currentConveyorState = start;
          wait = 0;
          motorStop();
          setColor(0, 0, 0);  // set color back to white to show clear
        }
        break;
      }
      new_ToF1 = readDistance(sensor1);
      new_For1 = readForceSensor(sensorPin1);
      // new_For1 = convertToForce(readForceSensor(sensorPin1));
      if ((prev_ToF1 != new_ToF1) | (prev_For1 != new_For1)) {
        prev_ToF1 = new_ToF1;
        prev_For1 = new_For1;
        readings[readIndex] = prev_For1;  // Read from the sensor
        for (int i = 0; i < numReadings; i++) {
          total += readings[i];
        }                                           // Add the newest reading to the total
        readIndex = (readIndex + 1) % numReadings;  // Advance to the next position in the array

        // Calculate the average:
        average = total / numReadings;
        total = 0;
        delay(500);
        // check if both ToF and force sensor are detecting an empty bin
        if (new_ToF1< ToF_THRESHOLD | average > F_THRESHOLD) {
          currentConveyorState = bin_full;
        }
        // check if either ToF or force sensor detect an empty bin
        else if (new_ToF1 > ToF_THRESHOLD && average < F_THRESHOLD) {
          operation_flag = STOP;
          currentConveyorState = start;  // sets back to start rather than sorting so user has to implement start command and can assure user safely away from belt
          wait = 0;
          ClearMovingAverage();
          // set LED to white to notify user of default state
          setColor(0, 0, 0);  // orange is typically red = 255, green = 165, blue = 0 with common cathode.
        }
      }
      break;
    case error:
      // check error state machine and return error message to user
      runErrorStateMachine();
      if (error_flag == true) {
        currentConveyorState = error;
      } else if (error_flag == false) {
        currentConveyorState = start;
        wait = 0;
        setColor(0, 0, 0);  // set color back to white to show clear
      }
      break;
  }
}

#ifdef MAIN_LOOP
void setup() {
  Main_init();
  ClearMovingAverage();
  total = 0;
}

void loop() {
  checkOperationStatus();
  runConveyorStateMachine();
}
#endif MAIN_LOOP