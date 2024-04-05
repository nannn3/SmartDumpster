#include "Config.h"

// red LED color when error occurs
// orange LED color when contaminant bin is full
// yellow LED color when arm is actively sorting
// green LED color when conveyor belt is actively moving

#define MAIN_LOOP
#define START 1           // used to tell the program to start the operations of the conveyor belt
#define STOP 0            // used to tell the program to stop the operations of the conveyor belt
#define ToF_THRESHOLD 10  // change this value to show desired minimum desitance of ToF reading in mm
#define F_THRESHOLD 10 // change this value to show max weight allowable for user to lift

int operation_flag = STOP;     // initial state of the operation should be stop
int currentConveyorState = 0;  // initialize conveyor belt state machine to start

typedef enum {
  start,
  sorting,
  bin_full,
  error
} encoder_states;

int checkOperationStatus(void) {
  if (Serial.available() > 0) {
    // read the incoming byte
    String command = Serial.readStringUntil('\n');  // read the command until the newline
    command.trim();                                 //remove the whitespace and newline

    // set the flag high or low
    if (command == "start" | command == "Start" | command == "START") {
      operation_flag = START;
      // echo the byte back to the serial monitor
      Serial.print("Command received: ");
      Serial.println(command);
    } else if (command == "stop" | command == "Stop" | command == "STOP") {
      operation_flag = STOP;
      // echo the byte back to the serial monitor
      Serial.print("Command received: ");
      Serial.println(command);
    } else {
      // print message for error input
      Serial.println("Error: Invalid Response!");
    }
  }
}

void runStateMachine() {
  switch (currentConveyorState) {
    case start:
      if (operation_flag == START) {
        currentConveyorState = sorting;
      } else {
        currentConveyorState = start;
        Serial.println("Awaiting START command.");
      }
      break;
    case sorting:
      if (readDistance() > ToF_THRESHOLD | convertResistanceToForce(readForceSensor(sensorPin1)) < F_THRESHOLD) {
        currentConveyorState = sorting;

        // set motor speed for conveyor belt
        motorForward(127);  // half speed is 127, full speed is 255

        // set LED to green for active sorting
        setColor(255, 0, 255); // since the LED is a common anode, 255 is off and 0 is on. This is inverse if LED is common cathode

      } else if (readDistance() < ToF_THRESHOLD) {
        currentConveyorState = bin_full;

        // set motor speed for conveyor belt
        motorStop();

        // set LED to orange to notify user bin is full
        setColor(0, 90, 255); // orange is typically red = 255, green = 165, blue = 0 with common cathode.
      }
      break;
    case bin_full:
      if (readDistance() < ToF_THRESHOLD){
        currentConveyorState = bin_full;
      } else if (readDistance() > ToF_THRESHOLD && convertResistanceToForce(readForceSensor(sensorPin1)) < F_THRESHOLD)
      break;
    case error:
      break;
  }
}

#ifdef MAIN_LOOP
void setup() {
  Serial.begin(115200);
  Encoder_init();
  LED_init();
  Motor_init();
  ToF_init();
}

void loop() {
  checkOperationStatus();
}
#endif MAIN_LOOP