#include "Config.h"

// #define FORCE_LOOP

// a function that takes a raw reading value from an analog pin
float readForceSensor(float forcePin){
  float voltage = analogRead(forcePin);
  return voltage;
}

// a function that takes an adc value from the analog pin and converts it to force
float convertToForce(float V2) {
  // float force = (0.0107 * V2) - 0.0594;
  float force = (0.0105 * V2) - 0.959;
  // float force = (0.00868 * V2) - 0.518;
  return force;
}

#ifdef FORCE_LOOP
void setup() {
  // Start serial communication
  Serial.begin(115200);
}

void loop() {
  // Read the sensor voltage
  float V2_1 = analogRead(sensorPin1);  // reads 0 - 1023
  float V2_2 = analogRead(sensorPin2);

  // Convert Rs to force in lbs
  float force1 = convertToForce(V2_1);  // This function needs to be defined based on sensor calibration
  // float force2 = convertToForce(V2_2);

  // Output the force reading
  // Serial.println(V2_1);


  Serial.println(force1);

  // Delay for a bit to avoid spamming
  delay(100);
}
#endif

