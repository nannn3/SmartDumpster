#include <Wire.h>
#include <VL53L0X.h>

#define VL53L0X_ADDRESS 0x29
// #define TOF_LOOP

int ToF_init(void) {
  // Initialize VL53L0X sensor
  Wire.begin();
  sensor.init();
  return true;
}

// SDA is pin A4
// SCL is pin A5
uint16_t readDistance() {

  // Start a single measurement
  // Read distance measurement in millimeters
  sensor.startContinuous(20);
  uint16_t distance = sensor.readRangeContinuousMillimeters() - 50;
  return distance;
}

#ifdef TOF_LOOP
void setup() {
  Serial.begin(115200);

  // Initialize VL53L0X sensor
  Wire.begin();
  sensor.init();
}

void loop() {
  // Read distance from VL53L0X sensor
  uint16_t distance = readDistance();

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" mm");
}
#endif TOF_LOOP