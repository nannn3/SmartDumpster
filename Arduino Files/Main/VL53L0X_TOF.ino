#include <Wire.h>
#include <VL53L0X.h>

#define TOF_LOOP

int ToF_init(void) {
  // Initialize VL53L0X sensor
  Wire.begin();
  pinMode(xshut1, OUTPUT);
  pinMode(xshut2, OUTPUT);

  // Deactivate sensor 2
  digitalWrite(xshut2, LOW);
  delay(10);

  // Initialize sensor 1 with default address
  digitalWrite(xshut1, HIGH);
  delay(10);
  sensor1.init();
  sensor1.setAddress(0x2A);  // Set a new address for sensor 1

  // Activate sensor 2 and initialize with a different address
  digitalWrite(xshut2, HIGH);
  delay(10);
  sensor2.init();
  sensor2.setAddress(0x2B);  // Set a new address for sensor 2

  // sensor1.init();
  // // Initialize sensor 1 (default address is already 0x29)
  // pinMode(xshut1, OUTPUT);
  // digitalWrite(xshut1, LOW);
  // delay(10);
  // digitalWrite(xshut1, HIGH);

  // sensor2.init();
  //   // Initialize sensor 2 and set a new address
  // pinMode(xshut2, OUTPUT);
  // digitalWrite(xshut2, LOW);
  // delay(10);
  // digitalWrite(xshut2, HIGH);
  // sensor2.setAddress(0x30);  // Set the new address for sensor 2
  return true;
}

// SDA is pin A4
// SCL is pin A5
uint16_t readDistance(VL53L0X &sensor) {

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
  ToF_init();
}

void loop() {
  // Read distance from VL53L0X sensor1
  uint16_t distance1 = readDistance(sensor1);

  Serial.print("Distance of sensor 1: ");
  Serial.print(distance1);
  Serial.println(" mm");

  // Read distance from VL53L0X sensor2
  uint16_t distance2 = readDistance(sensor2);

  Serial.print("Distance of sensor 2: ");
  Serial.print(distance2);
  Serial.println(" mm");
  delay(100);
}
#endif TOF_LOOP