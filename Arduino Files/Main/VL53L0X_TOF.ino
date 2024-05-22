#include <Wire.h>
#include <VL53L0X.h>

// #define TOF_LOOP

// initializes pins, instances, and sets the address for the sensors. Initializes the I2C
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
  return true;
}

// SDA is pin A4
// SCL is pin A5
// returns distance in cm
uint16_t readDistance(VL53L0X &sensor) {

  // Start a single measurement
  // Read distance measurement in millimeters
  sensor.startContinuous(20);
  uint16_t reading = sensor.readRangeContinuousMillimeters();
  uint16_t distance = (reading) / 10;  //Calibration and conversion to cm, soldered ToF does not need calibration, but breadboarded ToF does. Calibrated function commented below.
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
  Serial.println(" cm");

  // Read distance from VL53L0X sensor2
  uint16_t distance2 = readDistance(sensor2);

  // Serial.print("Distance of sensor 2: ");
  // Serial.print(distance2);
  // Serial.println(" cm");
  delay(100);
}
#endif TOF_LOOP