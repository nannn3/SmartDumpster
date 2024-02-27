#include <Wire.h>
#include <VL53L0X.h>

#define VL53L0X_ADDRESS 0x29

VL53L0X sensor;

// SDA is pin A4
// SCL is pin A5
uint16_t readDistance() {
  
    // Start a single measurement
    // Read distance measurement in millimeters
    uint16_t distance = sensor.readRangeContinuousMillimeters();
    sensor.setMeasurementTimingBudget(33000); // Set measurement timing budget to 33ms
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 14); // performs initial measurements to estimate the distance
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 10); // where the sensor performs the actual distance measurement
    sensor.startContinuous();

    // Wait for measurement to complete
        // Check if measurement is valid
    if (sensor.timeoutOccurred()) {
        Serial.println("Error: Timeout occurred while reading distance!");
    } else {
        Serial.print("Distance: ");
        Serial.print(distance);
        Serial.println(" mm");
    }
    return distance;
}

void setup() {
    Serial.begin(115200);

    // Initialize VL53L0X sensor
    Wire.begin();
    sensor.init();
    sensor.setTimeout(500);  // Set timeout to 500ms
    sensor.startContinuous();  // Start continuous measurement

    // Perform single reference calibration
    sensor.setSignalRateLimit(0.1);  // Set signal rate limit to reduce ambient noise
    sensor.writeReg(VL53L0X::SYSRANGE_START, 0x01);  // Start measurement
    sensor.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);  // Clear interrupt
    Serial.println("Calibration complete!");
}

void loop() {
    // Read distance from VL53L0X sensor
    uint16_t distance = readDistance();

    // Check if measurement is valid
    if (sensor.timeoutOccurred()) {
        Serial.println("Error: Timeout occurred while reading distance!");
    } else {
        Serial.print("Distance: ");
        Serial.print(distance);
        Serial.println(" mm");
    }
}