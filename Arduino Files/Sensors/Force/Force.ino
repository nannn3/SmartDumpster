// Define the Arduino analog pin connected to the sensor output
const int sensorPin1 = A0;
const int sensorPin2 = A1;
const int sensorPin3 = A2;

// Define the supply voltage and fixed resistor value
const float Vr = 5.0;    // Assuming a 5V supply voltage
const float R1 = 10000;  // 47k ohm fixed resistor

void setup() {
  // Start serial communication
  Serial.begin(115200);
}

void loop() {
  // Read the sensor voltage
  float V2_1 = analogRead(sensorPin1);  // reads 0 - 1023
  float V2_2 = analogRead(sensorPin2);
  float V2_3 = analogRead(sensorPin3);

  // Convert Rs to force in Newtons
  // Placeholder for the conversion formula - needs calibration or datasheet info
  float force1 = convertResistanceToForce(V2_1);  // This function needs to be defined based on sensor calibration
  float force2 = convertResistanceToForce(V2_2);
  float force3 = convertResistanceToForce(V2_3);

  // Output the force reading
  // Serial.print(V2_1);
  // Serial.print(", ");
  // Serial.print(V2_2);
  // Serial.print(", ");
  // Serial.println(V2_3);

  Serial.print(force1);
  Serial.print(", ");
  Serial.print(force2);
  Serial.print(", ");
  Serial.println(force3);

  // Delay for a bit to avoid spamming
  delay(1000);
}

// Example conversion function - needs actual implementation
float convertResistanceToForce(float V2) {
  float force = 0.028 * V2;
  return force;
}
