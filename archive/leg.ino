#define FLEX_SENSOR_PIN A0  // The analog pin connected to the flex sensor

void setup() {
  // Initialize Serial communication
  Serial.begin(9600);
}

void loop() {
  // Read the analog value from the flex sensor
  int flexValue = analogRead(FLEX_SENSOR_PIN);

  // Convert the analog value to voltage
  float voltage = (flexValue / 1023.0) * 5.0; // Assuming a 5V reference

  // Print the voltage value to the Serial Monitor
  Serial.print("Flex Sensor Voltage: ");
  Serial.println(voltage);

  delay(100); // Delay to avoid flooding the Serial Monitor, adjust as needed
}
