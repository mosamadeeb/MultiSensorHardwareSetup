#include <Wire.h>

#define HMC5883L_ADDRESS 0x0D  // Correct I2C address for your module
int16_t magnetometer[3]; // Array to store magnetometer readings

void setup() {
  Serial.begin(115200);  // Initialize serial communication for printing data
  Wire.begin();        // Initialize I2C communication

  // Since the initialization settings might differ for non-HMC5883L modules,
  // make sure to adjust these settings according to your module's datasheet.
  Wire.beginTransmission(HMC5883L_ADDRESS);
  Wire.write(0x09);  // Example: Set to Continuous Measurement Mode
  // This specific configuration may not apply to your module.
  // Refer to your module's datasheet for correct configuration commands.
  Wire.write(0x1D);  // Example: Set the output data rate and measurement configuration
  Wire.endTransmission();
}

void loop() {
  // Read magnetometer data
  Wire.beginTransmission(HMC5883L_ADDRESS);
  Wire.write(0x00);  // Set register pointer to Data Output X MSB Register
  Wire.endTransmission();
  Wire.requestFrom(HMC5883L_ADDRESS, 6);  // Request 6 bytes; 2 for each axis

  if(Wire.available() == 6) {
    // Combine high and low bytes for each axis
    magnetometer[0] = (Wire.read() << 8) | Wire.read(); // X-axis
    magnetometer[1] = (Wire.read() << 8) | Wire.read(); // Y-axis
    magnetometer[2] = (Wire.read() << 8) | Wire.read(); // Z-axis
  }

  // Print magnetometer readings to serial monitor
  //Serial.print("Magnetometer (X, Y, Z): ");
  Serial.print(magnetometer[0]);
  Serial.print(", ");
  Serial.print(magnetometer[1]);
  Serial.print(", ");
  Serial.print(magnetometer[2]);
  //Serial.println(" uT");
  Serial.println(", ");

  delay(1000); // Delay between readings (adjust as needed)
}
