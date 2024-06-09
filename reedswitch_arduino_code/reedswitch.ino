// Define the pins
const int reedSwitchPin1 = 2; // Assuming the DO pin of the reed switch is connected to digital pin 2
const int reedSwitchPin2 = 5;
void setup() {
  // Initialize Serial communication
  Serial.begin(9600);

  // Set the reed switch pin as input
  pinMode(reedSwitchPin1, INPUT);
  pinMode(reedSwitchPin2, INPUT);

}

void loop() {
  // Read the state of the reed switch
  int reedSwitchState1 = digitalRead(reedSwitchPin1);
  int reedSwitchState2 = digitalRead(reedSwitchPin2);
  // Print the state to the Serial Monitor
  char sensor1[10];
  char sensor2[10];
  if (reedSwitchState1 == HIGH) {
    //Serial.println("1:");
    strcpy(sensor1,"1:0");
  } else {
    //Serial.println("Reed Switch 1 state: HIGH (Magnetic field detected)");
    strcpy(sensor1,"1:1");
  }
  if (reedSwitchState2 == HIGH) {
    //Serial.println("Reed Switch 2 state: LOW (No magnetic field detected)");
    strcpy(sensor2,"2:0");
  } else {
    //Serial.println("Reed Switch 2 state: HIGH (Magnetic field detected)");
    strcpy(sensor2,"2:1");
  }
  char ourOut[15];
  snprintf(ourOut, sizeof(ourOut), "%s,%s", sensor1, sensor2);
  Serial.println(ourOut);
  // Delay for a short period to avoid flooding the Serial Monitor
  delay(500);
}