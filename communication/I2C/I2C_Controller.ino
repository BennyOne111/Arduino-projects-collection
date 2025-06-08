// I2C controller
// Yiyang Jiang

# include <Wire.h>
const int LED = 7;
// A potentiometer wired to analog input pin A0
const int pot = A0;

void setup() {
  Serial.begin(9600);
  // Begin I2C communication on the transmitter
  Wire.begin();
  pinMode(LED, OUTPUT);
  pinMode(pot, INPUT);
}

void loop() {
  bool myBooleanVariable;
  int potvol = analogRead(pot);
  
  float volToPer = potvol * 256.0/1024.0; // Read the voltage from the potentiometer and scale it from 0 to 1023 down to 0 to 255
  Serial.println(volToPer); // Display the value (from 0 to 255) on the Serial Monitor

  
  Wire.beginTransmission(0x09); // Start sending data to the peripheral
  Wire.write((byte)volToPer); // Send a byte
  Wire.endTransmission(0x09); // Finish sending data to the peripheral

  
  Wire.requestFrom(0x09, 1); // Request 1 byte from the peripheral
  while (Wire.available()){ // Read the data
    myBooleanVariable = Wire.read();
  }

  // Light the LED or not depending on if the data is true (1), or false (0)
  if (myBooleanVariable == 1){
    digitalWrite(LED, HIGH);
  }
  else{
    digitalWrite(LED, LOW);
  }
}
