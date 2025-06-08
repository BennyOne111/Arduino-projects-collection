// I2C peripheral
// Author: Yiyang Jiang

# include <Wire.h>
// Analog input pin A0 wired to the 3.3V output
const int pin = A0;
byte x;
byte y;
bool myBooleanVariable2;

void setup() {
  // Begin I2C communication on the receiver
  Wire.begin(0x09);
  Serial.begin(9600);
}

void receiveEvent(){
  // Read a byte of data
  x = Wire.read();
  Serial.println(x);
}
void requestEvent(){
  // Send the boolean variable as a byte of data
  Wire.write(myBooleanVariable2);
}

void loop() {
  // Point to a function that will be called when data is received
  Wire.onReceive(receiveEvent);
  // Read the voltage from the 3.3V output
  // and scale it from 0 to 1023 down to 0 to 255
  float y = analogRead(pin) * 256.0/1024.0;
  // The boolean variable is true if the received data is greater than the measured 3.3V
  if (x > y){
    myBooleanVariable2 = 1;
  }
  else{
    myBooleanVariable2 = 0;
  }
  // Point to a function that will be called when data is requested
  Wire.onRequest(requestEvent);
}
