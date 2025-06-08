// Receive a byte from another Arduino
// Author: Yiyang Jiang

#include <RH_RF69.h>
#include <SPI.h>

const int RSTPin = 8;
const int CS_pin = 7;
const int INT_pin = 3;

// Create an instance of the transceiver object
RH_RF69 myTransceiver(CS_pin, INT_pin);

void setup() {

  // Initialize the transceiver
  Serial.begin(9600);
  // Set up the pin that is connected to the transceiver’s RST pin to be an output
  pinMode(RSTPin, OUTPUT);
  // Set the voltage to be low
  digitalWrite(RSTPin, LOW);
  // Manually reset the transceiver by setting the RST pin to low for 100ms, 
  // high for 10ms, then back to low for 10ms
  delay(100);
  digitalWrite(RSTPin, HIGH);
  delay(10);
  digitalWrite(RSTPin, LOW);
  delay(10);
  // Initialize the transceiver and desplay the initialization information
  if (myTransceiver.init()){
    Serial.println("Initialize successfully!");
  }
  else{
    Serial.println("Initialize unsuccessfully!");
  }
  // Set the transceiver’s frequency and desplay the initialization information
  if (myTransceiver.setFrequency(915)){
    Serial.println("Set frequency successfully!");
  }
  else{
    Serial.println("Set frequency unsuccessfully!");
  }
  // Set the transceiver’s power level
  myTransceiver.setTxPower(15, true);
}

void loop() {
  // Create a byte to receive data
  byte RXbyte;
  // See if data has been received
  if (myTransceiver.available()){
    //A buffer array to hold the data
    byte buffer[10];
    byte length = sizeof(buffer);
    // Get the data
    if (myTransceiver.recv(buffer, &length)){
      RXbyte = buffer[0];
      // Print the first element of the buffer array on the Serial Monitor
      Serial.print("Received ");
      Serial.println(RXbyte);
    }
  }
}
