// SPI finite state machine model
// Author: Yiyang Jiang

#include <RH_RF69.h>
#include <SPI.h>

const int RSTPin = 8;
const int CS_pin = 7;
const int INT_pin = 3;
const int firstButton = 4;
const int secondButton = 6;
RH_RF69 myTransceiver(CS_pin, INT_pin);

// Set states
enum States {initialize, increment, transmit, listening};
// Initialize
States myState1 = initialize;
byte TXbyte;


void setup() {
  // Initialize the transceiver
  Serial.begin(9600);
  pinMode(RSTPin, OUTPUT);
  digitalWrite(RSTPin, LOW);
  delay(100);
  digitalWrite(RSTPin, HIGH);
  delay(10);
  digitalWrite(RSTPin, LOW);
  delay(10);
  if (myTransceiver.init()){
    Serial.println("Initialize successfully!");
  }
  else{
    Serial.println("Initialize unsuccessfully!");
  }
  if (myTransceiver.setFrequency(915)){
    Serial.println("Set frequency successfully!");
  }
  else{
    Serial.println("Set frequency unsuccessfully!");
  }
  myTransceiver.setTxPower(15, true);
  
  // Initialize the buttons
  pinMode(firstButton, INPUT_PULLUP);
  pinMode(secondButton, INPUT_PULLUP);
  
}

void loop() {
  switch (myState1){
    case initialize:{
      // Initialize the TXbyte
      TXbyte = 1;
      // Go to "listening"
      myState1 = listening;
      break;
    }

    case listening: {
      byte RXbyte;
      byte buffer[10];
      byte length = sizeof(buffer);
      // Receive the prime number from another Arduino
      if (myTransceiver.available()){
        if (myTransceiver.recv(buffer, &length)){
          RXbyte = buffer[0];
          Serial.print("Received ");
          Serial.println(RXbyte);
        }
      }
      // If the first button is pressed, the state changes to "increment"
      if (digitalRead(firstButton) == LOW){
        myState1 = increment;
      }
      // If the second button is pressed, the state changes to "transmit"
      if (digitalRead(secondButton) == LOW){
        myState1 = transmit;
      }
      break;
    }

    case increment:{
      // If a button is pressed, increment this number by one
      TXbyte = TXbyte + 1;
      // If it gets to 43, reset it to 1
      if (TXbyte == 43){
        TXbyte = 1;
      }
      delay(500);
      // Go back to "listening"
      myState1 = listening;
      break;
    }

    case transmit: {
      byte radiopacket[1] = { TXbyte };
      Serial.print("Sending  ");
      Serial.println(TXbyte);
      // Transmit the new number
      myTransceiver.send(radiopacket, sizeof(radiopacket));
      myTransceiver.waitPacketSent();
      delay(500);
      // Go back to "listening"
      myState1 = listening;
      break;
    }
  }
}
