// This code is for the joystick

//Include the necessary libraries
#include <RH_RF69.h>  //radiohead library for RFM69HCW transceiver
#include <SPI.h>      //SPI library for comunication
#include <Wire.h>
// #include <Adafruit_SSD1306.h>
// #include <Adafruit_GFX.h>

//REMOTE CONTROLLER ARDUINO
//Lets first configure the pins for the remote control/transmitter arduino part

//Define pins for joystick
const int joystick_x = A2;       //analog input pin on Arduino UNO for joystick control for steering
const int joystick_y = A3;       //analog input pin on Arduino UNO for joystick control for acceleration

//Define pins for RFM69HCW transceiver
const int CS = 5;   //Chip select pin
const int G0 = 2;   //Interrupt request pin
const int RST = 6;  //Reset pin

// //OLED Display
// #define SCREEN_WIDTH 128 //OLED display width in pixels
// #define SCREEN_HEIGHT 64 //OLED display height in pixels
// #define OLED_RESET -1 //OLED rest pin

// //create an object for oled display
// Adafruit_SSD1306 oledDisplay(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//Create the instance of the transceiver object
RH_RF69 RF(CS, G0);

//Define joystick center and threshold values
const int joystick_center = 0;  //this is the central value of joystick analog output
//(helps to determine how much the joystick is moved away from the center, goes below 0
//indicates -x or -y,
//goes above, indicates x or y)
const int joystick_threshold = 20;  //threshold value to prevent minor fluctuations around joystick
//center value, meaning the center value could not always be 0 so to prevent we have threshold value
//Lets create a data packet structure to send the data from remote controller with RF transceiver
//to the receiver arduino

struct RFData {
  byte x;         //joystick x value
  byte y;         //joystick y value
  // bool button; //joystick button value
} RFDataToSend;  //instance(containes the data to be sent) of the struct, the values in the struct can be called by this instance

//Define the states for the finite state machine
enum State {
  IDLE,     //The resting state, where the program waits for joystick movement 
  CONTROL,  //state for controlling joystick_x and joystick_y values
};

State currentState = IDLE;  //Start in the IDLE state

void setup() {
  Serial.begin(9600);  //Begin serial communication for debugging

  //Initialize RF transceiver
  pinMode(RST, OUTPUT);     //Set up the reset pin for the transceiver
  digitalWrite(RST, LOW);   //Starting with reset pin LOW
  delay(10);                //delay for 10 milliseconds
  digitalWrite(RST, HIGH);  //Switch to reset pin HIGH
  delay(10);                //delay for 10 milliseconds
  digitalWrite(RST, LOW);   //Switch to reset pin LOW
  delay(10);                //delay for 10 milliseconds

  Serial.println("Initializing transceiver");  //Initialize the transceiver

  //Halt if initialization fails
  if (!RF.init()) {
    Serial.println("Transceiver initializing failed");
    while (1)
      ;  //stay here if initializing fails
  }

  Serial.println("Transciver successfully initialized");

  //Set the frequency of the transceiver to 905 MHz
  Serial.println("Setting frequency");
  RF.setFrequency(905.0);
  if (!RF.setFrequency(905.0)) {
    Serial.println("Frequency set failed");
    while (1)
      ;  //Halt if frequency setting fails
  }

  Serial.println("Frequency set to 905 MHz");

  //Set the transmission power of the transceiver
  RF.setTxPower(20, true);  //Set maximum power with HCW mode enabled
  Serial.print("Joystick value x");
  Serial.println(joystick_x);
  Serial.print("Joystick value y");
  Serial.println(joystick_y);
}

void loop() {

  switch (currentState) {
    case IDLE:
      {
        RFDataToSend.x = map(analogRead(joystick_x), 0, 1023, 0, 255);
        RFDataToSend.y = map(analogRead(joystick_y), 0, 1023, 0, 255);
     
        //Check if the joystick has moved outside the threshold
        currentState = CONTROL;
        break;
      }

    case CONTROL:
      {
        transmitXY(RFDataToSend.x, RFDataToSend.y);
        delay(10);           //delay for debounce
        currentState = IDLE;  //return back to IDLE state   
        break;
      }
  }
}

void transmitXY(byte RFDataToSend_X, byte RFDataToSend_Y) {
  byte radiopacket[sizeof(RFDataToSend_X)+sizeof(RFDataToSend_Y)]; //create sizeof RFDataToSend array called radiopacket
  radiopacket[0] = RFDataToSend_X;
  radiopacket[1] = RFDataToSend_Y;
  Serial.print(radiopacket[0]);
  Serial.print(" ");
  Serial.println(radiopacket[1]);
  RF.send(radiopacket, sizeof(RFDataToSend_X)+sizeof(RFDataToSend_Y)); //send data
  RF.waitPacketSent();
  delay(500);
}
