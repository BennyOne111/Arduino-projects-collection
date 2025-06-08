// Ultrasonic Sensor
// Author: Yiyang Jiang

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1  // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 myOLED(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Set signal pin
int const signal = 2;

void setup() {
  // put your setup code here, to run once:
  // Serial.begin(9600);
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!myOLED.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  // Set the color of text on OLED
  myOLED.setTextColor(WHITE);
  myOLED.clearDisplay();

  // The signal is low and OUTPUT mode at the beginning
  pinMode(signal, OUTPUT);
  digitalWrite(signal, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  myOLED.clearDisplay();

  // Set the signal pin to OUTPUT mode and use it to send a 10μs high trigger pulse to the sensor
  pinMode(signal, OUTPUT);
  digitalWrite(signal, HIGH);
  delayMicroseconds(10);
  digitalWrite(signal, LOW);
  
  // Set the signal pin to INPUT mode
  pinMode(signal, INPUT);
  // Use a while loop to wait until the echo signal from the sensor on the signal pin goes HIGH
  while (digitalRead(signal) == LOW) {
  }
  // use the micros() function to record the beginning time of the pulse
  unsigned long startTime = micros();
  bool flag = 0;

  // Use a while loop to wait until the signal goes back to LOW
  while (digitalRead(signal) == HIGH) {
    // To check to see if more than 24000 microseconds have elapsed and break out of the while loop if this is the case
    if (micros()-startTime > 24000){
      flag = 1;
      break;
    }
  }

  // Use the micros() function to get the pulse ending time and then calculate the duration of the pulse
  unsigned long endTime = micros();
  unsigned long duration = endTime - startTime;
  //Serial.println(duration);

  // Calculate the distance in cm,and then display it on the OLED display
  float distance = 172.35/10000*duration;
  myOLED.setCursor(0, 0);
  if(flag == 0){
    myOLED.println(distance);
  }
  else{
    myOLED.println("Out of Range!");
  }
  myOLED.display();
  //Serial.println(distance);
  delay(1000);
}
