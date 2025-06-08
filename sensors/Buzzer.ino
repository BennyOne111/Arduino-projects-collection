// Buzzer
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


int const signal = 2;
int const buzzer = 8;

void setup() {
  // put your setup code here, to run once:
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!myOLED.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  myOLED.setTextColor(WHITE);
  myOLED.clearDisplay();

  pinMode(signal, OUTPUT);
  digitalWrite(signal, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  myOLED.clearDisplay();
  // keep buzzer silent 
  noTone(buzzer);
  
  pinMode(signal, OUTPUT);
  digitalWrite(signal, HIGH);
  delayMicroseconds(10);
  digitalWrite(signal, LOW);
  pinMode(signal, INPUT);
  while (digitalRead(signal) == LOW) {
  }
  unsigned long startTime = micros();
  bool flag = 0;
  while (digitalRead(signal) == HIGH) {
    if (micros()-startTime > 24000){
      flag = 1;
      break;
    }
  }
  unsigned long endTime = micros();
  unsigned long duration = endTime - startTime;
  //Serial.println(duration);
  float distance = 172.35/10000*duration;

  // buzzer on
  tone(buzzer, distance);

  myOLED.setCursor(0, 0);
  if(flag == 0){
    myOLED.println(distance);
  }
  else{
    myOLED.println("Out of Range!");
  }
  myOLED.display();
  delay(1000);
}

