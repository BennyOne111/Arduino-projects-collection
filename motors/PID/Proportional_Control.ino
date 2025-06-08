// proportional control
// Author: Yiyang Jiang

const int encoderA = 2;
const int encoderB = 3;

const int input1 = 8;
const int input2 = 9;
const int enable = 10;
const int pot = A0;

volatile int counter = 0;

const int Kprop = 1;

const int deltaT = 10;
unsigned long previousTime;

void setup() {
  Serial.begin(9600);
  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
  pinMode(input1, OUTPUT);
  pinMode(input2, OUTPUT);
  pinMode(enable, OUTPUT);
  pinMode(pot, INPUT);

  attachInterrupt(0, changeA, CHANGE);
  attachInterrupt(1, changeB, CHANGE);

  previousTime = 0;
}

void loop() {
  // Make the desired counter value so that a full turn of the potentiometer corresponds to the output shaft going a full revolution
  int desiredCounter = analogRead(pot)*960/1024;
  // Convert the error from counts to radians
  float error = (desiredCounter - counter)*2*3.14/960;
  // Calculate the control signal by using proportional control
  float controlSignal = Kprop*error;
  // If the control signal is positive, turn Input1 to HIGH and Input2 to LOW, the motor rotates clockwise
  if (controlSignal > 0){
    digitalWrite(input1, HIGH);
    digitalWrite(input2, LOW);
  }
  // If negative, do the opposite
  else{
    digitalWrite(input1, LOW);
    digitalWrite(input2, HIGH);
  }
  // Calculate PWM
  float pwm = constrain(abs(controlSignal*256.0/6.28), 0, 255) ;
  analogWrite(enable, pwm);
  
  while (millis() - previousTime < deltaT) { }
  previousTime = millis();

}

void changeA() {
  if (digitalRead(encoderA) != digitalRead(encoderB)){
    counter++;
  }
  else{
    counter--;
  }
}

void changeB() {
  if (digitalRead(encoderA) == digitalRead(encoderB)){
    counter++;
  }
  else{
    counter--;
  }
}