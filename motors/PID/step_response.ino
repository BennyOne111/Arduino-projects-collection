// proportional control & steo response
// Author: Yiyang Jiang

const int encoderA = 2;
const int encoderB = 3;

const int input1 = 8;
const int input2 = 9;
const int enable = 10;
const int pot = A0;

volatile float counter;

const int Kprop = 1; // V/rad
const int fullTurn = 960;

const int deltaT = 10;
const int outputDuration = 4;
unsigned long previousTime;
unsigned long startTime;

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

  counter = 0;
  previousTime = 0;
  int desiredCounter = fullTurn;
  int printCounter = 0;
  float error, controlSignal, pwm;

  startTime = millis();
  while(millis() - startTime < outputDuration*1e3){
    // Main loop that prints the current count value every second for monitoring
    int potentialmeter_Value = analogRead(pot);
    ///Serial.println(potentialmeter_Value);
    int desired_count = map(potentialmeter_Value,0,1023,0,960);
  
    int error = desired_count - counter;
    
    float errorRadians = (error*2*3.14/960); //convert into radians
    
    float controlSignal = Kprop*errorRadians;
    if(controlSignal > 0){
      digitalWrite(input1,HIGH);
      digitalWrite(input2,LOW);
    }
    else{
      digitalWrite(input1,LOW);
      digitalWrite(input2,HIGH);
    }
    float pwmValue = abs(controlSignal)*(255.0/6.28);
    pwmValue = constrain(pwmValue,0,255);
    // Serial.println(pwmValue);
    analogWrite(enable,pwmValue);

    while ( millis() - previousTime < 10 ) { }
    previousTime = millis();

    printCounter = printCounter % 2;
    if (printCounter == 0){
      Serial.println(counter/fullTurn*100);
    }
    printCounter++;
  }
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

void loop() {
}

