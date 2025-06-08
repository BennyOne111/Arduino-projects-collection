// Control the Wheels with PID
// Author: Yiyang Jiang


const int A_pin = 2;
const int B_pin = 3;

const int Enable1 = 10;
const int Input1 = 8;
const int Input2 = 9;
const int kp = 1;
const int ki = 1;
const int kd = 1;
const int deltaT = 10;
unsigned long previousTime = 0;

int desired_counter;
float error = 0;
float prevError = 0;
float errorIntegrated = 0;
float control_sig;
float pwm_val;
unsigned long startTime;
int outputDuration = 4000;
int n = 0;
volatile int actualCounter;

float PID(float error, float prevError, float errorIntegrated, int kp, int ki, int kd, unsigned long loopTime){
  float controlSignal;
  controlSignal = kp*error + ki*(errorIntegrated+error*loopTime) + kd*(error-prevError)/loopTime;
  return controlSignal;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(A_pin, INPUT);
  pinMode(B_pin, INPUT);

  pinMode(Enable1, OUTPUT);
  pinMode(Input1, OUTPUT);
  pinMode(Input2, OUTPUT);

  attachInterrupt(0, A_change, CHANGE);
  // attachInterrupt(1, B_change, CHANGE);

  startTime = millis();
  actualCounter = 0;

  while (millis() - startTime < outputDuration ) {

    desired_counter = 960;
    error = (desired_counter - actualCounter) * 2 * 3.14/960;

    control_sig = PID(error, prevError, errorIntegrated, kp, ki, kd, deltaT);
    prevError = error;
    errorIntegrated = errorIntegrated + error*deltaT;

    if (control_sig < 0) {
      digitalWrite(Input1, HIGH);
      digitalWrite(Input2, LOW);
    }
    else {
      digitalWrite(Input1, LOW);
      digitalWrite(Input2, HIGH);
    }

    pwm_val = abs(control_sig) * 255.0/6.28;
    pwm_val = constrain(pwm_val, 0, 255);

    if (n%2 == 0) {
      Serial.println(actualCounter/960.0 * 100.0);
    }
    n++;
 
    analogWrite(Enable1, pwm_val);

    while ( millis() - previousTime < deltaT ) { }
    previousTime = millis();
  }
}


void loop() {
}

// void A_change() {
//   if (digitalRead(A_pin) != digitalRead(B_pin)) {
//     actualCounter++;
//   }
//   else {
//     actualCounter--;
//   }
// }

// void B_change() {
//   if (digitalRead(A_pin) != digitalRead(B_pin)) {
//     actualCounter--;
//   }
//   else {
//     actualCounter++;
//   }
// }

