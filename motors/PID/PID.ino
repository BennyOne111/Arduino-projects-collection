// PID function
// Author: Yiyang Jiang
const int kp = 1;
const int ki = 1;
const int kd = 1;
const unsigned long loopTime;
float preError;
float error;
float errorIntegrated;

float PID(float error, float prevError, float errorIntegrated, int kp, int ki, int kd, unsigned long loopTime){
  float controlSignal;
  controlSignal = kp*error + ki*(errorIntegrated+error*loopTime) + kd*(error-prevError)/loopTime;
  return controlSignal;
}

void setup() {
  // put your setup code here, to run once:
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
