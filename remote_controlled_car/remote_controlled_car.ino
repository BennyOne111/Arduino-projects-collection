// The code is used to move the car

#include <RH_RF69.h> // Radiohead library for RFM69HCW transceiver
#include <SPI.h> // SPI library for comunication
#include <Servo.h> // Servomotor

// Define the car
class Robot {
  public:

    Robot() : RF(CS, G0) {}

    void initialization() {
      // Initialize the transceiver
      pinMode(RST, OUTPUT); //Set up the reset pin for the transceiver
      digitalWrite(RST,LOW); //Starting with reset pin LOW
      delay(10);  //delay for 10 milliseconds
      digitalWrite(RST,HIGH); //Switch to reset pin HIGH
      delay(10);  //delay for 10 milliseconds
      digitalWrite(RST,LOW); //Switch to reset pin LOW
      delay(10); //delay for 10 milliseconds
      Serial.println("Initializing transceiver"); //Initialize the transceiver
      //Halt if initialization fails
      if(!RF.init()){
      Serial.println("Transceiver initializing failed");
      while(1); //stay here if initializing fails
      }
      Serial.println("Transciver successfully initialized");
      //Set the frequency of the transceiver to 905 MHz
      Serial.println("Setting frequency");
      RF.setFrequency(900.0);
      if(!RF.setFrequency(900.0)){
      Serial.println("Frequency set failed");
      while(1); //Halt if frequency setting fails
      }
      Serial.println("Frequency set to 905 MHz");
      //Set the transmission power of the transceiver
      RF.setTxPower(20,true); //Set maximum power with HCW mode enabled

      // Initialize the servo motors and the servo motor
      myServoObject.attach(STEERING_SERVO_CONTROL_PIN);
      pinMode(MOTOR_R_CONTROL_PIN, OUTPUT);
      pinMode(MOTOR_L_CONTROL_PIN, OUTPUT);
      // Configure pins for encoder
      pinMode(MOTOR_R_ENCODER_A_PIN, INPUT);
      pinMode(MOTOR_L_ENCODER_A_PIN, INPUT);
    }

    void encoderL() {
      counterL++;
      return;
    }

    void encoderR() {
      counterR++;
      return;
    }

    int getCounterL() {
      return counterL;
    }

    int getCounterR() {
      return counterR;
    }

    void getReferenceFromJoystick() {
      if(RF.available()){
        //****Receive joy_stick****//
        byte buffer[sizeof(RFReceivedData.x)+sizeof(RFReceivedData.y)];
        byte length = sizeof(buffer);
        //Receive data into the buffer and copy it to the RFReceivedData
        if(RF.recv(buffer, &length)){
          RFReceivedData.x = buffer[0];
          RFReceivedData.y = buffer[1];
        }  
      }
      delay(10); 
    }

    float getWheelSpeed(int currentCounter, int prevCounter, int intervalInMs) {
      float countsPerSecond = (currentCounter - prevCounter) / (intervalInMs / 1000.0); // Counts/sec 
      float rotationsPerSecond = countsPerSecond / COUNTS_PER_ROTATION;
      float wheelSpeed = rotationsPerSecond * 2 * M_PI * WHEEL_RADIUS; // Speed in meters/second
      return wheelSpeed;
    }

    int getDesiredAngle() {
      int desiredAngle;
      if (RFReceivedData.x > 130 || RFReceivedData.x < 125) {
        desiredAngle = map(RFReceivedData.x, 0, 255, 0, 180); // Map input range to degrees
      }
      else {
        desiredAngle = 90; // Map input range to degrees
      }
      return desiredAngle;
    }

    float getDesiredSpeed() {
      float desiredSpeed;
      if (RFReceivedData.y > 128 && RFReceivedData.y < 134) {
        desiredSpeed = 0;
      }
      else {
        desiredSpeed = map(RFReceivedData.y, 0, 255, MIN_SPEED, MAX_SPEED);
      }
      return desiredSpeed;
    }

    void steer(int angle) {
      myServoObject.write(angle);
    }

    void move(int pwmL, int pwmR) {
      analogWrite(MOTOR_R_CONTROL_PIN, pwmR);
      analogWrite(MOTOR_L_CONTROL_PIN, pwmL);
    }

  private:
    //// Define pins
    // Define pins for RFM69HCW transceiver
    const int CS = 49;  // Chip Select
    const int G0 = 2;   // Interrupt
    const int RST = 48; // Reset
    // Define pins for motors
    const int MOTOR_R_CONTROL_PIN = 5;
    const int MOTOR_L_CONTROL_PIN = 6;
    const int MOTOR_R_ENCODER_A_PIN = 19;
    const int MOTOR_L_ENCODER_A_PIN = 18;
    const int STEERING_SERVO_CONTROL_PIN = 4;

    //// Define the mechanical parameters of the car
    const float MAX_SPEED = 3.0, MIN_SPEED = 1.0;
    const float WHEEL_RADIUS = 0.033; // in meters

    //// Define the variables of the sensors and actuators
    // Steering servo
    Servo myServoObject;
    // Transceiver
    RH_RF69 RF;
    struct RFData {
      byte x; //joystick x value
      byte y; //joystick y value
      // bool button; //joystick button value
    } RFReceivedData; // Data packet structure to receive the data from remote controller
    // Motors
    volatile int counterL = 0;
    volatile int counterR = 0;
    const float COUNTS_PER_ROTATION = 185;
};

Robot car;
unsigned long prevTime;
int prevCounterL, prevCounterR;
// Control Interval
const int INTERVAL = 50;
//Define joystick threshold for steering
const int joystick_threshold = 20;
// Define the PID parameters and errors
float kp = 1;
float ki = 0;
float kd = 0.1;
float errorL, errorR;
float integratedErrorL = 0, integratedErrorR = 0;  // Integral terms
float prevErrorL = 0, prevErrorR = 0; // Previous errors for derivative terms

void setup() {
  Serial.begin(9600);
  car.initialization();
  // Attach interrupts for handling encoder input changes on pins 4 and 5
  attachInterrupt(4, ISREncoderL, CHANGE); // Right 'panic1' function on any change of pin 2
  attachInterrupt(5, ISREncoderR, CHANGE); // Left 'panic2' function on any change of pin 3
  prevTime = millis();
  prevCounterL = car.getCounterL();
  prevCounterR = car.getCounterR();
}

// The controller for the movement of the car
void loop() {

  car.getReferenceFromJoystick();
  int desiredAngle = car.getDesiredAngle();
  float desiredSpeed = car.getDesiredSpeed();
  int pwmL, pwmR;
  if (desiredSpeed > 0) {
    // Get speed
    int currentCounterL = car.getCounterL();
    int currentCounterR = car.getCounterR();
    // Serial.print("current counter L: ");
    // Serial.println(currentCounterL);
    float actualSpeedL = car.getWheelSpeed(currentCounterL, prevCounterL, INTERVAL);
    float actualSpeedR = car.getWheelSpeed(currentCounterR, prevCounterR, INTERVAL);
    prevCounterL = currentCounterL;
    prevCounterR = currentCounterR;
    
    // PID control
    errorL = desiredSpeed - actualSpeedL;
    errorR = desiredSpeed - actualSpeedR;
    // Serial.print("Desired Speed: ");
    // Serial.println(desiredSpeed);
    // Serial.print("Actual Speed L: ");
    // Serial.println(actualSpeedL);
    // Serial.print("Error L: ");
    // Serial.println(errorL);

    integratedErrorL += errorL * INTERVAL / 1000;
    integratedErrorR += errorR * INTERVAL / 1000;

    float controlSignalL = PID(errorL, prevErrorL, integratedErrorL, kp, ki, kd, INTERVAL);
    float controlSignalR = PID(errorR, prevErrorR, integratedErrorR, kp, ki, kd, INTERVAL);
    // Serial.print("Control Signal L: ");
    // Serial.println(controlSignalL);
    prevErrorL = errorL;
    prevErrorR = errorR; 

    // PWM
    // controlSignalL = constrain(controlSignalL * 10, 0, 50);
    // controlSignalR = constrain(controlSignalR * 10, 0, 50);
    pwmL = map(controlSignalL, 0, 3, 230, 255);
    pwmR = map(controlSignalR, 0, 3, 230, 255);
  }
  else {
    pwmL = 0;
    pwmR = 0;
  }
  Serial.print("PWM L: ");
  Serial.println(pwmL);

  while (millis() - prevTime < INTERVAL) { }
  prevTime = millis(); // Update the previous time for the next iteration
  
  car.steer(desiredAngle); 
  car.move(pwmL, pwmR);
}

int PID(int error, int prevError, int errorIntegrated, int kp, int ki, int kd, unsigned long loopTime){
  float controlSignal;
  controlSignal = kp*error + ki*errorIntegrated + kd*(error-prevError)/(loopTime/1000);
  return controlSignal;
}

void ISREncoderL() {
  car.encoderL();
}

void ISREncoderR() {
  car.encoderR();
}
