// Kalman Filter
// Author: Yiyang Jiang

#include <BasicLinearAlgebra.h>
#include <Wire.h>
#include <LPS.h>
#include <LSM6.h>

LSM6 imu;
LPS ps;

using namespace BLA;

int N = 1;
double deltaT = 0.05;
// Define Kalman Filter Matrix
BLA::Matrix<3, 3> A = {1, deltaT, pow(deltaT, 2.0)/2.0,
                       0, 1, deltaT,
                       0, 0, 1};
BLA::Matrix<3, 2> K;
BLA::Matrix<3, 3> Q = {pow(deltaT, 6.0)/6.0, pow(deltaT, 5.0)/5.0, pow(deltaT, 4.0)/4.0,
                       pow(deltaT, 5.0)/5.0, pow(deltaT, 3.0)/3.0, pow(deltaT, 2.0)/2.0,
                       pow(deltaT, 4.0)/4.0, pow(deltaT, 2.0)/2.0, deltaT};
BLA::Matrix<2, 2> R;
BLA::Matrix<3, 3> I = {1, 0, 0,
                       0, 1, 0,
                       0, 0, 1};
BLA::Matrix<3, 3> P = {1, 0, 0,
                       0, 1, 0,
                       0, 0, 1}; 
BLA::Matrix<3, 3> P_pred;
BLA::Matrix<3, 1> x = {0, 0, 0};
BLA::Matrix<3, 1> x_pred;                 
BLA::Matrix<2, 1> z;
BLA::Matrix<2, 3> H = {1, 0, 0,
                       0, 0, 1};

unsigned long startTime;
float z0mean = 0.0;
float z0var = 0.0;
float z1mean = 0.0;
float z1var = 0.0;
float z0z1var = 0.0;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  if (!ps.init())
  {
    Serial.println("Failed to autodetect pressure sensor!");
    while (1);
  }
  ps.enableDefault();

  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  imu.enableDefault();

}

void loop() {
  startTime = millis();
  // Get data from IMU and barometer 
  float pressure = ps.readPressureMillibars();
  float altitude = ps.pressureToAltitudeMeters(pressure);
  z(0) = altitude + 71;
  imu.read();
  z(1) = (-1)*(0.061*0.001*9.81*imu.a.z + 9.81);

  // Calculate noise covariance matrix
  z0mean = (z0mean * (N - 1) + z(0)) / N;
  z1mean = (z1mean * (N - 1) + z(1)) / N;
  z0var = (z0var * (N - 1) + (z(0) - z0mean) * (z(0) - z0mean)) / N;
  z1var = (z1var * (N - 1) + (z(1) - z1mean) * (z(1) - z1mean)) / N;
  z0z1var = (z0z1var * (N - 1) + (z(0) - z0mean) * (z(1) - z1mean)) / N;
  R = {z0var, z0z1var,
       z0z1var, z1var}; 
       
  // KF formula      
  x_pred = A * x;
  P_pred = A * P * ~A + Q;
  K = P_pred * ~H * Inverse(H * P_pred * ~H + R);  // Kalman gain
  x = x_pred + K * (z - H * x_pred);  // the Kalman estimate of the state
  P = (I - K * H) * P_pred;

  Serial.print(z(0));
  Serial.print(" ");
  Serial.print(x(0));
  Serial.println(" ");

  N++;
  while(millis() - startTime < 50){}

}
