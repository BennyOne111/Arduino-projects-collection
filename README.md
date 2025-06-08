# Arduino Robotics Projects Collection

This repository contains a collection of Arduino-based robotics and embedded systems projects developed by **Yiyang Jiang** at Johns Hopkins University.

Projects span across sensors, motors, communication protocols, control algorithms, and system integration, demonstrating both fundamental and advanced techniques in embedded robotics.

---

## 📁 Folder Overview

### `sensors/`  
Sensor signal acquisition, environment perception, and actuator interaction:

- **IR Distance Measurement (`IR_Distance.ino`)**  
  Measures distance using a Sharp IR sensor and outputs to Serial Plotter.

- **Ultrasonic Ranging (`Ultrasoni_Sensor.ino`)**  
  Uses HC-SR04 to measure distance with time-of-flight and serial output.

- **Omni-Directional Radar Scanner (`Omni_Radar.ino`)**  
  Sweeps an ultrasonic sensor with a servo to simulate radar scanning.

- **Auditory Feedback (`Buzzer.ino`)**  
  Drives a piezo buzzer with modulated tone frequency based on distance or timing.

- **GPS Position Reader (`Final_Project_GPS.ino`)**  
  Parses GPS module data to extract latitude, longitude, altitude, and time.

---

### `motors/`  
DC and stepper motor control with feedback, user interface, and classic control strategies:

- **Proportional Control (`Proportional_Control.ino`)**  
  Controls DC motor speed using a potentiometer and adjustable P gain.

- **Step Response Visualization (`step_response.ino`)**  
  Plots DC motor step responses under varying gain values.

- **Encoder Feedback (`DC_Motor_With_Encoder.ino`)**  
  Tracks motor position using rotary encoder pulse counting.

- **PID Speed Control (`PID.ino`, `Final_Project_PID_Wheels.ino`)**  
  Implements PID velocity control with encoder feedback.

- **Full-Step Motor Control (`Full_Step_Motor_Control_with_Button_OneCircle.ino`)**  
  Rotates a 4-phase stepper motor one full turn per button press.

- **Half-Step Control (`Half_Step_Motor_Control.ino`)**  
  Increases resolution with half-step stepping logic.

- **Stepper UI Interface (`Final_Step_Motor_Control.ino`)**  
  Adds 3-button interface for single-step, full revolution, and reverse rotation.

---

### `communication/`  
Wired communication between Arduino boards using I2C and SPI protocols:

- **I2C Master/Peripheral (`I2C_Controller.ino`, `I2C_Per.ino`)**  
  Master sends ADC value; peripheral responds and triggers output based on threshold.

- **SPI with FSM (`SPI_Receive_A_Byte.ino`, `SPI_Finite_State_Machine.ino`)**  
  Demonstrates byte-level SPI data reception and finite state logic.

---

### `kalman_filter/`  
Sensor fusion for accurate altitude estimation:

- **Kalman Filter Fusion (`Kalman_Filter.ino`)**  
  Fuses barometer and IMU z-acceleration for vertical position estimation.  
  Implements prediction/update steps with defined A and H matrices.  
  Demonstrated during elevator ride test.

---

### `remote_controlled_car/`  
Integrated wireless mobile robot system:

- **RF Remote Control & PID Speed Regulation (`remote_controlled_car.ino`)**  
  Reads joystick input, sends via RF transceiver, and controls car motion with encoder-based PID loops.  
  Includes servo steering and FSM-based communication logic.

---

## 👤 Author

**Yiyang Jiang**  
MSE in Mechanical Engineering  
Johns Hopkins University  
📧 [yjian138@jh.edu](mailto:yjian138@jh.edu)  

---

## 📌 Notes

- All code is written for standard Arduino-compatible boards (e.g., UNO, MEGA).
- Libraries used include: `Wire.h`, `Servo.h`, `SPI.h`, `Adafruit_SSD1306`, `RadioHead`, etc.

---

## 📜 License

This project is licensed under the [MIT License](LICENSE).
