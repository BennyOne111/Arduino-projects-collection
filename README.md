# Arduino Projects

This repository contains Arduino-based projects. The code spans sensors, motor control, embedded communication, sensor fusion, and system integration.

## Folder Overview

### `sensors/`
Sensor reading and signal processing:
- **IR Distance Sensor**
- **Ultrasonic + OLED Display**
- **Virtual Trombone using hand proximity**
- **IMU and Barometer readers**

### `communication/`
Arduino-to-Arduino communication via:
- **RF transceivers (RFM69HCW)**
- **I2C (Master/Peripheral role)**

### `motors/`
Motor control experiments including:
- **DC motor with proportional control**
- **Stepper motor with full/half step & UI**
- **Encoder-based feedback**
- **Interrupt-driven interfaces**

### `kalman_filter/`
Sensor fusion:
- **Kalman filter combining IMU and barometric data for accurate altitude estimation**

### `remote_controlled_car/`
Integrated system:
- **RF joystick controller**
- **PID-based motor speed control**
- **Encoder feedback loop**
- **Finite State Machine logic**

## Author

**Yiyang Jiang**  
MSE in Mechanical Engineering, Johns Hopkins University  
[yjian138@jh.edu](mailto:yjian138@jh.edu)

---

Each folder contains `.ino` Arduino code and images. For more details, see individual README files.
