# ESP32 Self-Balancing Robot

![Language](https://img.shields.io/badge/Language-C%2B%2B-blue)
![Platform](https://img.shields.io/badge/Platform-ESP32-green)
![Status](https://img.shields.io/badge/Status-In%20Development-yellow)

## 📖 Project Overview
This project explores control theory and sensor fusion by creating an unstable inverted pendulum robot. The system uses a **PID controller** running on an **ESP32** to maintain vertical equilibrium, compensating for drift and external disturbances in real-time.

The goal was to move beyond abstract mathematical theory and apply it to a physical, real-world engineering challenge.

**Key Features:**
* **Active Balancing:** 100Hz PID control loop.
* **Sensor Fusion:** Complementary Filter to combine Accelerometer and Gyroscope data.
* **Deadzone Compensation:** Software offset to overcome motor friction.
* **Hardware:** Custom 3D printed chassis (OpenSCAD).

---

## ⚙️ Hardware Architecture

### Bill of Materials
| Component | Model | Purpose |
| :--- | :--- | :--- |
| **Microcontroller** | ESP32 DevKit V1 | Main processing unit (Dual Core). |
| **IMU Sensor** | MPU6050 | Provides 6-axis motion tracking (Accel + Gyro). |
| **Motor Driver** | L298N | H-Bridge driver for DC motor control. |
| **Actuators** | 2x N20 Gear Motors (6V) | High-torque motors for wheel drive. |
| **Power** | 2x 18650 Li-Ion Cells | 7.4V power supply. |

### Wiring Logic
* **Motors:** Powered directly from 7.4V battery source.
* **Logic:** ESP32 powered via L298N 5V regulator output.
* **Sensor:** MPU6050 communicates via I2C (GPIO 21 SDA, GPIO 22 SCL).

---

## 🧮 The Control Logic (PID)

The robot relies on a feedback loop where the error is the difference between the current angle and the target angle (0°).

**The Equation:**
$$Output = K_p e(t) + K_i \int e(t) dt + K_d \frac{de(t)}{dt}$$

* **Proportional ($K_p$):** Provides the immediate reaction to leaning.
* **Integral ($K_i$):** Corrects steady-state error (if the robot balances but leans slightly).
* **Derivative ($K_d$):** Dampens the oscillation to prevent the robot from overshooting.

**Sensor Fusion Strategy:**
Raw sensor data is noisy. I implemented a **Complementary Filter** to clean the signal:
```cpp
// 96% Gyro (fast response) + 4% Accel (stability)
angle = 0.96 * (angle + gyro_rate * dt) + 0.04 * accel_angle;# ESP32-Self-Balancing-Robot