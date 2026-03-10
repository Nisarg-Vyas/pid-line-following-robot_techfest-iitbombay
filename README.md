# PID Line Following Robot (Techfest IIT Bombay – Meshmerize)

A high-speed PID-based Line Following Robot designed for the **Meshmerize competition organised by Techfest IIT Bombay**.

The robot secured **3rd place in the Zonal round at IIT Gandhinagar** and directly qualified for the **final round**.

The robot uses an **8-channel reflectance sensor array**, a **PID control algorithm**, and a **dual H-bridge motor driver** to achieve stable and fast line tracking. (check [lfr_demo](videos/lfr_demo.mp4))

---

# Competition Performance

The robot performed very well in the zonal round and achieved **3rd position**, qualifying directly for the finals.

During the final round, the robot was tuned for **higher speed**, which caused the sensors to miss the line on sharp turns. However, at moderate speeds the robot was able to follow the track very accurately.

---

# Features

* PID based line following
* 8-channel reflectance sensor array
* Junction detection
* U-turn detection
* Dead-end detection
* Finish box detection
* Adjustable motor speed
* High-speed optimized control

---

# Hardware Components

Main components used:

* Arduino Nano
* QTR-8RC reflectance sensor array
* TB6612FNG motor driver
* N20 DC gear motors
* LiPo battery (7.4V)
* Buck converter for regulated voltage
* LEDs for status indication

The complete component list is available in:

[View COMPONENTS](docs/components.txt)

---

# System Architecture

The robot consists of three main subsystems:

### Sensor System

An **8-channel reflectance sensor array** reads the track.

Each sensor returns a value representing how much light is reflected from the surface.

* Dark line → low reflectance
* White surface → high reflectance

The sensors are read through the Arduino analog pins.

---

### Control System

The robot uses a **PID controller** to adjust motor speeds based on the line position.

The PID algorithm calculates the error between the detected line position and the center of the robot.

Correction formula:

```
correction = Kp * error + Kd * (error - lastError)
```

Motor speeds are adjusted accordingly to steer the robot back onto the line.

---

### Drive System

Two **N20 DC gear motors** drive the robot using a differential drive configuration.

The **TB6612FNG motor driver** controls direction and speed using PWM signals from the Arduino.

---

# Code Explanation

The control logic is implemented using a **state machine**.

States include:

* FOLLOW_LINE
* TURN_LEFT_STATE
* TURN_RIGHT_STATE
* UTURN_STATE
* FINISH_STATE

The robot continuously reads sensor values and determines which state to enter.

---

# Line Following Algorithm

The robot reads all 8 sensors and calculates a weighted average of their readings.

Example sensor weights:

```
{-3, -2, -1, -0.5, 0.5, 1, 2, 3}
```

These weights estimate the line position relative to the robot center.

The error is then fed into the PID controller to compute steering correction.

---

# Junction Detection

The robot checks specific sensors to determine junctions.

Left junction:

```
sensor[6] or sensor[7]
```

Right junction:

```
sensor[0] or sensor[1]
```

Straight path detection is based on center sensors.

---

# Dead-End Detection

If no valid path is detected, the robot identifies a **dead end** and performs a **U-turn**.

---

# Finish Detection

A white finish box is detected when all sensors detect a white surface for a sustained period.

Once detected:

* motors stop
* finish LED turns ON

---

# Pin Configuration

Motor Driver:

| Arduino Pin | Function |
| ----------- | -------- |
| 7           | AIN1     |
| 6           | AIN2     |
| 5           | PWMA     |
| 9           | BIN1     |
| 10          | BIN2     |
| 11          | PWMB     |
| 8           | STBY     |

Sensor Enable:

| Arduino Pin | Function         |
| ----------- | ---------------- |
| 4           | IR Sensor Enable |

Finish LED:

| Arduino Pin | Function         |
| ----------- | ---------------- |
| 13          | Finish Indicator |

Complete wiring details are available in:

docs/pinout_wiring.txt

---

# Future Improvements

* adaptive PID tuning
* higher sensor sampling rate
* dynamic speed control
* improved corner detection
* hardware encoder feedback

---

# Author
Nisarg Vyas  
B.Tech Computer Science and Engineering  
IIIT Vadodara (IIITV-ICD)
