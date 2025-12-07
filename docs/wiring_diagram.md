# 🔌 Wiring Diagram – Arduino Robot

This document explains how to wire the components of the Arduino Robot.

## 🧰 Required Components

- Arduino Uno R3
- V5 Expansion Board
- L298N Motor Driver
- HC-SR04 Ultrasonic Sensor
- SG90 Servo Motor
- IR Receiver
- 4x DC Motors
- Jumper Wires
- Power Supply (Battery Pack or USB)

---

## 🧷 Pin Connections

| Component        | Arduino Pin     | Notes                         |
|------------------|------------------|-------------------------------|
| Ultrasonic Trigger | 9               | Digital Output                |
| Ultrasonic Echo    | 10              | Digital Input                 |
| Servo Motor        | 11              | PWM Output                    |
| Motor Driver IN1   | 6               | Motor A                       |
| Motor Driver IN2   | 7               | Motor A                       |
| Motor Driver IN3   | 12              | Motor B                       |
| Motor Driver IN4   | 13              | Motor B                       |
| ENA (L298N)        | 5V + PWM (5)    | Enable Motor A (PWM capable) |
| ENB (L298N)        | 5V + PWM (2)    | Enable Motor B (PWM capable) |
| Power              | 5V / GND        | Shared Ground is important    |

> ⚠️ Make sure to power the motors separately from the Arduino (through the motor driver), especially if using 4 DC motors.
