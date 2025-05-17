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
| Ultrasonic Trigger | 7               | Digital Output                |
| Ultrasonic Echo    | 6               | Digital Input                 |
| Servo Motor        | 9               | PWM Output                    |
| Motor Driver IN1   | 12              | Motor A                       |
| Motor Driver IN2   | 11              | Motor A                       |
| Motor Driver IN3   | 10              | Motor B                       |
| Motor Driver IN4   | 2               | Motor B                       |
| ENA (L298N)        | 5V + PWM (13)   | Enable Motor A (PWM capable) |
| ENB (L298N)        | 5V + PWM (8)    | Enable Motor B (PWM capable) |
| Power              | 5V / GND        | Shared Ground is important    |

> ⚠️ Make sure to power the motors separately from the Arduino (through the motor driver), especially if using 4 DC motors.
