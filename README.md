🚗 Two-Wheel Self-Balancing Robot using PID Control


<img width="580" height="580" alt="image" src="https://github.com/user-attachments/assets/0f0c71b2-6579-4101-877f-f1d402a53038" />

📌 Introduction

This project presents the design and implementation of a two-wheel self-balancing robot based on the inverted pendulum model.
The system uses an STM32F407VET6 microcontroller to read sensor data, estimate the tilt angle using a Kalman Filter, and stabilize the robot using a dual-loop PID control algorithm.

The project was developed as part of the Embedded System Design course at the University of Information Technology – VNUHCM.

🎯 Project Objectives

Design and build a two-wheel self-balancing robot

Implement angle estimation using MPU6050 (Accelerometer + Gyroscope)

Apply Kalman Filter for sensor fusion

Develop a PID controller for stable balancing

Control DC motors using PWM and H-Bridge driver

Implement encoder-based velocity feedback

🧠 System Architecture

The system consists of four main functional blocks:

MPU6050 Sensor Interface

Reads acceleration and angular velocity via I2C

Computes initial roll angle

Kalman Filter Processing

Fuses accelerometer and gyroscope data

Reduces noise and gyro drift

Outputs stable tilt angle

PID Controller

Outer loop: Angle stabilization

Inner loop: Velocity control (using encoder feedback)

Produces motor control signal

Motor Controller

Controls direction via L298N H-Bridge

Controls speed via PWM (TIM1, TIM2)

Encoder feedback via TIM3, TIM5

⚙️ Hardware Components
Component	Description
STM32F407VET6	Main microcontroller
MPU6050	6-axis IMU sensor (I2C)
L298N	Dual H-Bridge motor driver
JGA25-370 DC Motor with Encoder	12V DC motor (130 RPM, 1:45 ratio)
LM2596 Buck Converter	12V → 5V voltage regulator
18650 Battery (3 cells)	12V power supply
🔌 Peripheral Configuration

System Clock: 72 MHz (HSE + PLL)

I2C1: 100 kHz (MPU6050 communication)

TIM1 & TIM2: PWM generation (Motor control)

TIM3 & TIM5: Encoder mode

TIM4: 10ms interrupt (Control loop timing)

USB CDC: Real-time monitoring & PID tuning

🧮 Control Algorithm
1️⃣ Angle Estimation

Roll angle from accelerometer:

Roll = atan(Ay / sqrt(Ax² + Az²)) × 180 / π

Gyroscope integration:

θ = θ_previous + ω × Δt

Kalman Filter fuses both signals to produce a stable angle.

2️⃣ PID Controller

The control signal is calculated as:

Output = Kp * error + Ki * ∫error + Kd * d(error)/dt

Two PID loops:

Angle PID (stabilization)

Velocity PID (motor speed control)

📊 Default PID Parameters
// Angle PID
Kp = 1325
Ki = 500
Kd = 43

// Velocity PID
Kp = 56
Ki = 2
Kd = 0.2

These parameters were experimentally tuned for optimal stability.

💻 Software Structure
Core/
 ├── main.c
 ├── pid.c
 ├── motor.c
 ├── mpu6050.c
 ├── kalman.c

Key Files:

main.c → System initialization & control loop

pid.c → PID algorithm implementation

motor.c → Motor direction & PWM control

mpu6050.c → Sensor interface

kalman.c → Sensor fusion

🖥️ USB Command Interface

The system supports real-time tuning via USB CDC.

Commands:

Tune PID:

T <Kp> <Ki> <Kd>

Check Status:

S

Show Help:

M
🔄 Control Loop Timing

Timer interrupt every 10 ms

Sensor read → Kalman filter → PID → Motor control

📈 Performance

Stable vertical balancing

Fast disturbance recovery

Smooth motor response

Reduced noise using Kalman Filter

Encoder-based velocity feedback

🚀 Future Development

ESP32 wireless control

Line-following mode

Obstacle avoidance (Ultrasonic sensor)

Camera integration (ESP32-CAM)

Advanced control (LQR / State-Space control)

👨‍💻 Authors

Dương Thanh Hiếu – 23520475

Trần Triệu Dân – 23520223

Nguyễn Quốc Cường – 23520204

Instructor: Trần Ngọc Đức

University of Information Technology – UIT
Embedded System Design – 2025

📈 Wiring diagram for a DC motor
<img width="975" height="683" alt="image" src="https://github.com/user-attachments/assets/2d7b3d0a-c3a4-4173-9960-d3665685e991" />
