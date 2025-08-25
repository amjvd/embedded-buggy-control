# Autonomous Buggy – PID Control

An embedded systems project implementing **PID control** and **PWM motor logic** in C++ (MPLAB X) to navigate a track autonomously.  
The buggy was designed to make real-time adjustments and maintain stability, completing **4 laps in 75 seconds**.

## Features
- **PID Controller** for stable navigation
- **PWM Motor Control** for precise speed regulation
- **Real-time Micro-adjustments** for error correction
- **Iterative Debugging & Tuning** of PID variables to improve lap consistency and reduce drift

## Project Structure
- `main.cpp` – entry point, high-level control logic  
- `pid_controller.cpp/h` – PID algorithm implementation  
- `motor_control.cpp/h` – PWM motor logic  

## Technologies Used
- Language: **C++ (MPLAB X IDE)**  
- Hardware: **Embedded microcontroller + motors**  
- Concepts: **PID control, PWM, real-time systems**

## Results
- Completed **4 laps in 75 seconds**  
- Improved lap stability and reduced error drift through iterative testing and optimisation

## How to Run
1. Clone the repository
2. Open in MPLAB X IDE
3. Compile and flash onto supported microcontroller
4. Run buggy on track

## Authors
- **Amjad Hamed** (Newcastle University)
