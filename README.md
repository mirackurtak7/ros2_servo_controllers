Servo Motor and Odometry Test Script:
This project provides a PowerShell-based test automation script to verify the functionality of servo motor control and odometry feedback for a micro-ROS–enabled robot platform (e.g., ESP32 or STM32). It is designed to run on Windows + WSL2 with ROS 2 Humble installed inside WSL.

Features:
Checks if ROS 2 is installed and running in WSL2.
Lists available serial ports on both Windows and WSL2.
Provides ready-to-use ROS 2 test commands for quick verification.
Automates starting the micro-ROS Agent.
Includes structured test scenarios for servo control, odometry feedback, motion commands, and PID tuning.

Requirements:
Windows 10/11 with WSL2 enabled.
ROS 2 Humble installed in WSL2.
ESP32 or STM32 microcontroller with micro-ROS firmware.
USB cable or USB-TTL adapter.
Hobby servo motor (e.g., SG90, MG995).
External power source for servo motors (recommended).

Usage:
Connect your microcontroller (ESP32/STM32) via USB.
Run the PowerShell script:
.\servo_odometry_test.ps1
Follow the script prompts to:
Verify ROS 2 installation in WSL2.
List available COM/serial ports.
Start the micro-ROS Agent.
Use the provided commands to test servo movement, robot motion, odometry data, and PID parameter tuning.
