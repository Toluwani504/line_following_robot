# line_following_robot
This is an experimental first attempt at building a line-following robot for a school project.


# Description And Purpose

This project involves designing and building an autonomous mobile robot that uses an infrared (IR) sensor, an Arduino Uno, and an L298N motor driver to detect and follow a predefined path marked by a black line on a contrasting surface.

Purpose of the Project:

The purpose is to develop a robot capable of navigating a predefined route without continuous human control. It uses sensor readings to determine the position of the line and automatically adjusts the speed and direction of its motors to stay on track, including handling bends and sharp turns.

The project demonstrates practical applications of embedded programming, sensor integration, motor control, and autonomous robotics.


## 1. Project Overview
A line-following robot is an autonomous robot designed to follow a predefined path, usually a black line on a white surface.

Project Objective: To build a robot that can independently detect and follow a marked path using an IR sensor, Arduino Uno, and two DC motors.

Basic Operating Principle: The IR sensors detect the line and send signals to the Arduino, which processes the readings and controls the motors through the L298N driver to keep the robot on track.

Project Stage: Experimental prototype. The robot is currently undergoing sensor testing, motor calibration, and control-program development.

## 2. Project Objectives
- Design and build an autonomous mobile robot.
- Implement line detection using infrared sensors.
- Control DC motors based on sensor feedback.
- Explore embedded programming and basic robotics.

## 3. Hardware Components
List all the components used in the project.

| Component | Quantity | Purpose |
| Arduino Uno | 1 | Main controller |
| L298N Motor Driver | 1 | Motor control |
| IR Sensors | 2 | Line detection |
| DC Gear Motors | 2 | Robot movement |
| Robot Chassis | 1 | Mechanical structure |
| Battery | 2 | Power supply |
| Jumper Wires | As needed | Electrical connections |

## 4. System Architecture
The system operates through four main stages:

1. Sensor Input: The IR sensor detects the position of the black line and sends signals to the Arduino.

2. Arduino Processing: The Arduino reads and processes the sensor signals to determine the robot's direction and required movement.

3. Motor Driver Control: The Arduino sends control signals to the L298N motor driver, which regulates the motors' speed and direction.

4. Motor Output: The two DC geared motors drive the wheels, allowing the robot to move forward, turn, and correct its position along the path.

   
## 5. Circuit Connections

The Schematics is in the "Images Folder"

## 6. Software Implementation

Include:
- Programming language: Embedded C
- Development environment: Arduino IDE


## 9. Challenges Encountered

- Sensor calibration and detection issues
- Motor direction inconsistencies
- Difficulty maintaining the line during turns

## 10. Results and Observations

Line Following: The robot successfully follows straight lines and navigates small curves.
Turning Performance: It experiences difficulties navigating sharp turns accurately.
Sensor Response: The two IR sensors provide basic line detection but limited information about the line's position.
Motor Behaviour: The motors respond to control signals and allow the robot to move forward and make directional corrections.
Current Limitations: The use of only two sensors limits the robot's ability to detect sharp changes in direction and make precise corrections.


## 11. Future Improvements

Possible improvements include:
- Implementing an 8-channel IR sensor array.
- Improving sensor calibration.
- Implementing PID-based control.
- Optimising motor speed.
- Testing on more complex tracks.

## 12. Repository Structure
Explain how the project files are organised.

line-following-robot/
│
├── README.md
├── src/
│   └── line_follower.ino
│
├── images/
│   ├── robot_front.jpg
│   ├── robot_top.jpg
│   └── circuit_diagram.png
│
└── videos/
    └── demonstration_link.md

## 13. Lessons Learned
Through this project, I gained practical experience in:
- Embedded Programming: Writing and uploading Arduino code to control hardware.
- Sensor Integration: Understanding how IR sensors detect lines and provide input to a microcontroller.
- Motor Control: Using the L298N motor driver to control motor speed and direction.
- Troubleshooting: Identifying and correcting hardware and software problems during testing.
- Control Systems: Understanding how sensor feedback helps a robot adjust its movement.
- System Integration: Combining electronic components, programming, and mechanical parts to build a functional robotic system.

## 14. Author
Toluwani Kayode

- GitHub: https://github.com/Toluwani504
- LinkedIn: https://www.linkedin.com/in/toluwani-kayode-3789b335a?utm_source=share_via&utm_content=profile&utm_medium=member_android

