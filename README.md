# RPA-for-Industrial-Warehouses-using-Swarm-Behavior
## Objective:

This is the final year project of four EEE students from PES University. 
The main idea behind the project was to give an objective or an application for the concept of swarm robotics. The objective was to build a multi-robot system consisting of 2 or 3 robots which can move around and localize themselves in an unknown environment and perform certain tasks.The main task which we provided was to pick-and-place goods/packages in an environment which would be emulated as a warehouse environment.
This project would be a prototype-of-sorts to further enhance warehouse activities by automation.  

### The main technical domains in the project are:
1. Mapping and Localization
2. Computer Vision
3. Navigation and Path planning
4. Control Law for Multi-robot communication system

### Hardware Details
The hardware consists of 2 omni-directional robots which 4 have mecanum wheels with encoded motors.
1. The first robot has Arduino Mega board to control the motors for PWM control and also to produce interrupts for the wheel encoders and Jetson Nano to handle the computations.
Sensors used: ZEDmini Camera for visualization and wheel encoders for odometry
2. The second robot also has an Arduino Mega for the same purpose but uses a Raspberry Pi 4 for the computation.
Sensors used: Wheel encoders for odometry

### Software Details
Using ROS Navigation Stack for Autonomous Navigation of the robots.
## - Prerequisites
1. 
## - Usage

