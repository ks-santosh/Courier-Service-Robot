# Autonomous Courier Service Robot

This project is part of the e-Yantra Robotics Competition Plus (eYRC+) 2015, focusing on developing an automated courier service using a programmed robot to efficiently pick up and deliver virtual packages within a city-like arena.


## Table of Contents
1. [Project Overview](#project-overview)
2. [Demo](#demo)
3. [Features](#features)
4. [Hardware Used](#hardware-used)
5. [Software and Libraries](#software-and-libraries)
6. [Installation and Setup](#installation-and-setup)
7. [Acknowledgments](#acknowledgements)

## Project Overview

The robot is designed to navigate a pre-loaded map, represented as a 6x6 grid with designated nodes for picking up and delivering packages. The robot uses image processing techniques to recognize packages by their shapes and colors and detects traffic lights to adjust its path dynamically. The main objective is to enhance the efficiency of courier services by using autonomous robots.

The following files are provided in the [src](/src/)
| File | Purpose |
| ---  | --- |
| `CourierServiceMain.py`         | This is the main file to be executed. It takes an input image and controls the overall execution of the robot's tasks, including navigating the grid, picking up packages, and handling traffic signals.|
| `GraphOfGrid.py`     | This script is responsible for converting the grid into a graph. It creates an adjacency list representation of the grid to facilitate pathfinding algorithms for efficient navigation. |
| `PackageDetails.py` | This script provides details about the packages, including their shapes and colors. It also identifies the pick-up and delivery junctions for each package, ensuring the robot knows where to collect and deliver them. |
| `TrafficSignals.py`       | This script determines the positions of the traffic signals within the grid. It helps the robot identify where traffic signals are located and their states (red or blue) to decide whether to wait or reroute. |
| `PathTraversal.py`       | This script controls the motors and ensures the robot follows the black lines on the grid.|
|`ProjectExplanation.pdf`| Contains the details about the working of robot and the different algorithms used.|

The [package](/package) folder folder contains the source code for setting up client-server communication between the robot and the computer. 

[testImages](/testImages) contains the example images that represent the grid map that the robot navigates.

Refer the [cs_rulebook.pdf](cs_rulebook.pdf) for more details about the competition and the project.

## Demo

The video has been sped up to shorten its duration and size.

https://github.com/ks-santosh/Courier-Service-Robot/assets/164272377/fbd3bb01-ff7c-4c03-b2a6-cba939d03b74

## Features

- **Pathfinding:** Utilizes shortest path algorithms to navigate the grid.
- **Package Identification:** Recognizes packages to be delivered based on color and shape using image processing.
- **Traffic Light Detection:** Adjusts route based on the status of traffic signals (red or blue).
- **Autonomous Navigation:** Follows paths based on the map and also dynamically adjusts in response to real-time inputs.

## Hardware Used

- **Raspberry Pi B+:** Central controller of the robot.
- **Edimax WiFi Dongle:** For wireless communication.
- **8GB Micro SD Card:** Contains the Raspbian image and necessary software.
- **Intex USB Camera:** Captures images for processing.
- **BO Motors:** Drives the robot's wheels.
- **L293D Motor Driver IC:** Controls the motors.
- **Zook 5000 mAH Power Bank:** Powers the Raspberry Pi.
- **Chassis, Wheels, Castor Wheels, Nuts, and Bolts:** Structural components of the robot.

## Software and Libraries

- **Operating System:** Raspbian (e-Yantra version).
- **Programming Language:** Python.
- **Libraries:**
  - **OpenCV:** For image processing.
  - **Numpy:** For numerical operations.
  - **time:** For managing delays (e.g., waiting at traffic lights).

## Installation and Setup

1. **Prepare the Raspberry Pi:**
   - Install the e-Yantra Raspbian image on the Micro SD card using Win32DiskImager or a similar tool.
   - Insert the Micro SD card into the Raspberry Pi and power it using the Zoook power bank.

2. **Hardware Assembly:**
   - Assemble the robot chassis and attach the wheels, motors, and castor wheels.
   - Connect the motors to the Raspberry Pi via the L293D Motor Driver IC.
   - Mount the USB camera on the robot.

3. **Software Setup:**
   - Transfer the test image to the Raspberry Pi using a USB storage device.
   - Ensure all necessary Python libraries (OpenCV, Numpy) are installed.
   - Write and upload the main control program to the Raspberry Pi.

## Running the Robot

1. **Arena Setup:**
   - Print the arena design on a flex sheet and set it up according to the provided guidelines.
   - Place roadblocks and traffic signals as specified in the test image and arena configuration image.

2. **Execution:**
   - Place the robot at the start position (green junction).
   - Use remote login to run the control program on the Raspberry Pi.
   - Monitor the robot as it navigates, picks up, and delivers packages.


- [e-Yantra Official Website](http://www.e-yantra.org)
- [e-Yantra Competition Rulebook](./cs_rulebook.pdf)

## Acknowledgments

- e-Yantra team at IIT-Bombay for organizing the competition and providing resources.
- All team members for their hard work and dedication.