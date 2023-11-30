# UGR-internship

This is the developed code to perform a pick and place action on an object detected by a Vicon system and for a torque controller that uses the URDF to calculate the feedforward part.

## Overview

During my internship at the Universidad De Granada, a 7-DOF robot called Sawyer is integrated with both a Vicon vision system and 2 RGB cameras. All sensors and actuators are made available on a ROS network. To verify the functionality of all components a pick and place action is performed by using the Vicon system to detect the object pose and using the robot arm to move to that particular pose and pick up the object. After this verification was successfully performed a torque controller is implemented on the robot to create an increased amount of insight and control into the robot function by directly sending control torques to the joints. To this end a model-based control law is used that takes care of both feedforward and feedback. By using the same Euler-Lagrange model as in the control law a simple collision detection observer is implemented as well, which can detect the increased torque the robot is experiencing compared to the predicted amount of torque and determine if a collision is being made. 


## Features

- Feature 1: Pick and place using MoveIt trajectory planner.
- Feature 2: Torque controller with model-based feedforward and collision detection

## Table of Contents

1. [Requirements](#requirements)
2. [Installation](#installation)
3. [Usage](#usage)

## Requirements

List of hardware and software requirements for running the project. The installation of this software is not discussed in more detail here.

- Hardware:
  - Sawyer Robot
  - Vicon Vero System
  - Manta RGB Cameras (List specific models)
  - Development PC

- Software:
  - ROS 1: Melodic or Noetic (Robot Operating System) 
  - Python (Version 3)
  - [MoveIt](https://moveit.ros.org/)
  - [Vicon Tracker](https://www.vicon.com/software/tracker/)
  - [Vicon bridge](http://wiki.ros.org/vicon_bridge)
  - [Vimba AVT](http://wiki.ros.org/avt_vimba_camera)
  - [Vimba SDK]([https://www.alliedvision.com/en/products/software/](https://www.alliedvision.com/en/products/vimba-sdk/))
  - [Orocos KDL](https://github.com/orocos/orocos_kinematics_dynamics/blob/master/orocos_kdl/INSTALL.md)

## Installation

Below are the step-by-step instructions on how to install and set up the project.

1. Create ROS workspace
2. Install packages:
    - [InteraSDK](https://support.rethinkrobotics.com/support/solutions/articles/80000980134-workstation-setup)
    - [Gazebo Simulator](https://support.rethinkrobotics.com/support/solutions/articles/80000980381-gazebo-tutorial)
    - [MoveIt](https://support.rethinkrobotics.com/support/solutions/articles/80000980338-moveit-tutorial)
    - [Orocos KDL](https://github.com/orocos/orocos_kinematics_dynamics/blob/master/orocos_kdl/INSTALL.md) Install this package into a directory called repos in your home folder by following the steps on the page.
    - [AVT Vimba Camera](https://github.com/astuff/avt_vimba_camera) Install this package into ~/ros_ws/src.

3. Clone the repository:
   ```bash
   git clone https://github.com/your-username/your-repo.git

## Usage

   
