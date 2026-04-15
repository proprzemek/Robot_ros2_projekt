# ROS 2 Autonomous Robot Simulation & Control

## Overview
This repository contains a modular ROS 2 system for a mobile robot. It is split into two main packages:
1. **my_robot_sim**: Robot description (URDF), Gazebo worlds, and launch configurations.
2. **my_robot_controller**: C++ implementation of the navigation and control logic.

## Project Structure
* **my_robot_sim/**
  * `urdf/`: Robot model and sensor definitions.
  * `worlds/`: Custom Gazebo simulation environments.
  * `launch/`: ROS 2 launch files for simulation setup.
* **my_robot_controller/**
  * `src/navigation_node.cpp`: Main C++ node for robot guidance and navigation.

## Tech Stack
* **OS:** Ubuntu 24.04 (Noble Numbat)
* **Middleware:** ROS 2 Jazzy Jalisco
* **Simulation:** Gazebo
* **Language:** C++ (C++17/20)

## How to Build
1. Create a workspace: `mkdir -p ~/ros2_ws/src`
2. Clone this repo into `src/`
3. Install dependencies: `rosdep install --from-paths src -y --ignore-src`
4. Build: `colcon build --symlink-install`
