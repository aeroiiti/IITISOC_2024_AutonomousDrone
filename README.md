# Autonomous Navigation in Drones
This project is a collaborative effort by:

- **Nambiar Anand Sreenivasan** - [GitHub: @NambiarAnand](https://github.com/NambiarAnand)
- **Lavanya Bhatnagar** - [GitHub: @Lavanya-1133](https://github.com/Lavanya-1133)
- **Rohan Dhiman** - [GitHub: @rohandhiman5](https://github.com/rohandhiman5)
- **R Gopikrishnan** - [GitHub: @Rgopikrishnan18](https://github.com/Rgopikrishnan18)

## Overview

This project aims to develop autonomous navigation capabilities for drones using ROS 2, Gazebo, and the Navigation 2 (Nav2) stack. The project was partially completed with significant achievements in setting up drone teleoperation and TurtleBot autonomous navigation.

## Table of Contents

1. [Project Overview](#project-overview)
2. [Drone Setup and Teleoperation](#drone-setup-and-teleoperation)
3. [Autonomous Navigation of TurtleBot](#autonomous-navigation-of-turtlebot)
4. [Map Generation Using LIDAR](#map-generation-using-lidar)
5. [References](#references)

## Project Overview

The project is divided into two main objectives:

1. **Autonomous Navigation of a Drone**: The goal was to develop a drone capable of autonomous navigation. While full autonomous capabilities were not achieved, we successfully set up a drone model that can be controlled via teleoperation within a simulated environment.

2. **Autonomous Navigation of a TurtleBot**: Using the Navigation 2 (Nav2) stack, we implemented autonomous navigation for a TurtleBot, including path planning, localization, and obstacle avoidance.

## Drone Setup and Teleoperation

### Prerequisites

To set up the drone model, the following software must be installed:

- **ROS 2 Humble**: ROS 2 (Robot Operating System) is an open-source framework for robotics software development. The installation guide for ROS 2 Humble can be found [here](https://docs.ros.org/en/humble/Installation.html).
- **Gazebo 11**: Gazebo is a robotics simulator that provides accurate and realistic simulations. The installation guide for Gazebo 11 is available [here](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install).
- **RViz 2**: RViz is a visualization tool for ROS. It helps in visualizing sensor data and robot states. RViz is included in the ROS 2 packages.
- **Necessary Plugins**: These plugins integrate Gazebo with ROS 2.

  ```bash
  sudo apt-get install ros-humble-gazebo-plugins ros-humble-gazebo-ros
  ```

### Setup Instructions

1. **Fork the Repository**

   Fork the repository from [Autonomous-Drones-SoC](https://github.com/Rgopikrishnan18/Autonomous-Drones-SoC) to create your own copy.

2. **Install Dependencies**

   Navigate to the project folder and install dependencies using ROS 2’s `rosdep` tool:

   ```bash
   rosdep install -r -y --from-paths src --ignore-src --rosdistro humble
   ```

   This command installs all the required dependencies specified in the `package.xml` files of the ROS 2 packages.

3. **Build the Workspace**

   Build the ROS 2 workspace using `colcon`:

   ```bash
   colcon build --packages-select-regex sjtu*
   ```

   This command compiles the source code in the workspace, focusing on packages that match the `sjtu*` pattern.

4. **Source the Installation**

   After building, source the setup file to set up the environment for ROS 2:

   ```bash
   source install/setup.bash
   ```

   This command configures the environment variables needed for ROS 2.

### Teleoperation

1. **Launch the System**

   Launch the drone model along with RViz, Gazebo, and teleoperation windows using:

   ```bash
   ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py
   ```

   This command starts all necessary nodes and tools for the simulation and control of the drone.

2. **Take Off**

   To take off the drone, publish an empty message to the `/drone/takeoff` topic:

   ```bash
   ros2 topic pub /drone/takeoff std_msgs/msg/Empty {}
   ```

   This command sends a takeoff command to the drone.

3. **Navigate Using Teleoperation**

   Control the drone using the teleoperation window with the keys: `{'u','i','o','h','j','k','m',',','.','t','b'}`. Each key corresponds to different directions and actions for the drone.

4. **Land the Drone**

   To land the drone, publish an empty message to the `/drone/land` topic:

   ```bash
   ros2 topic pub /drone/land std_msgs/msg/Empty {}
   ```

   This command sends a landing command to the drone.

## Autonomous Navigation of TurtleBot

### Overview

Autonomous navigation for the TurtleBot was achieved using the Navigation 2 (Nav2) stack. Nav2 provides a comprehensive framework for robot navigation, including path planning, localization, and obstacle avoidance.

### Setup Instructions

1. **Initial Setup**

   Follow the tutorial for setting up Nav2 on a TurtleBot3 [here](https://docs.nav2.org/tutorials/docs/navigation2_on_real_turtlebot3.html). This guide covers the installation and configuration of the Nav2 stack on TurtleBot3.

2. **Setting Initial and Final Poses**

   In RViz, use the "2D Pose Estimate" tool to set the robot’s initial pose and the "2D Nav Goal" tool to specify the final destination. This process initializes the robot’s starting position and target location for navigation.

3. **Navigation**

   The TurtleBot uses the Nav2 stack to plan a path from the initial pose to the goal pose. Nav2 handles the path planning, obstacle avoidance, and control of the robot’s movement.

### Explanation

The Nav2 stack includes:

- **Global Planner**: Computes a high-level path from the start to the goal.
- **Local Planner**: Adjusts the path in real-time to avoid obstacles and navigate around them.
- **Costmaps**: Represent the environment, including static and dynamic obstacles, to help the robot plan its path.
- **Localization**: Helps the robot determine its position on the map using sensor data.

## Map Generation Using LIDAR

### Overview

Map generation involves creating a map of the environment using LIDAR (Light Detection and Ranging) sensors. This map is essential for navigation and obstacle avoidance.

### Steps

1. **Setup LIDAR Sensor**

   Ensure that the LIDAR sensor is properly integrated into your ROS 2 setup. The sensor should be configured to publish data to relevant ROS 2 topics.

2. **Generate Map**

   Use mapping tools like `slam_toolbox` or `hector_slam` to create a map from the LIDAR data. These tools use sensor data to build a 2D or 3D map of the environment.

3. **Save Map**

   Save the generated map using the `map_saver` node, which allows you to store the map in a file format suitable for later use.

   ```bash
   ros2 run map_server map_saver_cli -f <map_name>
   ```

## References

- [ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- [Gazebo 11 Installation](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)
- [RViz Documentation](https://docs.ros.org/en/humble/Visualizing.html)
- [Nav2 Tutorial for TurtleBot3](https://docs.nav2.org/tutorials/docs/navigation2_on_real_turtlebot3.html)
- [Navigation2 Documentation](https://docs.nav2.org/getting_started/index.html)
- [Medium Article on ROS 2 Mobile Robotics](https://medium.com/@thehummingbird/ros-2-mobile-robotics-series-part-2-e8dd6116aacb)


