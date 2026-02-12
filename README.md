# franka_panda_moveit
# Franka Panda – MoveIt 2 Cartesian Planning (C++)

## Overview

This project implements Cartesian and joint-space motion planning for the Franka Panda robotic arm using **MoveIt 2** and **ROS 2 (Jazzy)**.
The system runs entirely in simulation using fake hardware and RViz.

The implementation demonstrates:

* MoveGroupInterface usage in C++
* Cartesian motion with `computeCartesianPath()`
* Named-target joint motion
* Collision object integration and avoidance behaviour
* Structured ROS 2 node design

---

## 1. Complete Source Code

The repository contains the following structure:

```
panda_moveit_assignment/
│
├── src/
│   └── cartesian_planner.cpp
│
├── launch/
│
├── config/
│
├── CMakeLists.txt
├── package.xml
└── README.md
```

Main node:

```
src/cartesian_planner.cpp
```

---

## 2. Build and Run Instructions

### Dependencies

* ROS 2 Jazzy
* MoveIt 2
* moveit_resources_panda_moveit_config

Install dependencies:

```
sudo apt install ros-jazzy-moveit ros-jazzy-moveit-resources-panda-moveit-config
```

---

### Build Workspace

```
cd ~/panda_assignment_ws
colcon build --symlink-install
source install/setup.bash
```

---

### Launch MoveIt Panda Demo

```
ros2 launch moveit_resources_panda_moveit_config demo.launch.py use_fake_hardware:=true use_sim_time:=true
```

Wait until RViz loads and the robot model is visible.

---

### Run Cartesian Planner Node

```
ros2 run panda_moveit_assignment cartesian_planner
```

---

## 3. Planning Logic (Short Explanation)

The planner performs three main stages:

### Joint-Space Initialization

The robot first moves to a predefined **named target ("ready")** using MoveGroupInterface.
This ensures a valid start configuration before Cartesian planning.

---

### Collision Scene Setup

A box (`Box_0`) is added programmatically to the planning scene using `PlanningSceneInterface`.
This object is used to demonstrate collision-aware behaviour during motion planning.

---

### Cartesian Motion Strategy

The motion is divided into two Cartesian segments:

1. **Waypoint 1 → Waypoint 2**

   * Straight-line Cartesian path
   * Executed successfully

2. **Waypoint 2 → Waypoint 3**

   * Path intersects with the collision box
   * Planning fraction drops below threshold
   * Node logs collision detection and aborts execution

`computeCartesianPath()` is used strictly for straight-line interpolation between waypoints.

---

## 4. RViz Execution (Screenshot / Recording)

Include a screenshot or recording showing:

* Panda robot in RViz MotionPlanning panel
* Collision box visible in the scene
* Robot executing first Cartesian motion
* Second motion failing due to collision

Suggested capture:

* RViz MotionPlanning view
* Planning Scene tab showing Box_0
* Robot trajectory visualization

Example filename:

```
docs/rviz_execution.png
```

---

## Notes

* Implemented using **C++17**.
* No hardcoded sleeps used.
* Executor spinning ensures correct state updates.
* Clean logging used for debugging and evaluation.

---
