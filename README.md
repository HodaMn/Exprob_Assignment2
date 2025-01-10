# Experimental Robotics Assignment 2 - Planning System and Localization

## Description
In this project we utilize the ROSPlan package to implement a planning system for a mobile robot equipped with a camera and laser scanner in a Gazebo environment. The environment features four waypoints, each associated with an ArUco marker. The robot's objective is to navigate to each waypoint, identify the marker ID at each location, and proceed to the next waypoint. This process continues until all markers are detected, guided by PDDL actions. Upon completing the detection phase, the robot navigates to the waypoint associated with the marker that has the smallest ID. The system leverages a custom action interface to execute planned tasks and ensures obstacle avoidance, including walls and ArUco markers, during navigation.

The system uses the GMapping SLAM algorithm to map the environment and determine the robot's position. Additionally, the MoveBase package is employed to enable autonomous navigation.

---

## Getting Started

### Prerequisites

1. **Clone the Repository:**
   ```bash
   git clone https://github.com/HodaMn/Exprob_Assignment2.git
   ```

2. **Install Required Packages:**
   ```bash
   git clone https://github.com/CarmineD8/aruco_ros.git
   ```
   Replace the `marker_publish.cpp` file in the `src` folder with the one provided in this repository.

3. **Clone Packages below:**
   ```bash
   git clone https://github.com/CarmineD8/SLAM_packages.git # Switch to noetic branch
   git clone https://github.com/KCL-Planning/ROSPlan.git
   ```
   Follow ROSPlan installation instructions for dependencies.
---

### Running the System

1. **Launch the Main System:**
   ```bash
   roslaunch assignment2_exprob main.launch
   ```

2. **Start the Planning Process:**
   ```bash
   roslaunch assignment2_exprob rosplan_simulation.launch
   ```

3. **Generate and Dispatch the Plan:**
   ```bash
   rosservice call /rosplan_problem_interface/problem_generation_server
   rosservice call /rosplan_planner_interface/planning_server
   rosservice call /rosplan_parsing_interface/parse_plan
   rosservice call /rosplan_plan_dispatcher/dispatch_plan
   ```

4. **View the Plan:**
   ```bash
   rostopic echo /rosplan_planner_interface/planner_output -p
   ```
---

## Features & Overview

### Waypoint Navigation
- The robot moves to predefined waypoints using MoveBase. 
- Waypoints and coordinates:
  - **WP0**: x = -7.0, y = 1.5
  - **WP1**: x = -3.0, y = -8.0
  - **WP2**: x = 6.0, y = 2.0
  - **WP3**: x = 7.0, y = -5.0
  - **WP4** (Initial Position): x = 0, y = 2.75

### Marker Detection
- Searches for markers.
- Updates the location with the lowest marker ID.
- Navigates based on detected markers.

### ROSPlan Integration
- Utilizes ROSPlan for automated task planning and execution.
- Actions are defined in PDDL and dispatched to the robot.
---

### Defined Actions in `exp_domain.pddl`

1. **go_to_waypoint**
   - Moves the robot from a specified waypoint to a target waypoint.

2. **detect**
   - Rotates the robot to locate an ArUco marker.
   - Updates the global variable `least_marker_id` if a smaller marker ID is detected.

3. **detect_all_markers**
   - Verifies that all waypoints have been visited.
   - Sends the robot to the location with the least marker ID.

---

### `exprob_assignment2`
- **Purpose:** Implements PDDL planning.
- **Contents:**
  - `pddl/`: Domain and problem files.
  - Launch files:
    - `main.launch`: Starts nodes and algorithms.
    - `rosplan_simulation.launch`: Begins planning simulation.
  - Configurations:
    - GMapping and MoveBase settings in `param/` folder.
    - Robot model files (`exp_robot4.xacro` and `exp_robot4.gazebo`) in `urdf/` folder.

### `my_rosplan_interface`
- **Purpose:** Handles action dispatch mechanisms.
- **Action Handlers:**
  - **go_to_waypoint:** Sends waypoint coordinates to MoveBase for navigation.
  - **detect:** Checks for markers and updates relevant parameters.
  - **detect_all_markers:** Retrieves the location of the marker with the least ID and sends the goal to the robot.

---

## Additional Notes
- Replace `marker_publish.cpp` as described to ensure compatibility.
- Review ROSPlan dependencies before starting the system.


