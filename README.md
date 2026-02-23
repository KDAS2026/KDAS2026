# KDAS2026


<div align="center">
  <img src="images/kdasmain.gif" alt="KDAS Main" width="800"/>
</div>



## Introduction

This project repository organizes the source code of an autonomous driving system developed by the Kookmin University K-DAS Team, targeting submission to the **American Control Conference (ACC) Self-Driving Car Student Competition 2026** Virtual Stage.

This repository contains the implementation code of the autonomous driving algorithms used in the ACC Virtual Stage demonstration video, along with explanations of the design intent and implementation approach of each autonomous driving module, and the overall system architecture based on the ACC Core Principles.

In addition, the repository is structured to allow users to verify the system execution procedure and the interconnection structure between modules.


---


## System Evolution and Objectives

The team has experience advancing to the Physical Stage by passing the Virtual Stage evaluation in the Self-Driving Car Student Competition 2025.

During the on-track driving phase, limitations were identified in terms of **driving stability** and **speed**, and these issues were set as the primary improvement objectives for the 2026 system design.

Accordingly, the existing autonomous driving system was advanced by improving the control architecture to enhance driving stability and by upgrading the decision-making and path planning logic to enable more aggressive speed planning.

Through these improvements, the team aims to achieve **meaningful results in the 2026 ACC Self-Driving Car Student Competition**.


---

### Key Architectural Focus (K-DAS 2026)

- **Primary Focus**: Decision & Planning Architecture

- **Core Improvement**:
  - Separation of **mission-level decision making** and **path generation**
  - Emphasis on **stable planning under dynamic driving scenarios**

- **Key Techniques**:
  - RRT-based global path planning
  - DQN-based decision making for driving strategy selection

- **Design Goal**:
  - Improved driving stability and speed through structured decision execution

---

## Whole Project

This autonomous driving system follows a hierarchical structure consisting of  
Perception, Localization, Decision & Planning, and Control stages,  
based on the system architecture of real autonomous vehicles.

- The perception modules extract necessary information from the driving environment  
- The localization module stably estimates the vehicle’s current position and orientation  
- The decision and planning module determines driving strategies and paths based on the extracted information  
- The control module generates steering and braking commands to allow the vehicle to follow the selected path  

The GIFs below summarize the actual implemented functions and their results for each module.  
By clicking each link, the detailed documentation and implementation results of the corresponding module can be reviewed.

<table>
  <tr>
    <td align="center">
      <img src="images/YOLOReadme1.gif" width="320"><br>
      <b>YOLO – Object Detection (1)</b><br>
      <a href="Perception/YOLO/">Go to folder</a>
    </td>
    <td align="center">
      <img src="images/YOLOReadme2.gif" width="320"><br>
      <b>YOLO – Object Detection (2)</b><br>
      <a href="Perception/YOLO/">Go to folder</a>
    </td>
  </tr>
  <tr>
    <td align="center">
      <img src="images/SCNNmodel.gif" width="320"><br>
      <b>SCNN – Lane Detection</b><br>
      <a href="Perception/SCNN/">Go to folder</a>
    </td>
    <td align="center">
      <img src="images/ControlMat.gif" width="320"><br>
      <b>Control – Steering and Braking Control</b><br>
      <a href="Control/">Go to folder</a>
    </td>
  </tr>
</table>

---

## Project Modules Overview

Below is an implementation-oriented summary of the major modules composing this autonomous driving system and the roles each module performs within the overall autonomous driving pipeline.

This system implements the core autonomous driving principles presented in the ACC Self-Driving Car Student Competition  
(Data Collection, Localization, Decision & Planning, Control) as concrete module-level implementations.

The modules are ordered according to the actual data flow  
(Perception → Localization → Planning → Control),  

and the detailed documentation and implementation of each module can be accessed via the title links.

---

### Core Principles Mapping

| ACC Core Principle | Implemented Modules |
|---|---|
| Data Collection | SCNN (Lane Detection), YOLO (Object Detection), LiDAR (Cartographer) |
| Interpretation & Decision Making | Decision & Planning (DQN-based decision) |
| Localization & Path Planning | Localization (Cartographer SLAM), Decision & Planning (Path generation) |
| Control Systems | Control (Pure Pursuit, Obstacle Avoidance), MATLAB/Simulink interface |

---

### [1) Localization](Localization/Cartographer/)


<p align="center">
  <img src="images/localization_rviz2.png" width="800"/>
</p

**Core Principle**: Localization & Path Planning  

**Role**  
The Localization module estimates the vehicle’s current position and orientation and provides  
a **reference coordinate frame and pose information** commonly used throughout the autonomous driving system.

Both the Planning and Control modules operate based on the Localization results, and  
all driving control actions such as path tracking and obstacle avoidance  
are executed with respect to the estimated pose.

**Implementation Overview**  
- 2D SLAM-based localization using LiDAR sensors  
- Map generation and localization pipeline using Cartographer  
- Coordinate frame management via ROS2 TF tree  

---

### [2) Decision & Planning](Decision&Planning/)

<p align="center">
  <img src="images/dqn1.png" width="800"/>
</p
  
  <p align="center">
  <img src="images/dqn2.png" width="800"/>
</p

**Core Principle**: Interpretation & Decision Making, Path Planning  

**Role**  
The Decision & Planning module receives perception and localization results as inputs,  
determines what action the vehicle should take in the current driving situation,  
and generates the corresponding driving path.

In this system, driving strategies are selected using a DQN-based decision-making logic  
based on interpreted environmental and vehicle state information,  
and waypoint-based paths are generated according to the selected strategy  
and passed to the control stage.

**Implementation Overview**
- Interpretation of driving situations using lane detection and object detection results  
- Driving strategy selection via DQN-based decision-making logic  
- Waypoint-based path generation according to the selected strategy  

---

### [3) Control](Control/)

**Core Principle**: Control Systems  

**Role**  
The Control module receives the driving path generated by the Planning stage and  
the vehicle state estimated through Localization,  
and serves as the final control stage that generates **steering and braking commands** for actual vehicle driving.

The Control module is designed to operate by distinguishing between  
normal driving conditions and obstacle-handling situations.

---

#### [3-1) Pure Pursuit Control](Control/Matlab&Simulink/Pure%20Pursuit/)

<table>
  <tr>
    <td align="center">
      <img src="images/control.gif"
           alt="Pure Pursuit Control Result"
           width="700"/><br/>
      <b>Pure Pursuit Steering Control Result</b>
    </td>
  </tr>
  <tr>
    <td align="center">
      <img src="images/ControlMat.gif"
           alt="Simulink Control Structure"
           width="600"/><br/>
      <b>Simulink-Based Steering Control Structure</b>
    </td>
  </tr>
</table>

**Role**  
The Pure Pursuit module computes steering angles for stable vehicle driving under normal conditions without obstacles,  
based on waypoint paths provided by the Planning stage and  
the vehicle’s current position and orientation received from the Localization module.

**Implementation Overview**  
- Target point tracking based on Look-Ahead Distance  
- Steering angle computation using the geometric relationship between the vehicle pose and the target point  
- Controller design in Simulink and integration as a ROS2 node  

---

#### [3-2) Obstacle Avoidance Control](Control/Obstacle%20Avoidance/)

**Role**  
The Obstacle Avoidance module performs lateral avoidance maneuvers and lane recovery actions to prevent collisions  
by considering the vehicle position obtained from Localization,  
the detected obstacle’s position and size from sensors,  
and the relative distance between the vehicle and the obstacle.

**Implementation Overview**  
- Avoidance triggering based on distance conditions to front obstacles  
- Estimation of obstacle position and size using LiDAR or Depth Camera  
- Minimum lateral offset calculation considering vehicle width and safety margin  
- Local path modification based on existing waypoints  
- Constant-curvature avoidance maneuver and symmetric lane recovery  

---

### [4) SCNN – Lane Detection](Perception/SCNN/)

<img src="images/SCNN1.gif" width="800"/>

**Core Principle**: Data Collection  

**Role**  
The SCNN module detects lane markings from camera input images and  
generates **lane structure and reference line information** that the vehicle can follow.

The lane detection results are used for drivable area inference and path generation,  
supporting path tracking in both the Planning and Control stages.

**Implementation Overview**  
- Lane detection pipeline based on the SCNN model  
- Input image preprocessing and output postprocessing  
- Lane information transmission via ROS2 messages  

---

### [5) YOLO – Object Detection](Perception/YOLO/)

<table>
  <tr>
    <td align="center"><img src="images/yolo9.gif" alt="Sim clip #1" width="420"/><br/><b>Video (Sim) #1</b></td>
    <td align="center"><img src="images/yolo10.gif" alt="Sim clip #2" width="420"/><br/><b>Video (Sim) #2</b></td>
  </tr>
</table>

**Core Principle**: Data Collection  

**Role**  
The YOLO module receives camera images as input and performs real-time detection of objects such as traffic signs and signals.

The detected object information is used for driving situation interpretation and decision-making,  
and serves as an input for behavior selection in the Planning and Control stages.

**Implementation Overview**  
- Application of YOLO-based object detection models  
- Transmission of real-time inference results via ROS2 topics  
- Interface design for integration with control logic  

---

## Repository Scope

This repository contains a fully integrated autonomous driving system  
including perception, localization, decision making, planning, and control modules.  
All components are designed to operate together within a ROS2-based framework.

---

## Virtual Stage Execution Guide (Docker + ROS2)

This section describes how to run the full autonomous driving system  
in the **Quanser Virtual QCar2 (QLabs) simulation environment** using Docker and ROS2.

The procedure below reproduces the complete pipeline:  
**Perception – Localization – Planning – Control**.

---

## System Overview

This demo uses **two Docker containers**:

- **Shell A — QLabs Simulation Container**
  - Spawns the Virtual QCar2 vehicle, sensors, map, and scenario
  - Must be started **before** the ROS2 containers

- **Shell B–I — Development Container (ROS2)**
  - Runs all ROS2 nodes:
    - Perception
    - Localization
    - Planning
    - Control
      
---

## Shell A — QLabs Virtual QCar2 Simulation

### 1) Run QLabs Simulation Container

```bash
cd /home/$USER/Documents/ACC_Development/docker/development_docker/quanser_dev_docker_files

sudo docker run --rm -it \
  --network host \
  --name virtual-qcar2 \
  quanser/virtual-qcar2 bash
```

---

### 2) Initialize Scenario (Inside Container)

```bash
cd /home/qcar2_scripts/python
python3 Base_Scenarios_Python/Setup_Real_Scenario.py
```

This script performs:
- Virtual QCar2 vehicle spawning
- Sensor and environment initialization
- Driving scenario setup

---

## Shell B – Development Container (ROS2)

### 1) Start Development Container

Open multiple terminals (Shell B–I).  
In **each terminal**, execute:

```bash
cd /home/$USER/Documents/ACC_Development/isaac_ros_common
./scripts/run_dev.sh /home/$USER/Documents/ACC_Development/Development

cd ros2
```

---

### 2) Build (Run Once Only)

In **one terminal only**:

```bash
colcon build
```

---

### 3) Source Environment (Every Terminal)

After the build, run in **all Shells (B–I)**:

```bash
source install/setup.bash
```

---

## ROS2 Launch Sequence (Recommended Order)

### Shell B — Base Virtual QCar2 Nodes

```bash
ros2 launch qcar2_nodes qcar2_virtual_launch.py
```

---

### Shell C — Localization (Cartographer)

```bash
ros2 launch qcar2_nodes localization0_launch.py
```

> **Wait until Cartographer localization is stable before proceeding.**

---

### Shell D — Utility Nodes

```bash
ros2 launch util util_launch.py
```

---

### Shell E — Path Planning

```bash
ros2 launch path_planning path_planning_launch.py
```

---

### Shell F — Control (MATLAB / Simulink Interface)

```bash
ros2 launch kdas_mat kdas_mat.launch.py
```

---

### Shell H — Object Detection (YOLO)

```bash
ros2 run yolo_detection yolo_node.py
```

---

### Shell I — Lane Detection

```bash
ros2 launch lane_detection lane_detection_launch.py
```

---

## Start Autonomous Driving (Run Last)

### Shell G — Enable Autonomous Ride

After **all nodes are running** and  
**Cartographer localization is confirmed**, execute:

```bash
ros2 topic pub /ride std_msgs/msg/Char "{data: 86}"
```

This command enables autonomous driving.

---

## Operational Notes

If the vehicle does not start moving, verify the following:

- All required nodes are running (Shell B–F, H–I)
- Cartographer localization is stable and not drifting
- Required ROS2 topics are properly published and subscribed
