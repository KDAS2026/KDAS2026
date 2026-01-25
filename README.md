# KDAS2026


<div align="center">
  <img src="images/kdasmain.gif" alt="KDAS Main" width="800"/>
</div>


## Introduction

안녕하세요.  
본 저장소는 **국민대학교 자율주행 팀(K-DAS)**이 수행한 자율주행 시스템 연구 및 개발 결과를 정리한  
메인 프로젝트 레포지토리입니다.

본 프로젝트는 **Quanser Autonomous Car Competition (ACC)** 제출을 목표로 수행되었으며,  
실제 자율주행 차량의 작동 방식을 기준으로  
**인지(Perception) – 위치추정(Localization) – 판단·계획(Decision & Planning) – 제어(Control)** 단계를 분리하여  
시스템을 구성하였습니다.

각 단계는 명확한 입력과 출력을 갖도록 설계되었으며,  
각 모듈의 역할과 데이터 흐름을 중심으로  
전체 자율주행 시스템의 구조를 정리하였습니다.

---

## Whole Project

본 프로젝트에서 저희 팀은 자율주행 시스템을  
실제 자율주행 차량의 시스템 구조에 맞추어 단계적으로 구현하였습니다.

- 인지 모듈은 주행 환경으로부터 필요한 정보를 추출하고  
- 위치추정 모듈은 차량의 현재 위치와 자세를 안정적으로 추정하며  
- 판단·계획 모듈은 해당 정보를 바탕으로 주행 전략과 경로를 결정하고  
- 제어 모듈은 결정된 경로를 따라 차량이 실제로 주행할 수 있도록 조향·제동 명령을 생성합니다.

아래 GIF들은 각 모듈에서 실제로 구현한 기능과 그 결과를 요약적으로 보여줍니다.  
각 항목의 링크를 클릭하면, 해당 모듈의 상세 문서와 구현 결과를 확인할 수 있습니다.

<table>
  <tr>
    <td align="center">
      <img src="images/YOLOReadme1.gif" width="320"><br>
      <b>YOLO – 객체 인식 (1)</b><br>
      <a href="Perception/YOLO/">폴더 바로가기</a>
    </td>
    <td align="center">
      <img src="images/YOLOReadme2.gif" width="320"><br>
      <b>YOLO – 객체 인식 (2)</b><br>
      <a href="Perception/YOLO/">폴더 바로가기</a>
    </td>
  </tr>
  <tr>
    <td align="center">
      <img src="images/SCNNmodel.gif" width="320"><br>
      <b>SCNN – 차선 인식</b><br>
      <a href="Perception/SCNN/">폴더 바로가기</a>
    </td>
    <td align="center">
      <img src="images/ControlMat.gif" width="320"><br>
      <b>Control – 조향 및 제동 제어</b><br>
      <a href="Control/">폴더 바로가기</a>
    </td>
  </tr>
</table>

---

## Project Modules Overview

아래는 본 자율주행 시스템을 구성하는 주요 모듈과,  
각 모듈이 전체 파이프라인 내에서 수행하는 역할에 대한 설명입니다.  
모듈 순서는 실제 데이터 흐름을 기준으로 정렬되어 있습니다.
각 모듈의 파란색 제목을 클릭하면 세부 내용을 확인 할 수 았습니다.

---

### [1) Localization](Localization/Cartographer/)


<p align="center">
  <img src="images/localization_rviz2.png" width="800"/>
</p
  
**역할**  
Localization 모듈은 차량의 현재 위치와 자세를 추정하여,  
자율주행 시스템 전반에서 공통으로 사용되는 **기준 좌표계 및 위치 정보**를 제공합니다.

Planning 및 Control 모듈은 모두 Localization 결과를 기반으로 동작하며,  
경로 추종, 장애물 회피 등 모든 주행 제어 동작은  
해당 위치 추정 결과를 기준으로 수행됩니다.

**구현 개요**  
- LiDAR 센서를 활용한 2D SLAM 기반 위치 추정  
- Cartographer를 이용한 지도 생성 및 위치 추정 파이프라인 구성  
- ROS2 TF 트리를 통한 좌표계 관리  
- 시뮬레이션 환경에서의 위치 추정 안정성 검증  

---

### [2) Decision & Planning](Planning/)

<p align="center">
  <img src="images/dqn1.png" width="800"/>
</p
  
  <p align="center">
  <img src="images/dqn2.png" width="800"/>
</p

**역할**  
Decision & Planning 모듈은 인지 및 위치추정 결과를 입력으로 받아,  
차량이 현재 주행 상황에서 어떤 행동을 수행해야 하는지를 결정하고  
그에 따른 주행 경로를 생성하는 역할을 담당합니다.

본 시스템에서는 주행 환경 해석 결과와 차량 상태 정보를 바탕으로  
DQN 기반 의사결정 로직을 통해 주행 전략을 선택하며,  
선택된 전략에 따라 waypoint 기반 경로를 생성하여  
제어 단계로 전달합니다.

**구현 개요**
- 차선 인식 및 객체 인식 결과를 입력으로 주행 상황 해석  
- DQN 기반 의사결정 로직을 통한 주행 전략 선택  
- 선택된 전략에 따른 waypoint 기반 경로 생성  

---

### [3) Control](Control/)

**역할**  
Control 모듈은 Planning 단계에서 생성된 주행 경로와  
Localization을 통해 추정된 차량 상태를 입력으로 받아,  
차량이 실제로 주행할 수 있도록 **조향 및 제동 명령을 생성하는 최종 제어 단계**입니다.

본 시스템의 Control 모듈은  
정상 주행과 장애물 대응 상황을 구분하여 동작하도록 구성되어 있습니다.

---

#### [3-1) Pure Pursuit Control](Control/Matlab&Simulink/Pure%20Pursuit/)

**역할**  
Pure Pursuit 모듈은 장애물이 없는 정상 주행 상황에서,  
Planning 단계에서 전달된 waypoint 경로와  
Localization 모듈로부터 전달받은 차량의 현재 위치 및 자세 정보를 기반으로  
차량이 안정적으로 주행하도록 조향각을 계산하는 경로 추종 제어 모듈입니다.

**구현 개요**  
- Look-Ahead Distance 기반 목표점 추종 방식 적용  
- 차량의 현재 위치와 목표점 간의 기하학적 관계를 이용한 조향각 계산  
- Simulink 기반 제어기 설계 및 ROS2 노드 형태로 연동  

---

#### [3-2) Obstacle Avoidance Control](Control/Obstacle%20Avoidance/)

**역할**  
Obstacle Avoidance 모듈은 주행 경로 상에 장애물이 존재하는 경우,  
Localization 결과를 통해 파악된 차량 위치와  
센서를 통해 인식된 장애물의 위치, 크기, 그리고 차량과 장애물 간의 상대 거리를 고려하여  
충돌을 방지하기 위한 횡방향 회피 기동과 차선 복귀 동작을 수행하는 제어 모듈입니다.

**구현 개요**  
- 전방 장애물과의 거리 조건을 기반으로 회피 기동 필요 여부 판단  
- LiDAR 또는 Depth Camera를 이용한 장애물 위치 및 크기 추정  
- 차량 폭과 안전 여유 거리를 고려한 최소 횡방향 회피 거리 계산  
- 기존 waypoint를 기반으로 한 국소적 경로 수정 방식 적용  
- 일정 곡률을 이용한 회피 기동 수행 및 동일 곡률 기반 차선 복귀  

---

### [4) SCNN – Lane Detection](Perception/SCNN/)

**역할**  
SCNN 모듈은 카메라 입력 영상을 기반으로 차선을 인식하고,  
차량이 추종할 수 있는 **차선 구조 및 기준선 정보**를 생성하는 인지 모듈입니다.

차선 인식 결과는 주행 가능 영역 추론 및 경로 생성에 활용되며,  
Planning 단계와 제어 단계의 경로 추종을 지원합니다.

**구현 개요**  
- SCNN 모델 기반 차선 인식 파이프라인 구성  
- 입력 영상 전처리 및 출력 후처리 설계  
- ROS2 메시지 형태로 차선 정보 전달  

---

### [5) YOLO – Object Detection](Perception/YOLO/)

**역할**  
YOLO 모듈은 카메라 영상을 입력으로 받아  
교통 표지판 및 신호와 같은 객체를 실시간으로 인식하는 인지 모듈입니다.

인식된 객체 정보는 주행 상황 해석 및 의사결정에 활용되며,  
Planning 및 Control 단계에서의 행동 선택에 필요한 입력으로 사용됩니다.

**구현 개요**  
- YOLO 기반 객체 인식 모델 적용  
- 실시간 추론 결과를 ROS2 토픽으로 송신  
- 제어 로직과 연동 가능한 인터페이스 설계  

---

## Core Principles of Self-Driving

본 시스템에서는 자율주행 차량이 실제로 동작하기 위해 필요한  
데이터 흐름과 의사결정 과정을 기준으로 전체 파이프라인을 구성하였습니다.

아래에서는 센서 입력부터 제어 출력까지,  
각 단계에서 데이터가 어떻게 처리되고  
어떤 모듈을 거쳐 주행 동작으로 이어지는지를 정리합니다.

---

### 1) Data Collection

본 시스템은 주행 환경과 차량 상태를 인식하기 위해  
카메라 및 LiDAR 센서를 기반으로 데이터를 수집합니다.

- SCNN 기반 차선 인식 모듈을 통해 차선 영상 데이터를 입력으로 사용  
- YOLO 기반 객체 인식 모듈을 통해 교통 표지판 및 신호 객체 인식 수행  
- Cartographer SLAM을 통해 주변 환경의 거리 정보 및 지도 정보 획득  

수집된 센서 데이터는 이후 단계에서  
주행 판단과 제어에 활용될 수 있는 형태로 전달됩니다.

---

### 2) Interpretation & Decision Making

수집된 데이터는 주행 환경과 차량 상태를 해석하고,  
차량이 어떤 행동을 수행해야 하는지를 결정하는 데 사용됩니다.

- 차선 구조 및 주행 가능 영역 추론  
- 인식된 객체의 의미 해석  
- DQN 기반 의사결정 로직을 통한 주행 전략 선택  

---

### 3) Localization & Path Planning

차량은 지도 상에서 자신의 위치를 이해하고,  
선택된 주행 전략에 따라 경로를 생성 및 수정합니다.

- LiDAR 기반 Cartographer SLAM을 이용한 위치 추정  
- waypoint 기반 경로 생성 및 갱신  

---

### 4) Control Systems

계획된 경로와 판단 결과를 바탕으로,  
차량은 실제 조향 및 제동 동작을 수행합니다.

- Pure Pursuit 기반 경로 추종 제어  
- 장애물 상황에서의 회피 기동 및 차선 복귀 제어  

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

> **Important**  
> The QLabs container must be running first.  
> Containers communicate using `--network host`.

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

## Shell B–I — Development Container (ROS2)

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
