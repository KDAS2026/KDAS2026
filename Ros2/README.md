# ROS2-Based Autonomous Driving System on QCar2 



This repository contains the source code for the **QCar2 Autonomous Driving System** built on **ROS2 Humble Hawksbill**. It seamlessly integrates **Perception, Planning, and Control** modules to implement real-time End-to-End autonomous driving.

<div align="center">
  <img src="../images/ROS1.gif" alt="ROS" width="700" />
  <br/>
  <b>ROS</b>
</div>

##  Project Overview

To overcome the structural limitations of legacy ROS1 (such as dependency on a central Master Node) and to ensure the reliable integration of various sensors and control modules, we adopted **ROS2 Humble** as our middleware.

- **Middleware:** ROS2 Humble (DDS-based distributed architecture)
- **Target Hardware:** Quanser QCar2 (NVIDIA Jetson based)
- **Key Features:**
    - Ensured data reliability and real-time performance via DDS.
    - QoS (Quality of Service) optimization based on topic importance.
    - Hybrid integration between MATLAB/Simulink controllers and ROS2 nodes.

---

##  Network Design & QoS Optimization

All nodes communicate through a structured topic tree. We established differentiated QoS policies specifically tailored to the characteristics of control feedback (real-time focus) and path data (integrity focus).

| Topic | Purpose | QoS Policy | Reasoning |
| :--- | :--- | :--- | :--- |
| `/current_location` | Vehicle Pose Feedback | **Keep Last (1) + Best Effort** | Minimize control loop latency (Prioritize latest Pose info) |
| `/waypoints` | Path Planning & Control | **Reliable + Keep All + Transient Local** | Prevent waypoint loss & maintain data for late-joining nodes |

---

##  Why ROS2? (Design Journey)

This project aimed not just for a simple assembly of technologies, but for the "optimal integration of heterogeneous systems." We went through the following trial-and-error process before selecting ROS2.

 * **CUDA Unified Approach:** We attempted to consolidate all computations into CUDA to fully utilize the NVIDIA Jetson. However, we confirmed that this was unsuitable as a real-time integration layer at the robotics level.
 * **Language Unification (C++ or Python):** Unifying to a single language alone could not fundamentally solve module synchronization and communication issues.
 * **MATLAB Integration:** We faced scalability constraints due to failures in converting custom deep learning models (e.g., SCNN) to ONNX and limited support for specific sensors (e.g., RPLiDAR).

**Conclusion:**
> **ROS2** was the optimal solution that leverages the strengths of both C++ (Control/Sensors) and Python (Perception/AI) while **meeting real-time requirements through DDS.**

---
