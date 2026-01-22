# Decision & Planning (RRT Global Path + DQN Mission Sequencing)
/
This document describes our **planning layer** for a closed-track autonomous taxi scenario.
We explicitly separate planning into two layers:

1. **Global Path Generation (Geometry / Drivable Trajectory)**  
   - Generates a continuous global trajectory (waypoints) inside the track corridor using **segment-wise RRT** and **post-processing smoothing**.

2. **Planning & Decision (Mission / Node Visit Order)**  
   - Computes the **optimal route sequence (node visit order)** for Pick-up / Drop-off tasks using a **Deep Q-Network (DQN)** on a directed graph representation.

This separation improves interpretability and integration:
- **RRT** answers: *“Where can the car physically drive (drivable trajectory)?”*  
- **DQN** answers: *“In what order should the car visit mission nodes (task sequencing)?”*  

The outputs are stored as **JSON waypoint segments** and can be loaded by downstream modules
(e.g., **MATLAB/Simulink controllers** or **ROS2 helper nodes**) for real-time execution.

---

## Part A. RRT-Based Global Path Generation

---

## 1. Overview

This project pre-generates a **global driving path** for an autonomous vehicle operating in a **closed-track environment** using the **RRT (Rapidly-exploring Random Tree)** algorithm.

Because the taxi scenario was conducted on a competition-grade closed track, pre-generation of the entire path was practically feasible.  
Although the conventional RRT algorithm is effective for path exploration, its **random sampling** nature introduces the following limitations:

- The result differs each time the algorithm is executed, and the generated path often lacks **continuity**.  
- Since no cost function is considered, RRT does not guarantee an **optimal (shortest or smoothest)** path.

To overcome these issues, the study defines **Free Space** for each track segment, repeatedly executes RRT in every segment, and then performs **post-processing (smoothing)** to construct a continuous and drivable global path.

In other words, while embracing the stochastic exploration characteristic of RRT, we exploit the repeatability of a closed-track environment to evaluate path quality quantitatively and **pre-construct the most stable and consistent trajectory** before real driving.

---

## 2. Theoretical Background of RRT

The core idea of RRT is to randomly sample a point in the search space, find the nearest node in the existing tree, and extend (steer) the tree from that node toward the sample by a small step Δq.  
Repeating this process rapidly expands the tree throughout the space until it reaches the goal region.

---

### (1) Random Sampling

$$
q_{\text{rand}} \sim U(Q)
$$

A random point \( q_{\text{rand}} \) is sampled within the search space \( Q \).

---

### (2) Nearest-Node Search

$$
q_{\text{near}} = \underset{q_i \in T}{\arg\min}\, \text{dist}(q_i, q_{\text{rand}})
$$

Among all nodes currently in the tree, the one with the smallest distance to \( q_{\text{rand}} \) is selected.  
The distance metric can be Euclidean or any metric suitable for the configuration space.

---

### (3) Tree Extension (Steer Function)

$$
q_{\text{new}} = q_{\text{near}} + 
\frac{(q_{\text{rand}} - q_{\text{near}})}{\|q_{\text{rand}} - q_{\text{near}}\|} \cdot \varepsilon
$$

From \( q_{\text{near}} \), a new point \( q_{\text{new}} \) is created by moving a step \( \varepsilon \) toward \( q_{\text{rand}} \).  
(In other words, one incremental step in the direction of \( q_{\text{rand}} \).)

---

### (4) Collision Check

$$
q_{\text{new}} \in C_{\text{free}}
$$

Verify that \( q_{\text{new}} \) lies within the free configuration space \( C_{\text{free}} \).  
If it violates the corridor boundary, discard \( q_{\text{new}} \).

> Note: The closed track has no internal physical obstacles, but **the drivable corridor boundaries**
> (left/right lane limits) function as constraints that must be respected.  
> Thus, collision checking is implemented as an **in-corridor validity test**.

**Segment-based collision test**

$$
\text{CollisionFree}(q_{\text{near}}, q_{\text{new}}) =
\begin{cases}
\text{True}, & \text{if } (1-s)q_{\text{near}} + s q_{\text{new}} \notin O,\; \forall s \in [0,1] \\
\text{False}, & \text{otherwise}
\end{cases}
$$

- \( O \): invalid region (outside the corridor)  
- If no point on the segment between \( q_{\text{near}} \) and \( q_{\text{new}} \) lies inside \( O \) for all \( s \in [0,1] \), the motion is valid.

---

### (5) Tree Expansion and Node Addition

$$
T = T \cup \{ (q_{\text{near}}, q_{\text{new}}) \}
$$

If valid, add the new node and edge to the tree.

---

## 3. Definition of Free Space and Problem Recognition

### 3.1 Structural Characteristics of the Track

The track has only an **outer boundary**, with no internal physical obstacles.  
While traffic lights and signs exist visually, the interior is essentially an **open empty space**, meaning that an **occupancy-grid representation** cannot accurately describe drivable regions.  
Therefore, a conventional occupancy-based environment model has fundamental limitations for applying RRT.

---

### 3.2 Centerline Extraction via Cartographer SLAM

To overcome this issue, the **centerline coordinates** obtained from **Cartographer-based SLAM** during vehicle driving were used.  
The centerline was laterally expanded by the lane width to generate left and right boundaries, modeling each track segment’s **Free Space** as a **polygonal corridor**.

---

### 3.3 Problems of Whole-Track Free-Space Generation

Initially, RRT was executed by forming a single free-space polygon using the entire track centerline.  
However, the presence of **intersections**, **roundabouts**, and **bi-directional lanes** caused **overlaps and tangling** among free-space polygons.

<p align="center"><img src="../images/rrt1.png" width="700"/></p>

As a result:
- The free-space definition became incomplete.  
- RRT expansion behaved abnormally or extended beyond the valid track boundary.  
- Numerous off-track trajectories were generated.

---

## 4. Segment-wise Free-Space Division and RRT Execution

To resolve these issues, the track was divided into multiple **segments**.  
Each segment consists of a continuous portion of the centerline with a fixed arc length,  
and an **independent free-space corridor** was modeled for each one.

---

### 4.1 Free-Space Modeling Procedure (Pseudo Code)

The following pseudo code summarizes how each segment’s Free Space was modeled using the waypoint data.  
The actual implementation loads waypoint JSON files and processes them sequentially.

Each segment includes the following steps:
1. Load and merge waypoints.  
2. Re-sample by arc length.  
3. Generate left/right lane boundaries using the lane width.  
4. Define the polygonal Free Space.

```python
INPUT: files = sorted(glob("waypoints_*.json"), by natural order)
PARAM: ds=0.05, lane_width=0.4
OUTPUT: segments_free_space = [FreeSpace_i], centerline, left/right boundaries

# 1) Load & stitch
segments = [ load_json_xy(f) for f in files ]
center_raw = stitch(segments)  # remove duplicated endpoints at joins

# 2) Resample (arc-length uniform)
centerline = resample_uniform(center_raw, step=ds)

# 3) Lane boundaries via normal offsets
tangent = normalize(gradient(centerline))
normal  = rotate90(tangent)  # [-t_y, t_x]
left_boundary  = centerline + 0.5 * lane_width * normal
right_boundary = centerline - 0.5 * lane_width * normal

# 4) Segmenting rules
breaks = []
for i from 1 to len(centerline)-2:
    kappa = curvature(centerline, i)
    dpsi  = heading(centerline, i) - heading(centerline, i-1)
    if kappa > κ_thresh OR abs(dpsi) > ψ_thresh OR arc(i) - arc(last_break) > L_max:
        breaks.append(i)

segments_idx = pairwise([0] + breaks + [len(centerline)-1])

# 5) Free space polygons
segments_free_space = []
for (s, e) in segments_idx:
    L = left_boundary[s:e+1]
    R = right_boundary[s:e+1]
    corridor = polygon( concatenate(L, reverse(R)) )
    segments_free_space.append(corridor)

