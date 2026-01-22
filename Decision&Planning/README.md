# Decision & Planning (RRT + DQN)

This module describes our **high-level decision-making and global planning framework**
for an autonomous vehicle operating in a **closed-loop track environment**.

To satisfy both **geometric feasibility** and **mission-level decision optimality**,
we adopt a **hybrid hierarchical planning strategy**:

1. **RRT-based Global Path Generation**
   - Constructs **continuous, drivable global trajectories** inside lane-constrained free space.
   - Focuses on geometric feasibility and path smoothness.

2. **DQN-based Decision & Route Sequencing**
   - Learns the **optimal visiting order of mission nodes**
     (e.g., Pickup → Drop-off → Hub).
   - Operates on a **directed graph abstraction** of the roadmap.

> In summary:  
> **RRT answers _“Where can the car physically drive?”_**  
> **DQN answers _“Which mission location should be visited next?”_**

---

## 1. System-Level Architecture

Roadmap / SLAM
↓
Centerline & Lane Boundaries
↓
[ RRT Global Path Generation ]
↓
Smoothed Global Path (Trajectory Library)
↓
[ DQN Decision Module ]
↓
Selected Path Segments / Node Sequence
↓
Control (Pure Pursuit)


- **RRT** produces a library of feasible path segments.
- **DQN** selects and orders these segments based on mission objectives.
- The resulting path is executed by downstream control modules.

---

## 2. Global Path Generation using RRT

### 2.1 Motivation

Although RRT efficiently explores free space, its stochastic nature causes:
- Non-deterministic paths
- Sharp curvature and discontinuities
- Lack of optimality guarantees

Because the competition environment is a **closed and repeatable track**,  
we pre-generate and validate global paths offline.

---

### 2.2 RRT Theoretical Background

**Random Sampling**
\[
q_{\text{rand}} \sim U(Q)
\]

**Nearest Node Selection**
\[
q_{\text{near}} = \arg\min_{q_i \in T} \text{dist}(q_i, q_{\text{rand}})
\]

**Tree Extension**
\[
q_{\text{new}} = q_{\text{near}} +
\frac{q_{\text{rand}} - q_{\text{near}}}
{\|q_{\text{rand}} - q_{\text{near}}\|} \cdot \varepsilon
\]

**Collision / Corridor Check**
\[
q_{\text{new}} \in C_{\text{free}}
\]

Only collision-free edges inside the corridor are added to the tree.

---

## 3. Free-Space Modeling via SLAM Data

### 3.1 Track Characteristics

- Single outer boundary
- No internal static obstacles
- Visual traffic objects (lights, signs) do not define physical occupancy

This makes **occupancy-grid-based RRT unsuitable**.

---

### 3.2 Corridor Construction from Cartographer SLAM

- Extract centerline from Cartographer SLAM
- Offset laterally by lane width
- Generate **left/right lane boundaries**
- Define **polygonal free-space corridors**

---

### 3.3 Segment-wise Free Space Division

Using a single free-space polygon for the entire track caused:
- Corridor overlaps at intersections
- Invalid RRT expansion
- Off-track trajectories

**Solution:** Divide the track into multiple segments.

Each segment:
- Has its own corridor
- Runs RRT independently
- Produces a local feasible path

---

### 3.4 Segment-wise RRT Execution (Pseudo Code)

```text
Algorithm: RRT in Corridor C
1. Initialize tree T with start node q_start
2. for i = 1..N_max:
     q_rand ← SampleFree(C)
     q_near ← Nearest(T, q_rand)
     q_new  ← Steer(q_near, q_rand, η)
     if EdgeInCorridor(q_near, q_new, C):
         AddNode(T, q_new)
         AddEdge(T, q_near → q_new)
         if ||q_new - q_goal|| < r_goal:
             return BacktrackPath(T)
3. return Failure

```

## 4. Path Post-Processing and Smoothing

### 4.1 Motivation

Raw RRT-generated paths exhibit several drawbacks when directly applied
to autonomous vehicle driving:

- Discontinuous waypoints
- Sharp turns with high curvature
- Lack of smoothness required for stable lateral control

Therefore, post-processing is required before the path can be executed
by a real vehicle or simulation controller.

---

### 4.2 Elastic-Band-Based Path Smoothing

To address these issues, we apply an **Elastic-Band-based smoothing method**.
The generated path is treated as a deformable band that iteratively minimizes
a weighted energy function composed of:

- **Path length (tension term)**
- **Curvature smoothness**
- **Boundary clearance**
- **Attraction to centerline**

Each waypoint is adjusted to reduce total energy while remaining inside
the free-space corridor.

This results in:

- Continuous curvature
- Reduced steering oscillations
- Improved drivability under vehicle kinematic constraints

---

### 4.3 Final Global Path Construction

After smoothing:

- Each segment-level path becomes dynamically feasible
- All segments are concatenated in sequence
- A complete **global trajectory** covering the entire track is obtained

The final output is stored as **JSON waypoint files**, which can be
directly loaded into:

- MATLAB/Simulink controllers
- ROS2 trajectory-following nodes

---

## 5. Decision and Planning using DQN

### 5.1 Problem Definition

While RRT generates **geometrically feasible trajectories**, it does not
decide **which mission objective should be visited next**.

The **DQN-based decision module** addresses this by learning an optimal
policy for mission-level planning.

Its responsibilities include:

- Selecting the next Pickup / Drop-off location
- Avoiding cyclic or inefficient routes
- Minimizing total mission cost

---

### 5.2 Graph-Based Environment Modeling

The roadmap is abstracted as a **directed graph**:

- Nodes represent intersections or mission points
- Edges represent valid transitions between nodes

The agent:

- Starts at node 0
- Sequentially visits assigned mission nodes
- Terminates at the final hub

---

### 5.3 Optimal Q-Function

The optimal action-value function is defined as:

\[
Q^*(s, a) = \max_\pi \mathbb{E}
\left[ \sum_{t=0}^{\infty} \gamma^t r_t \mid s_0 = s, a_0 = a \right]
\]

---

### 5.4 Q-Learning Update Rule

The Q-value update rule is given by:

\[
Q(s, a) \leftarrow Q(s, a) +
\alpha \left[ r + \gamma \max_{a'} Q(s', a') - Q(s, a) \right]
\]

---

### 5.5 Deep Q-Network Approximation

To handle large state spaces, a neural network is used to approximate
the Q-function:

\[
Q(s, a; \theta) \approx Q^*(s, a)
\]

Stability is ensured through:

- Experience replay
- Target network updates
- ε-greedy exploration strategy

---

### 5.6 Reward Design

| Condition | Reward |
|---------|--------|
| Reach sub-goal | +100 |
| Enter loop | −1000 |
| Revisit node | −10 × visit count |
| General movement | −1 |
| Branch node | +5 |

---

### 5.7 Integration with RRT Paths

- DQN outputs an **ordered sequence of nodes / segments**
- Each segment corresponds to a **pre-generated RRT path**
- Selected paths are concatenated into a mission trajectory

This decoupling ensures:

- Deterministic vehicle behavior
- Stable learning
- Real-time execution feasibility

---

## 6. Conclusion

By combining:

- **Sampling-based global planning (RRT)**
- **Learning-based mission decision (DQN)**

we construct a **hierarchical planning framework** that satisfies both
geometric feasibility and decision optimality.

This architecture:

- Aligns with real-world autonomous driving systems
- Enables modular testing and replacement
- Meets the evaluation criteria of the Virtual Stage competition


