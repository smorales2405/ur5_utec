# ur5_utec

ROS 2 workspace for pick-and-place motion planning with a UR5e robot and Robotiq 2F-85 gripper in Gazebo Fortress. This repository bundles all four packages required to run the full simulation and multi-objective trajectory optimisation (CU3) without any additional workspace dependencies.

| Package | Role |
|---|---|
| `ur5_pick_place` | Main node, Cartesian trajectory generation, simulation assets (URDF, world, meshes) |
| `ur5_kinematics` | QP-based inverse kinematics library (Pinocchio + OsqpEigen) |
| `robotiq_description` | URDF and meshes for the Robotiq 2F-85 gripper (trimmed to 2F-85 only) |
| `ur5_trajectory_optimization` | Offline multi-objective optimizer (CU3): NSGA-II + ε-constraint, Python |

---

## Table of Contents

1. [Requirements](#1-requirements)
2. [Workspace Setup](#2-workspace-setup)
3. [Building](#3-building)
4. [Usage](#4-usage)
5. [CU3 — Multi-objective Trajectory Optimisation](#5-cu3--multi-objective-trajectory-optimisation)
6. [Package Structure](#6-package-structure)
7. [Parameters](#7-parameters)
8. [Output Data](#8-output-data)

---

## 1. Requirements

### Assumed already installed
- ROS 2 Humble
- Gazebo Fortress
- Pinocchio (C++ + Python bindings)
- Eigen3

### ROS 2 system packages

```bash
sudo apt update && sudo apt install -y \
  ros-humble-ur-description \
  ros-humble-ros-gz \
  ros-humble-gz-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui
```

### OSQP v0.6.3 (from source)

```bash
git clone --recursive https://github.com/osqp/osqp.git
cd osqp
git checkout v0.6.3
git submodule update --init --recursive
mkdir build && cd build
cmake .. && make
sudo make install
```

### OsqpEigen v0.8.1 (from source)

```bash
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen
git checkout v0.8.1
mkdir build && cd build
cmake .. && make
sudo make install
```

```bash
sudo ldconfig
```

### Python packages for `ur5_trajectory_optimization`

```bash
pip3 install "numpy<2"   # must stay < 2 for Pinocchio Python binding compatibility
pip3 install pymoo scipy matplotlib pyyaml
```

> **NumPy version note:** The Pinocchio Python bindings in this installation are compiled against NumPy 1.x. Installing `numpy>=2` (e.g. as a side-effect of `pip install pymoo`) will cause a segfault. Always pin `numpy<2` after installing `pymoo`.

---

## 2. Workspace Setup

```bash
mkdir -p ~/ur5_ws/src
cd ~/ur5_ws/src
```

Clone this repository into the workspace source:

```bash
git clone https://github.com/smorales2405/ur5_utec.git
```

> **Note:** `ur_description` is installed via apt (`ros-humble-ur-description`) and does **not** need to be cloned. This single repository replaces the three separate clones (`ur5_pick_place`, `ur5_simulation`, `ros2_robotiq_gripper`) described in the standalone `ur5_pick_place` README.

---

## 3. Building

```bash
cd ~/ur5_ws
colcon build
source install/setup.bash
```

> On slow machines you can limit parallel jobs to avoid memory issues:
> ```bash
> colcon build --parallel-workers 2
> ```

---

## 4. Usage

### 4.1 Simulation only (Gazebo + robot + controllers)

Launches Gazebo Fortress with the UTEC lab world, spawns the UR5e + Robotiq 2F-85 robot, and starts ros2_control with the `joint_trajectory_controller`.

```bash
ros2 launch ur5_pick_place ur5_robotiq_gz.launch.py
```

| Argument | Default | Description |
|---|---|---|
| `ur_type` | `ur5e` | UR robot model |
| `safety_limits` | `true` | Enable joint safety limits |
| `start_joint_controller` | `true` | Activate `joint_trajectory_controller` on start |
| `initial_joint_controller` | `joint_trajectory_controller` | Controller to spawn |
| `gazebo_gui` | `true` | Show Gazebo GUI |

### 4.2 Full pipeline (simulation + pick-and-place)

Launches the simulation and automatically starts the pick-and-place node after a configurable delay.

```bash
ros2 launch ur5_pick_place simulation.launch.py
```

| Argument | Default | Description |
|---|---|---|
| `method` | `clamped_spline` | Trajectory method: `clamped_spline` or `piecewise_linear` |
| `ur_type` | `ur5e` | UR robot model |
| `node_start_delay` | `20.0` | Seconds to wait before starting the pick-and-place node |

```bash
# Example with piecewise linear and 30 s delay:
ros2 launch ur5_pick_place simulation.launch.py method:=piecewise_linear node_start_delay:=30.0
```

### 4.3 Pick-and-place node only (simulation already running)

```bash
ros2 launch ur5_pick_place pick_place.launch.py
```

| Argument | Default | Description |
|---|---|---|
| `method` | `clamped_spline` | Trajectory method: `clamped_spline` or `piecewise_linear` |
| `extra_params_file` | `""` | Optional extra YAML loaded after `pick_place_params.yaml` (later keys win) |

### 4.4 Pick-and-place with optimised via-point (CU3)

After running the optimisation (see [Section 5](#5-cu3--multi-objective-trajectory-optimisation)), pass the selected solution via `extra_params_file`:

```bash
# Terminal 1 — simulation
ros2 launch ur5_pick_place ur5_robotiq_gz.launch.py

# Terminal 2 — node with optimised via-point
ros2 launch ur5_pick_place pick_place.launch.py \
  extra_params_file:=$HOME/ur5_ws/src/ur5_utec/ur5_trajectory_optimization/results/selected_solution.yaml
```

`extra_params_file` is loaded after `pick_place_params.yaml`; only `point_O` is overridden, all other parameters remain unchanged.

### 4.5 RViz inspection (without Gazebo)

Launches RViz with the full robot model and visualisations of the five trajectory waypoints, the scene geometry, and the obstacle AABB — no simulation required.

```bash
ros2 launch ur5_pick_place rviz_inspection.launch.py
```

| Argument | Default | Description |
|---|---|---|
| `ur_type` | `ur5e` | UR robot model |
| `tf_prefix` | `""` | TF prefix |

The launch starts four nodes:

| Node | Description |
|---|---|
| `robot_state_publisher` | Publishes TF tree from URDF |
| `joint_state_publisher_gui` | Slider GUI to manually pose the robot |
| `waypoint_markers_node.py` | Publishes waypoint, scene, and obstacle markers |
| `rviz2` | RViz2 with pre-configured `inspection.rviz` display |

**Published topics:**

| Topic | Type | Content |
|---|---|---|
| `/waypoint_markers` | `MarkerArray` | Coloured spheres + text labels at A, B, O, C, D |
| `/scene_markers` | `MarkerArray` | surgery_table and ur5_base meshes in `base_link` frame |
| `/obstacle_markers` | `MarkerArray` | Obstacle AABB: solid box, wireframe edges, and dimension label |

The `waypoint_markers_node` reloads `pick_place_params.yaml` every 2 seconds automatically, so waypoint and obstacle geometry changes take effect after the next `colcon build` without restarting the node.

---

## 5. CU3 — Multi-objective Trajectory Optimisation

### Overview

CU3 treats the via-point position **p_O = [x, y, z]** as a 3-D decision variable and minimises three conflicting objectives simultaneously over the **B → O → C** arc:

| Objective | Formula | Meaning |
|---|---|---|
| f₁ | ∫₀ᵀ Σᵢ τᵢ(t)² dt | Joint effort [N²·m²·s] — minimise actuator load |
| f₂ | ∫₀ᵀ ‖ṗ(t)‖ dt | TCP arc length [m] — minimise path length |
| f₃ | −d_min | Negative obstacle clearance [m] — maximise safety margin |

Subject to the hard constraint **d_min ≥ r_grip** (gripper does not penetrate the obstacle AABB).

The pipeline runs **offline** (no Gazebo required) using Pinocchio's RNEA for dynamics and a damped least-squares IK solver.

### Coordinate frame

All Cartesian positions in `pick_place_params.yaml` and in the optimiser are expressed in the **`base_link` frame**, where z = 0 is the robot base mounting surface (= Pinocchio world frame).

In Gazebo Fortress the robot base link is spawned at z = 0.63 m above the ground plane:

```
z_Gazebo = z_base_link + 0.63
```

### Gripper TCP frame

The robot model chains: `tool0` → 11 mm UR-to-Robotiq adapter → Robotiq 2F-85 → `gripper_tcp`. The combined Z-offset from `tool0` to `gripper_tcp` is:

```
11 mm  (ur_to_robotiq_adapter)
+ 130 mm  (Robotiq 2F-85 to tcp)
= 141 mm  →  kGripperTcpOffsetZ = 0.141 m
```

All five waypoints (A, B, O, C, D) specify the desired position of `gripper_tcp` in the `base_link` frame. The IK solver (`ur5_kinematics`) registers `gripper_tcp` as a fixed operational frame in the Pinocchio model and solves directly for it — no post-hoc offset correction is needed in the pick-and-place node.

### Waypoints

The full pick-and-place motion follows five waypoints in sequence:

| Point | Coordinates [m] (base_link) | Role |
|---|---|---|
| **A** | [0.00, 0.70, 0.15] | Start / retraction (pick side) |
| **B** | [0.65, 0.55, 0.20] | Pick / approach |
| **O** | *optimised* | Via-point over the obstacle ← **CU3 decision variable** |
| **C** | [0.65, −0.55, 0.20] | Place / approach |
| **D** | [0.00, −0.70, 0.15] | End / retraction (place side) |

Trajectory segments:
- **A → B**: linear segment, `pre_post_duration` seconds
- **B → O → C**: clamped cubic spline through O, `total_duration` seconds (the arc optimised by CU3)
- **C → D**: linear segment, `pre_post_duration` seconds

### Obstacle definition

The obstacle is modelled as an axis-aligned bounding box (AABB) in the `base_link` frame:

| Parameter | Value | Description |
|---|---|---|
| `obstacle_center` | [0.85, 0.00, 0.10] m | Centre of the box in base_link |
| `obstacle_half_extents` | [0.20, 0.30, 0.10] m | Half-dimensions along x, y, z |
| `obstacle_r_grip` | 0.05 m | Gripper enveloping radius (constraint: d_min ≥ r_grip) |
| `obstacle_delta_safe` | 0.05 m | Additional safety margin |

This gives the physical box:

```
x ∈ [0.65, 1.05]   y ∈ [−0.30, 0.30]   z ∈ [0.00, 0.20]  (base_link)
```

The **top surface (z = 0.20 m)** coincides with the pick/place height (z_B = z_C = 0.20 m), so the optimizer must always route the arc **above** this plane.

In Gazebo the same geometry is represented as a primitive box model named `obstacle`:
- Pose: (0.85, 0.00, 0.73) in Gazebo world frame (z + 0.63)
- Size: 0.40 × 0.60 × 0.20 m

The optimizer (`objective_evaluators.py`), the C++ pick-and-place node, and the RViz marker node all read the same `obstacle_center` / `obstacle_half_extents` parameters from `pick_place_params.yaml`, ensuring a single source of truth.

### Search domain

| Variable | Range | Constraint |
|---|---|---|
| x_O | [0.50, 0.95] m | ≥ 0.50 prevents excessive arm retraction (joint 1 velocity peak → JTC abort) |
| y_O | [−0.25, 0.25] m | Keeps via-point within the obstacle's y footprint |
| z_O | [0.25, 0.60] m | ≥ 0.25 ensures arc stays above obstacle top (z = 0.20 m) |

### Two-stage optimisation

**Stage 1 — NSGA-II** (pymoo): approximates the Pareto front with `pop_size=60`, `n_gen=120`.

**Stage 2 — ε-constraint** (scipy SLSQP): sweeps 25 ε-levels across the f₁ range to refine compromise solutions along the frontier.

A **knee point** is selected automatically by minimum normalised distance to the utopia point.

### Running the optimisation

```bash
source ~/ur5_ws/install/setup.bash

# Run both stages (~10–20 min depending on hardware)
ros2 run ur5_trajectory_optimization run_optimization
```

Results are written to `ur5_trajectory_optimization/results/`:

| File | Description |
|---|---|
| `pareto_nsga2.csv` | Non-dominated front from NSGA-II (columns: via_x/y/z, f1, f2, f3, g1) |
| `pareto_epsilon.csv` | Compromise solutions from ε-constraint sweep (columns: via_x/y/z, f1, f2, f3) |
| `selected_solution.yaml` | ROS 2 param override with the knee-point `point_O` |

### Selecting and exporting a solution

`export_trajectory` uses Python argparse — arguments go **directly after the command**, without `--ros-args`.

```bash
# Knee point from ε-constraint (default — recommended):
ros2 run ur5_trajectory_optimization export_trajectory

# Knee point from NSGA-II:
ros2 run ur5_trajectory_optimization export_trajectory --source nsga2

# Specific index from ε-constraint:
ros2 run ur5_trajectory_optimization export_trajectory --index 3

# Specific index from NSGA-II:
ros2 run ur5_trajectory_optimization export_trajectory --source nsga2 --index 7
```

> **Why ε-constraint is the default:** In practice, NSGA-II solutions tend to cluster near the lower bound of the x search domain (`via_x_min`), producing via-points that force the arm to retract ~0.25 m from B before swinging over the obstacle. This causes a sharp peak in the shoulder joint (joint 1) velocity that exceeds the JTC position-tracking tolerance (0.200 rad), triggering a controller abort. The ε-constraint sweep produces a more structured Pareto front with higher x_O values, resulting in smoother, more executable trajectories.

### Visualising the Pareto front

```bash
# Interactive 3-panel plot (3D scatter, 2D projections, parallel coordinates):
python3 ~/ur5_ws/src/ur5_utec/ur5_trajectory_optimization/scripts/plot_pareto.py

# Save PNG figures to results/:
python3 ~/ur5_ws/src/ur5_utec/ur5_trajectory_optimization/scripts/plot_pareto.py --save
```

---

## 6. Package Structure

```
ur5_utec/
│
├── ur5_pick_place/                         # Main package
│   ├── config/
│   │   ├── pick_place_params.yaml          # All pick-and-place parameters + obstacle geometry (CU3)
│   │   ├── inspection.rviz                 # Pre-configured RViz2 layout for rviz_inspection.launch.py
│   │   ├── joint_state_initial.yaml        # Initial joint positions for joint_state_publisher_gui
│   │   └── ur5_robotiq_controllers.yaml    # ros2_control controller configuration
│   ├── data/                               # CSV trajectory logs (auto-generated, git-ignored)
│   ├── include/ur5_pick_place/
│   │   ├── ik_wrapper.hpp                  # IK wrapper: registers gripper_tcp frame (0.141 m offset)
│   │   └── trajectory_generator.hpp        # Cartesian trajectory generator interface
│   ├── launch/
│   │   ├── ur5_robotiq_gz.launch.py        # Simulation launch (Gazebo + controllers)
│   │   ├── simulation.launch.py            # Combined launch (simulation + pick-and-place)
│   │   ├── pick_place.launch.py            # Pick-and-place node only (supports extra_params_file)
│   │   └── rviz_inspection.launch.py       # RViz inspection without Gazebo (waypoints + obstacle AABB)
│   ├── meshes/                             # Gazebo SDF models for the UTEC lab
│   │   ├── surgery_table/                  # Work surface (mesa de trabajo del UR5e)
│   │   └── ur5_base/                       # Robot pedestal
│   ├── scripts/
│   │   └── waypoint_markers_node.py        # Publishes /waypoint_markers, /scene_markers, /obstacle_markers
│   ├── src/
│   │   ├── pick_place_node.cpp             # Main ROS 2 node (RNEA torque logging, extended CSV)
│   │   ├── ik_wrapper.cpp                  # TCP offset correction for gripper_tcp
│   │   ├── trajectory_generator.cpp        # Clamped spline (with analytic jerk) and linear
│   │   ├── plots_trajectory.m              # MATLAB: plot one trajectory CSV (6 figures)
│   │   └── compare_trajectories.m          # MATLAB: compare clamped_spline vs piecewise_linear
│   ├── urdf/
│   │   └── ur5_robotiq_2f85.urdf.xacro    # Combined UR5e + adapter + Robotiq 2F-85 URDF
│   └── worlds/
│       └── lab_base_world.sdf              # Gazebo world: ground, ur5_base, surgery_table, obstacle box
│
├── ur5_kinematics/                         # IK library
│   ├── include/ur5_kinematics/
│   │   └── kinematics.hpp                  # QP-based IK solver interface
│   ├── src/
│   │   └── kinematics.cpp                  # Pinocchio + OsqpEigen implementation
│   └── urdf/
│       └── ur5e.urdf                       # UR5e URDF used by Pinocchio
│
├── robotiq_description/                    # Gripper description (2F-85 only)
│   ├── meshes/
│   │   ├── collision/2f_85/
│   │   └── visual/2f_85/
│   └── urdf/
│       ├── robotiq_2f_85_macro.urdf.xacro
│       ├── 2f_85.ros2_control.xacro
│       └── ur_to_robotiq_adapter.urdf.xacro  # 11 mm UR-to-Robotiq adapter plate
│
└── ur5_trajectory_optimization/            # CU3 multi-objective optimizer (Python)
    ├── config/
    │   └── optimization_params.yaml        # NSGA-II / ε-constraint / IK / search domain
    ├── plots/
    │   └── comparison/                     # MATLAB comparison figures (PNG + EPS)
    ├── results/                            # Auto-generated optimisation outputs
    │   ├── pareto_nsga2.csv
    │   ├── pareto_epsilon.csv
    │   ├── selected_solution.yaml
    │   └── pareto_*.png                    # Pareto visualisation exports
    ├── scripts/
    │   ├── plot_pareto.py                  # Pareto front visualisation (3 figure types)
    │   └── compare_optimization.m          # MATLAB: compare NSGA-II vs ε-constraint trajectories
    └── ur5_trajectory_optimization/
        ├── trajectory_model.py             # Python port of clamped cubic spline (exact C++ match)
        ├── ik_interface.py                 # Damped least-squares IK with gripper_tcp offset
        ├── objective_evaluators.py         # f1 (RNEA), f2 (arc length), f3 (AABB clearance)
        ├── constraints.py                  # IK convergence + joint limit checks
        ├── multiobjective_optimizer.py     # NSGA-II (pymoo) + ε-constraint (scipy)
        ├── run_optimization.py             # Entry point: full two-stage pipeline
        └── export_selected_trajectory.py  # Export selected solution as param override YAML
```

---

## 7. Parameters

### 7.1 Pick-and-place node (`pick_place_params.yaml`)

| Parameter | Default | Description |
|---|---|---|
| `method` | `clamped_spline` | Interpolation method: `clamped_spline` or `piecewise_linear` |
| `points_per_segment` | `20` | Waypoints per trajectory segment |
| `total_duration` | `2.0` | Sim-time duration [s] of the B→O→C arc |
| `pre_post_duration` | `2.0` | Sim-time duration [s] of the A→B and C→D segments |
| `start_delay` | `2.5` | Sim-time delay [s] before the first waypoint (allows arm to reach home) |
| `tcp_orientation_rpy` | `[π, 0, −π/2]` | Fixed TCP orientation — gripper pointing down |
| `point_A` | `[0.00, 0.70, 0.15]` | Start / retraction point, pick side [m] |
| `point_B` | `[0.65, 0.55, 0.20]` | Pick / approach point [m] |
| `point_O` | `[0.75, 0.00, 0.40]` | Via-point over obstacle [m] ← **overridden by CU3** |
| `point_C` | `[0.65, −0.55, 0.20]` | Place / approach point [m] |
| `point_D` | `[0.00, −0.70, 0.15]` | End / retraction point, place side [m] |
| `home_joint_angles` | `[0.87, −0.80, 2.00, −2.20, −1.5708, 0.87]` | IK seed configuration [rad] |
| `ik_max_iterations` | `450` | Max IK solver iterations |
| `ik_alpha` | `0.5` | IK step size |
| `ik_weight_pos` | `1.0` | IK position error weight |
| `ik_weight_orient` | `1.0` | IK orientation error weight |
| `csv_output_dir` | `""` | CSV output directory (empty → `…/ur5_pick_place/data/`) |
| `obstacle_center` | `[0.85, 0.00, 0.10]` | Obstacle AABB centre [m] in base_link |
| `obstacle_half_extents` | `[0.20, 0.30, 0.10]` | Obstacle AABB half-dimensions [m] |
| `obstacle_r_grip` | `0.05` | Gripper enveloping radius [m] (clearance constraint) |
| `obstacle_delta_safe` | `0.05` | Additional safety margin [m] |
| `joint_q_min` | `[−2π, −2π, −π, −2π, −2π, −2π]` | UR5e lower joint limits [rad] |
| `joint_q_max` | `[ 2π,  2π,  π,  2π,  2π,  2π]` | UR5e upper joint limits [rad] |

All Cartesian parameters (waypoints, obstacle) are expressed in the **`base_link` frame** (z = 0 at robot base mounting surface). To convert to Gazebo world coordinates: **z_Gazebo = z_base_link + 0.63**.

### 7.2 Optimisation (`optimization_params.yaml`)

| Parameter | Default | Description |
|---|---|---|
| `via_x_min/max` | `0.50 / 0.95` | x search domain [m] — lower bound prevents JTC abort |
| `via_y_min/max` | `−0.25 / 0.25` | y search domain [m] |
| `via_z_min/max` | `0.25 / 0.60` | z search domain [m] — lower bound stays above obstacle top |
| `pts_per_seg` | `8` | Samples per segment during optimisation (coarser than simulation) |
| `pop_size` | `60` | NSGA-II population size |
| `n_gen` | `120` | NSGA-II generations |
| `seed` | `42` | Random seed |
| `n_epsilon_steps` | `25` | Number of ε levels in the ε-constraint sweep |
| `epsilon_obj_idx` | `0` | Objective bounded by ε (0 = f₁ effort, 1 = f₂ arc length) |
| `ik_max_iter` | `120` | Optimiser IK max iterations |
| `ik_tol` | `1e-4` | Optimiser IK convergence tolerance [m/rad] |
| `ik_lambda` | `0.05` | Levenberg-Marquardt damping factor |
| `ik_alpha` | `0.8` | Optimiser IK step size |
| `results_dir` | `""` | Output directory (empty → `…/ur5_trajectory_optimization/results/`) |

---

## 8. Output Data

### 8.1 Trajectory CSV

Each execution of `pick_place_node` exports a timestamped CSV to `ur5_pick_place/data/` with filename `trajectory_YYYYMMDD_HHMMSS_<method>.csv` and 32 columns:

| Column(s) | Description |
|---|---|
| `time_s` | Trajectory timestamp [s] |
| `tcp_x/y/z` | `gripper_tcp` Cartesian position [m] in base_link |
| `waypoint` | Keypoint tag: `0`=interpolated, `1`=A, `2`=B, `3`=O, `4`=C, `5`=D |
| `vel_x/y/z` | TCP Cartesian velocity [m/s] |
| `acc_x/y/z` | TCP Cartesian acceleration [m/s²] |
| `q0…q5` | Joint positions [rad] |
| `dq0…dq5` | Joint velocities [rad/s] |
| `tau0…tau5` | Joint torques [N·m] from Pinocchio RNEA: τ = M(q)q̈ + C(q,q̇)q̇ + g(q) |
| `jerk_x/y/z` | TCP jerk [m/s³]: analytic from spline coefficients; zero for piecewise linear |

### 8.2 Optimisation results

| File | Columns | Description |
|---|---|---|
| `pareto_nsga2.csv` | `via_x, via_y, via_z, f1_effort, f2_arclen, f3_clearance, g1_constr` | Pareto front from NSGA-II |
| `pareto_epsilon.csv` | `via_x, via_y, via_z, f1_effort, f2_arclen, f3_clearance` | Compromise solutions from ε-constraint |
| `selected_solution.yaml` | — | ROS 2 param override — sets `point_O` in `pick_place_node` |

### 8.3 MATLAB analysis scripts

All scripts auto-detect the most recent relevant CSV when no filename is specified.

**`ur5_pick_place/src/plots_trajectory.m`** — plot a single execution (6 figures):
- 3D TCP path + obstacle AABB
- Cartesian position x(t), y(t), z(t)
- Velocity and acceleration norms
- Joint positions q₀..q₅
- Joint velocities q̇₀..q̇₅
- Joint torques τ₀..τ₅

**`ur5_pick_place/src/compare_trajectories.m`** — compare `clamped_spline` vs `piecewise_linear` (4 figures):
- 3D TCP paths
- Cartesian position
- Velocity norm
- Acceleration norm

**`ur5_trajectory_optimization/scripts/compare_optimization.m`** — compare the two Gazebo execution CSVs recorded with the NSGA-II and ε-constraint via-points (6 figures):

| Figure | Content |
|---|---|
| Fig 1 | 3D TCP paths + obstacle AABB; O_N and O_ε marked |
| Fig 2 | Cartesian position x(t), y(t), z(t) with obstacle z-band |
| Fig 3 | ‖v‖, ‖a‖, ‖j‖ — smoothness and jerk comparison |
| Fig 4 | Joint velocities q̇₀..q̇₅ — diagnostic for JTC abort (joint 1 annotated) |
| Fig 5 | Joint torques τ₀..τ₅ — f₁ objective comparison |
| Fig 6 | Pareto fronts: f₁ vs f₂ and f₁ vs clearance, with selected solutions marked |

Pre-generated plots are stored in `ur5_trajectory_optimization/plots/comparison/` (PNG + EPS).

**Usage:**
```matlab
% In MATLAB — run from any directory:
run('~/ur5_ws/src/ur5_utec/ur5_trajectory_optimization/scripts/compare_optimization.m')
```

By default the script auto-detects the two most recent `clamped_spline` CSV files (older = NSGA-II, newer = ε-constraint). To specify files manually, set `FILE_NSGA2` and `FILE_EPSILON` at the top of the script.

Figures are exported to `ur5_trajectory_optimization/plots/comparison/` when `EXPORT_PNG = true` (default).
