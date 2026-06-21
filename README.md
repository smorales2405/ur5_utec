# ur5_utec

ROS 2 workspace for pick-and-place motion planning with a UR5 robot and Robotiq 2F-85 gripper in Gazebo Fortress. This repository bundles all four packages required to run the full simulation and multi-objective trajectory optimisation (CU3) without any additional workspace dependencies.

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

Launches Gazebo Fortress with the UTEC lab world, spawns the UR5 + Robotiq 2F-85 robot, and starts ros2_control with the `joint_trajectory_controller`.

```bash
ros2 launch ur5_pick_place ur5_robotiq_gz.launch.py
```

| Argument | Default | Description |
|---|---|---|
| `ur_type` | `ur5e` | UR robot model (`ur5`, `ur5e`, `ur10e`, …) |
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
| `node_start_delay` | `20.0` | Real-time seconds to wait before starting the pick-and-place node (increase on slower machines) |

```bash
# Example with piecewise linear and 30 s delay:
ros2 launch ur5_pick_place simulation.launch.py method:=piecewise_linear node_start_delay:=30.0
```

### 4.3 Pick-and-place node only (simulation already running)

Use this when the simulation is already up in another terminal.

```bash
ros2 launch ur5_pick_place pick_place.launch.py
```

```bash
# With piecewise linear interpolation:
ros2 launch ur5_pick_place pick_place.launch.py method:=piecewise_linear
```

### 4.4 Pick-and-place with optimised via-point (CU3)

After running the optimisation (see [Section 5](#5-cu3--multi-objective-trajectory-optimisation)), pass the selected solution via the `extra_params_file` launch argument:

```bash
# Terminal 1 — simulation
ros2 launch ur5_pick_place ur5_robotiq_gz.launch.py

# Terminal 2 — node with optimised via-point
ros2 launch ur5_pick_place pick_place.launch.py \
  extra_params_file:=$HOME/ur5_ws/src/ur5_utec/ur5_trajectory_optimization/results/selected_solution.yaml
```

`extra_params_file` is loaded after `pick_place_params.yaml`; only `point_via` is overridden, all other parameters remain unchanged.

Alternatively, using `ros2 run` directly:

```bash
ros2 run ur5_pick_place pick_place_node \
  --ros-args \
  --params-file ~/ur5_ws/src/ur5_utec/ur5_pick_place/config/pick_place_params.yaml \
  --params-file ~/ur5_ws/src/ur5_utec/ur5_trajectory_optimization/results/selected_solution.yaml
```

---

## 5. CU3 — Multi-objective Trajectory Optimisation

### Overview

CU3 treats the via-point position **p_via = [x, y, z]** as a 3-D decision variable and minimises three conflicting objectives simultaneously:

| Objective | Formula | Meaning |
|---|---|---|
| f₁ | ∫₀ᵀ Σᵢ τᵢ(t)² dt | Joint effort [N²·m²·s] — minimise actuator load |
| f₂ | ∫₀ᵀ ‖ṗ(t)‖ dt | TCP arc length [m] — minimise path length |
| f₃ | −d_min | Negative obstacle clearance [m] — maximise safety margin |

Subject to the hard constraint **d_min ≥ r_grip** (gripper does not penetrate the obstacle AABB).

The pipeline runs **offline** (no Gazebo required) using Pinocchio's RNEA for dynamics and a damped least-squares IK solver.

### Search domain

| Variable | Range | Unit |
|---|---|---|
| x_via | [0.55, 0.75] | m (Pinocchio frame) |
| y_via | [−0.10, 0.10] | m |
| z_via | [−0.45, −0.15] | m |

### Two-stage optimisation

**Stage 1 — NSGA-II** (pymoo): approximates the Pareto front with `pop_size=60`, `n_gen=120`.

**Stage 2 — ε-constraint** (scipy SLSQP): sweeps 25 ε-levels across the f₁ range to refine compromise solutions.

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
| `pareto_nsga2.csv` | Non-dominated front from NSGA-II (columns: via_x/y/z, f1, f2, f3) |
| `pareto_epsilon.csv` | Compromise solutions from ε-constraint sweep |
| `selected_solution.yaml` | ROS 2 param override with the knee-point `point_via` |

### Selecting a specific solution

```bash
# Re-export knee point from NSGA-II results (default):
ros2 run ur5_trajectory_optimization export_trajectory

# Export a specific row (e.g. index 7):
ros2 run ur5_trajectory_optimization export_trajectory --index 7

# Use ε-constraint results instead:
ros2 run ur5_trajectory_optimization export_trajectory --source epsilon
```

### Visualising the Pareto front

```bash
# Interactive 3-panel plot (3D scatter, 2D projections, parallel coordinates):
python3 ~/ur5_ws/src/ur5_utec/ur5_trajectory_optimization/scripts/plot_pareto.py

# Save PNG figures instead of displaying:
python3 ~/ur5_ws/src/ur5_utec/ur5_trajectory_optimization/scripts/plot_pareto.py --save
```

### Coordinate frame

All Cartesian points and the obstacle AABB are expressed in the **Pinocchio world frame**, where the robot base is at z = 0. In Gazebo, the robot base is spawned at z = 0.63 m:

```
z_pinocchio = z_gazebo − 0.63
```

The obstacle (`mesa_bidones`, a surgery table) is modelled as an AABB:
- **Centre:** [0.30, 0.00, −0.46] m
- **Half-extents:** [0.20, 0.30, 0.09] m → table top at z ≈ −0.37 m
- **Gripper radius r_grip:** 0.05 m
- **Safety margin δ_safe:** 0.05 m

---

## 6. Package Structure

```
ur5_utec/
│
├── ur5_pick_place/                         # Main package
│   ├── config/
│   │   ├── pick_place_params.yaml          # All pick-and-place parameters + obstacle geometry (CU3)
│   │   └── ur5_robotiq_controllers.yaml    # ros2_control controller configuration
│   ├── data/                               # CSV trajectory logs (auto-generated)
│   │   └── plots/                          # Exported figures
│   ├── include/ur5_pick_place/
│   │   ├── ik_wrapper.hpp                  # IK wrapper interface (tool0 → gripper_tcp offset)
│   │   └── trajectory_generator.hpp        # Cartesian trajectory generator interface
│   ├── launch/
│   │   ├── ur5_robotiq_gz.launch.py        # Simulation launch (Gazebo + controllers)
│   │   ├── simulation.launch.py            # Combined launch (simulation + pick-and-place)
│   │   └── pick_place.launch.py            # Pick-and-place node only
│   ├── meshes/                             # Gazebo SDF models for the UTEC lab
│   │   ├── bidon/                          # Object to manipulate
│   │   ├── surgery_table/                  # Obstacle model (mesa_bidones)
│   │   └── ur5_base/                       # Robot pedestal
│   ├── src/
│   │   ├── pick_place_node.cpp             # Main ROS 2 node (RNEA torque logging, extended CSV)
│   │   ├── ik_wrapper.cpp                  # TCP offset correction for gripper_tcp
│   │   ├── trajectory_generator.cpp        # Clamped spline (with analytic jerk) and linear
│   │   ├── plots_trajectory.m              # MATLAB: plot one trajectory CSV
│   │   └── compare_trajectories.m          # MATLAB: compare spline vs linear
│   ├── urdf/
│   │   └── ur5_robotiq_2f85.urdf.xacro    # Combined UR5 + adapter + Robotiq 2F-85 URDF
│   └── worlds/
│       └── lab_base_world.sdf              # Gazebo world (UTEC lab environment)
│
├── ur5_kinematics/                         # IK library
│   ├── include/ur5_kinematics/
│   │   └── kinematics.hpp                  # QP-based IK solver interface
│   ├── src/
│   │   └── kinematics.cpp                  # Pinocchio + OsqpEigen implementation
│   └── urdf/
│       └── ur5.urdf                        # UR5 URDF used by Pinocchio for kinematics
│
├── robotiq_description/                    # Gripper description (2F-85 only)
│   ├── meshes/
│   │   ├── collision/2f_85/                # Collision meshes (.stl)
│   │   └── visual/2f_85/                   # Visual meshes (.dae)
│   └── urdf/
│       ├── robotiq_2f_85_macro.urdf.xacro  # Gripper URDF macro
│       ├── 2f_85.ros2_control.xacro        # ros2_control hardware interface
│       └── ur_to_robotiq_adapter.urdf.xacro # 11 mm UR-to-Robotiq adapter plate
│
└── ur5_trajectory_optimization/            # CU3 multi-objective optimizer (Python)
    ├── config/
    │   └── optimization_params.yaml        # NSGA-II / ε-constraint / IK / domain parameters
    ├── results/                            # Auto-generated optimisation outputs
    │   ├── pareto_nsga2.csv
    │   ├── pareto_epsilon.csv
    │   └── selected_solution.yaml
    ├── scripts/
    │   └── plot_pareto.py                  # Pareto front visualisation (3 figure types)
    └── ur5_trajectory_optimization/
        ├── trajectory_model.py             # Python port of clamped cubic spline (exact C++ match)
        ├── ik_interface.py                 # Damped least-squares IK with TCP offset
        ├── objective_evaluators.py         # f1 (RNEA), f2 (arc length), f3 (clearance)
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
| `total_duration` | `2.0` | Sim-time duration [s] of the A→via→B arc |
| `pre_post_duration` | `0.5` | Sim-time duration [s] of approach (pre_A→A) and retreat (B→post_B) |
| `start_delay` | `1.0` | Sim-time delay [s] before the first waypoint |
| `tcp_orientation_rpy` | `[π, 0, -π/2]` | Fixed TCP orientation (gripper pointing down) |
| `point_A_pre` | `[0.1, 0.7, -0.54]` | Approach point above pick [m] (Pinocchio frame) |
| `point_A` | `[0.75, 0.38, -0.5]` | Pick point [m] |
| `point_via` | `[0.65, 0.0, -0.32]` | Arc via-point above the obstacle [m] ← **CU3 decision variable** |
| `point_B` | `[0.75, -0.48, -0.5]` | Place point [m] |
| `point_B_post` | `[0.1, -0.7, -0.54]` | Retreat point above place [m] |
| `home_joint_angles` | `[0, -π/2, π/2, -π/2, -π/2, 0]` | IK seed configuration [rad] |
| `ik_max_iterations` | `450` | Max IK solver iterations (3 × 150 internal cap) |
| `ik_alpha` | `0.5` | IK step size |
| `ik_weight_pos` | `1.0` | IK position error weight |
| `ik_weight_orient` | `1.0` | IK orientation error weight |
| `csv_output_dir` | `""` | CSV output directory (empty → `$HOME/ur5_ws/src/ur5_utec/ur5_pick_place/data`) |
| `obstacle_center` | `[0.30, 0.00, -0.46]` | Obstacle AABB centre [m] (Pinocchio frame) |
| `obstacle_half_extents` | `[0.20, 0.30, 0.09]` | Obstacle AABB half-dimensions [m] |
| `obstacle_r_grip` | `0.05` | Gripper enveloping radius [m] |
| `obstacle_delta_safe` | `0.05` | Additional safety margin [m] |
| `joint_q_min` | `[-2π, -2π, -π, -2π, -2π, -2π]` | UR5 lower joint limits [rad] |
| `joint_q_max` | `[ 2π,  2π,  π,  2π,  2π,  2π]` | UR5 upper joint limits [rad] |

### 7.2 Optimisation (`optimization_params.yaml`)

| Parameter | Default | Description |
|---|---|---|
| `via_x_min/max` | `0.55 / 0.75` | x search domain [m] |
| `via_y_min/max` | `-0.10 / 0.10` | y search domain [m] |
| `via_z_min/max` | `-0.45 / -0.15` | z search domain [m] |
| `pts_per_seg` | `8` | Samples per segment during optimisation (coarser than simulation) |
| `pop_size` | `60` | NSGA-II population size |
| `n_gen` | `120` | NSGA-II generations |
| `seed` | `42` | Random seed |
| `n_epsilon_steps` | `25` | Number of ε levels in the ε-constraint sweep |
| `epsilon_obj_idx` | `0` | Objective to bound with ε (0=f₁, 1=f₂) |
| `ik_max_iter` | `80` | Optimiser IK max iterations |
| `ik_tol` | `1e-4` | Optimiser IK convergence tolerance [m/rad] |
| `ik_lambda` | `0.05` | Levenberg-Marquardt damping factor |
| `ik_alpha` | `0.8` | Optimiser IK step size |
| `results_dir` | `""` | Output directory (empty → `$HOME/…/ur5_trajectory_optimization/results`) |

---

## 8. Output Data

### 8.1 Trajectory CSV

Each execution of `pick_place_node` exports a timestamped CSV to `ur5_pick_place/data/` with 31 columns:

| Column(s) | Description |
|---|---|
| `time_s` | Trajectory timestamp [s] |
| `tcp_x/y/z` | TCP Cartesian position [m] |
| `waypoint` | Keypoint tag (0=interpolated, 1=pre_A, 2=A, 3=via, 4=B, 5=post_B) |
| `vel_x/y/z` | TCP Cartesian velocity [m/s] (central finite differences) |
| `acc_x/y/z` | TCP Cartesian acceleration [m/s²] (central finite differences) |
| `q0…q5` | Joint positions [rad] |
| `dq0…dq5` | Joint velocities [rad/s] (central finite differences) |
| `tau0…tau5` | Joint torques [N·m] from Pinocchio RNEA: τ = M(q)q̈ + C(q,q̇)q̇ + g(q) |
| `jerk_x/y/z` | TCP jerk [m/s³]: analytic from spline coefficients (6d/dt³), zero for linear |

### 8.2 Optimisation results

| File | Columns | Description |
|---|---|---|
| `pareto_nsga2.csv` | `via_x, via_y, via_z, f1, f2, f3, g1` | Pareto front from NSGA-II |
| `pareto_epsilon.csv` | `via_x, via_y, via_z, f1, f2, f3` | Compromise solutions from ε-constraint |
| `selected_solution.yaml` | — | ROS 2 param override for `pick_place_node` |

### 8.3 MATLAB analysis scripts

```
# Plot a single trajectory (6 figures: 3D path, position, velocity, acceleration, joint pos/vel):
ur5_pick_place/src/plots_trajectory.m

# Compare clamped_spline vs piecewise_linear side by side (4 figures):
ur5_pick_place/src/compare_trajectories.m
```

Open either script in MATLAB, set `FILE_OVERRIDE` to a specific CSV filename or leave it empty to auto-load the most recent one, then run.
