# ur5_utec

ROS 2 workspace for pick-and-place motion planning with a UR5e robot and Robotiq 2F-85 gripper in Gazebo Fortress. This repository bundles all four packages required to run the full simulation and multi-objective trajectory optimisation (CU3) without any additional workspace dependencies.

| Package | Role |
|---|---|
| `ur5_pick_place` | Main node, Cartesian trajectory generation, simulation assets (URDF, world, meshes) |
| `ur5_kinematics` | QP-based inverse kinematics library (Pinocchio + OsqpEigen) |
| `robotiq_description` | URDF and meshes for the Robotiq 2F-85 gripper (trimmed to 2F-85 only) |
| `ur5_trajectory_optimization` | Offline multi-objective optimizer (CU3): NSGA-II + ε-constraint, Python — plus the *Trabajo Integrador* (3 numerical-methods pillars) |

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
9. [Test organisation](#9-test-organisation)
10. [Trabajo Integrador — Numerical-methods pillars](#10-trabajo-integrador--numerical-methods-pillars)

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
Per-generation hypervolume is tracked via a `Callback` and saved to `convergence_nsga2.csv` (B1).

**Stage 2 — ε-constraint 2D** (scipy SLSQP): two independent sweeps over the Pareto range:
- Sweep 1: `n_epsilon_f1=20` ε-levels over f₁ → minimise f₂ + f₃
- Sweep 2: `n_epsilon_f3=15` ε-levels over f₃ → minimise f₁ + f₂

The f₃ sweep adds solutions with varied clearance, giving 30× wider f₂ coverage than a single-axis f₁ sweep.

**Compromise selection**: minimum Euclidean distance to utopia in the space normalised over the **combined** NSGA-II + ε front (method configurable: `knee` | `min_effort` | `weighted`). The selection method and `norm_dist_utopia` are recorded in `selected_solution.yaml`.

### Running the optimisation

```bash
source ~/ur5_ws/install/setup.bash

# Run both stages (~20–30 min depending on hardware)
ros2 run ur5_trajectory_optimization run_optimization

# Run and store results under results/test1/
ros2 run ur5_trajectory_optimization run_optimization --test 1
```

Results are written to `ur5_trajectory_optimization/results/` (or `results/testN/` with `--test N`):

| File | Description |
|---|---|
| `pareto_nsga2.csv` | Non-dominated front from NSGA-II (columns: via_x/y/z, f1, f2, f3, g1) |
| `pareto_epsilon.csv` | Compromise solutions from 2D ε-constraint sweep (columns: via_x/y/z, f1, f2, f3) |
| `selected_solution.yaml` | ROS 2 param override — sets `point_O` in `pick_place_node` |
| `convergence_nsga2.csv` | Per-generation hypervolume + non-dominated set size (B1) |
| `method_comparison.csv` | Comparative metrics table: HV, spacing, IGD, C-metric, time, evals (B2) |

### Selecting and exporting a solution

`export_trajectory` uses Python argparse — arguments go **directly after the command**, without `--ros-args`.  
`selected_solution.yaml` is **always written to `results/`** root regardless of `--test`, so `extra_params_file` never needs to change.

```bash
# Knee point from ε-constraint (default — recommended):
ros2 run ur5_trajectory_optimization export_trajectory

# Read CSVs from test1/, write YAML to results/ root:
ros2 run ur5_trajectory_optimization export_trajectory --test 1

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
SCRIPT=~/ur5_ws/src/ur5_utec/ur5_trajectory_optimization/scripts/plot_pareto.py

# Interactive (3D scatter, 2D projections, parallel coordinates, convergence):
python3 $SCRIPT

# Save PNGs to results/ (no --test):
python3 $SCRIPT --save

# Save PNGs to results/test1/plots/pareto/:
python3 $SCRIPT --test 1 --save
```

When `convergence_nsga2.csv` is present a fourth figure is produced showing hypervolume and non-dominated set size per generation (B1 convergence curve).

### Baseline comparison (B3)

Evaluates the CU2 fixed via-point `[0.75, 0.00, 0.40]` with the same objectives and compares it against the selected CU3 solution:

```bash
# Compare against results in results/test1/:
ros2 run ur5_trajectory_optimization eval_baseline_cu2 --test 1
```

Outputs:
- `results/testN/baseline_vs_optimized.csv` — f₁, f₂, clearance + % improvement per objective
- `results/testN/plots/baseline_comparison.png` — grouped bar chart

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
    │   └── optimization_params.yaml        # NSGA-II / ε-constraint / selection / IK / search domain + final_project block
    ├── results/                            # All optimisation outputs (auto-generated)
    │   ├── selected_solution.yaml          # Active ROS 2 param override (always at root)
    │   ├── final/                          # Trabajo Integrador outputs (3 pillars)
    │   │   ├── integration_comparison.csv      # Pillar 1: quadrature convergence study
    │   │   ├── singleobjective_comparison.csv  # Pillar 2: solver comparison
    │   │   ├── selected_solution_final.yaml    # Pillar 2: canonical O★ (from SLSQP)
    │   │   ├── kkt_certificate.txt             # Pillar 2: KKT / bordered-Hessian certificate
    │   │   ├── baseline_vs_optimized.csv       # Pillar 2: CU2 baseline vs O★
    │   │   ├── dynamics_validation.csv         # Pillar 3: Euler vs RK4 error table
    │   │   └── plots/                          # integration_convergence, steepest_descent_path, euler_vs_rk4
    │   └── testN/                          # Per-test outputs (--test N)
    │       ├── pareto_nsga2.csv            # NSGA-II non-dominated front
    │       ├── pareto_epsilon.csv          # ε-constraint 2D sweep solutions
    │       ├── convergence_nsga2.csv       # HV + n_nondom per generation (B1)
    │       ├── method_comparison.csv       # HV, spacing, IGD, C-metric, time, evals (B2)
    │       ├── baseline_vs_optimized.csv   # CU2 vs CU3 objectives + % improvement (B3)
    │       ├── selected_solution.yaml      # Internal copy for the test run
    │       └── plots/
    │           ├── pareto/                 # plot_pareto.py output (PNG)
    │           ├── traj_comparison/        # compare_optimization.m output (PNG + EPS)
    │           └── baseline_comparison.png # eval_baseline_cu2 output (B3)
    ├── scripts/
    │   ├── plot_pareto.py                  # Pareto + convergence visualisation (--test N)
    │   ├── compare_optimization.m          # MATLAB: compare NSGA-II vs ε-constraint (TEST_ID)
    │   ├── compare_integration.py          # Pillar 1: quadrature convergence study → CSV + figure
    │   ├── kkt_certification.py            # Pillar 2: Lagrange + bordered Hessian of active bound
    │   └── validate_dynamics.py            # Pillar 3: Euler vs RK4 forward-dynamics validation
    └── ur5_trajectory_optimization/
        ├── trajectory_model.py             # Python port of clamped cubic spline (exact C++ match)
        ├── ik_interface.py                 # Damped least-squares IK with gripper_tcp offset
        ├── objective_evaluators.py         # f1 (RNEA), f2 (arc length), f3 (AABB clearance); f1 method= (Pillar 1)
        ├── constraints.py                  # IK convergence + joint limit checks
        ├── metrics.py                      # HV, spacing, IGD, C-metric, filter_nondominated (B1/B2)
        ├── multiobjective_optimizer.py     # NSGA-II + _HVCallback + 2D ε-constraint + selectors
        ├── run_optimization.py             # Entry point: two-stage pipeline + CSV outputs
        ├── export_selected_trajectory.py  # Export selected solution to results/ root YAML
        ├── eval_baseline_cu2.py           # CU2 baseline vs CU3 comparison + bar chart (B3)
        ├── numerical_integration.py        # Pillar 1: trapezoid/Simpson/Romberg/Gauss-Legendre (by hand)
        ├── integrands.py                   # Pillar 1: arc-length and effort integrands as callables
        ├── singleobjective_optimizer.py    # Pillar 2: steepest descent / direct search / SLSQP
        ├── run_singleobjective.py          # Pillar 2: entry point → comparison + canonical O★
        └── forward_dynamics_validation.py  # Pillar 3: pin.aba + Euler / RK4 (by hand)
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
| `n_epsilon_f1` | `20` | ε-constraint steps over f₁ range (minimises f₂ + f₃) |
| `n_epsilon_f3` | `15` | ε-constraint steps over f₃ range (minimises f₁ + f₂); `0` = 1D sweep only |
| `selection_method` | `knee` | Compromise selector: `knee` \| `min_effort` \| `weighted` |
| `weights` | `[1,1,1]` | Per-objective weights for `weighted` method [w_f1, w_f2, w_f3] |
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

Files at `results/` root (constant across tests):

| File | Description |
|---|---|
| `selected_solution.yaml` | Active ROS 2 param override — sets `point_O` in `pick_place_node` |

Files at `results/testN/` (one set per `--test N` run):

| File | Columns | Description |
|---|---|---|
| `pareto_nsga2.csv` | `via_x, via_y, via_z, f1_effort, f2_arclen, f3_clearance, g1_constr` | NSGA-II non-dominated front |
| `pareto_epsilon.csv` | `via_x, via_y, via_z, f1_effort, f2_arclen, f3_clearance` | 2D ε-constraint solutions |
| `convergence_nsga2.csv` | `gen, hypervolume, n_nondominated` | Per-generation HV (ref: `[20000, 3.0, 0.0]`) |
| `method_comparison.csv` | `method, n_solutions, n_nondominated, hypervolume, spacing, igd_vs_combined, c_vs_other, c_by_other, time_s, n_evals` | Method quality metrics |
| `baseline_vs_optimized.csv` | `solution, via_x/y/z, f1, f2, clearance_m, pct_improve_*` | CU2 vs CU3 comparison |
| `selected_solution.yaml` | — | Internal copy for this test run |

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

**Usage:**
```matlab
% In MATLAB — run from any directory:
run('~/ur5_ws/src/ur5_utec/ur5_trajectory_optimization/scripts/compare_optimization.m')
```

**Configuration block** (top of the script):

| Variable | Default | Description |
|---|---|---|
| `FILE_NSGA2` | `''` | Trajectory CSV for NSGA-II run (`''` = auto-detect oldest) |
| `FILE_EPSILON` | `''` | Trajectory CSV for ε-constraint run (`''` = auto-detect newest) |
| `TEST_ID` | `0` | Test number: `0` = flat `results/`; `N` → reads `results/testN/`, writes `results/testN/plots/traj_comparison/` |
| `EXPORT_PNG` | `true` | Export PNG (300 dpi) and EPS when `true` |

By default the script auto-detects the two most recent `clamped_spline` CSV files in `ur5_pick_place/data/` (older = NSGA-II, newer = ε-constraint). Set `FILE_NSGA2` / `FILE_EPSILON` to override.

Figures are written to `results/testN/plots/traj_comparison/` (PNG + EPS) when `TEST_ID > 0`, or `results/plots/traj_comparison/` when `TEST_ID = 0`.

---

## 9. Test organisation

All optimisation scripts accept a `--test N` argument (Python) or `TEST_ID = N` variable (MATLAB) that routes every output to a numbered subdirectory. This allows multiple independent runs to coexist without overwriting each other.

### Complete workflow for test N

```bash
source ~/ur5_ws/install/setup.bash

# 1. Run optimisation (Stage 1 NSGA-II + Stage 2 ε-constraint)
ros2 run ur5_trajectory_optimization run_optimization --test N

# 2. Export selected solution to results/ root (used by pick_place_node)
ros2 run ur5_trajectory_optimization export_trajectory --test N

# 3. Run Gazebo + pick-and-place with NSGA-II via-point
ros2 launch ur5_pick_place ur5_robotiq_gz.launch.py
ros2 launch ur5_pick_place pick_place.launch.py \
  extra_params_file:=$HOME/ur5_ws/src/ur5_utec/ur5_trajectory_optimization/results/selected_solution.yaml

# 4. (repeat step 3 for the ε-constraint via-point)

# 5. Generate Pareto + convergence plots
python3 ~/ur5_ws/src/ur5_utec/ur5_trajectory_optimization/scripts/plot_pareto.py --test N --save

# 6. Evaluate baseline CU2 vs CU3
ros2 run ur5_trajectory_optimization eval_baseline_cu2 --test N

# 7. Generate comparison plots in MATLAB (set TEST_ID = N, FILE_NSGA2, FILE_EPSILON)
run('~/ur5_ws/src/ur5_utec/ur5_trajectory_optimization/scripts/compare_optimization.m')
```

### Output layout per test

```
results/
├── selected_solution.yaml              ← updated by step 2 (always at root)
└── testN/
    ├── pareto_nsga2.csv
    ├── pareto_epsilon.csv
    ├── convergence_nsga2.csv           ← step 1 (B1)
    ├── method_comparison.csv           ← step 1 (B2)
    ├── baseline_vs_optimized.csv       ← step 6 (B3)
    ├── selected_solution.yaml          ← internal copy for this test
    └── plots/
        ├── pareto/                     ← step 5 (pareto_3d/2d/parallel, convergence)
        ├── traj_comparison/            ← step 7 (6 figures, PNG + EPS)
        └── baseline_comparison.png     ← step 6 (B3 bar chart)
```

### HV reference point

The hypervolume indicator uses a fixed reference point `[20 000, 3.0, 0.0]` (f₁ [N²·m²·s], f₂ [m], f₃) across all tests, ensuring HV values are directly comparable between runs.

---

## 10. Trabajo Integrador — Numerical-methods pillars

The *Trabajo Integrador* extends CU3 with three numerical-methods pillars (course MCI8102), using **only methods covered in class — implemented by hand** — and **reusing the existing CU3 infrastructure** (evaluator, IK, Pinocchio, trajectory model). All code is **additive**: the CU3 pipeline (`run_optimization`) is unchanged by default (`f1_joint_effort` keeps `method='trapezoid'`, which is bit-identical to `np.trapz`).

All outputs are written to `results/final/` (the `results/test1`, `results/test2` directories are never touched). Configuration lives in the `final_project:` block of `optimization_params.yaml`.

```bash
source ~/ur5_ws/install/setup.bash

# Pillar 2 first — it produces the canonical O★ used by the other pillars
ros2 run ur5_trajectory_optimization run_singleobjective

SCRIPTS=~/ur5_ws/src/ur5_utec/ur5_trajectory_optimization/scripts
python3 $SCRIPTS/compare_integration.py     # Pillar 1
python3 $SCRIPTS/kkt_certification.py        # Pillar 2 (certificate)
python3 $SCRIPTS/validate_dynamics.py        # Pillar 3
```

### Pillar 1 — Numerical integration comparison (Unit 5)

Compares **trapezoid, Simpson, Romberg and Gauss-Legendre** (hand-implemented in `numerical_integration.py`, with an evaluation counter) on the two objective integrands built in `integrands.py`:
- **Arc length** `‖ṗ(t)‖` — analytic from the spline derivative (clean, smooth integrand).
- **Effort** `Σᵢ τᵢ(t)²` — costly: samples `q(t)` by IK, estimates `q̇, q̈` by finite differences, calls `pin.rnea`.

```bash
python3 scripts/compare_integration.py        # uses O★ from selected_solution_final.yaml
```

| Output | Description |
|---|---|
| `results/final/integration_comparison.csv` | `objetivo, metodo, n, valor, error_abs, error_rel, n_evals` |
| `results/final/plots/integration_convergence.png` | Error vs. n (log-log), one curve per method per integrand |

- `f1_joint_effort(taus, times, method='trapezoid'|'simpson')` parametrises the rule and dispatches to `numerical_integration`. The default `'trapezoid'` preserves CU3 behaviour exactly.
- **Result:** on the smooth arc-length integrand Gauss-Legendre and Romberg converge spectrally (error ≈ 1e-13) while trapezoid stays ≈ 1e-4; on the effort integrand (IK + RNEA, less smooth) Gauss-Legendre still wins but by a smaller margin.
- **Study domain:** the integrands are evaluated over a single smooth spline segment `[t_B, t_C]` (the B→O→C arc containing O★). Integrating across spline junctions introduces zero-velocity corners that destroy the spectral convergence of global rules — the correct way to integrate across junctions is per-segment (exactly what `f2` does). The effort integrand uses a strict IK (`tol = 1e-8`) so the finite-difference `q̈` is not dominated by IK noise.

### Pillar 2 — Single-objective optimization with constraints (Unit 7, "Alternative 1b")

Solves `min f1(O)` subject to `d_min(O) ≥ d_safe` and the box bounds, with three solvers in `singleobjective_optimizer.py` reusing `TrajectoryEvaluator`:
- **`steepest_descent`** — exterior penalty + steepest descent (finite-difference gradient, bounded line search) with **projected gradient** so iterates slide along active bounds.
- **`direct_search`** — gradient-free coordinate (pattern) search, robustness check against evaluator noise.
- **`slsqp_reference`** — SLSQP with the explicit constraints (the professional reference).

```bash
ros2 run ur5_trajectory_optimization run_singleobjective
python3 scripts/kkt_certification.py
```

| Output | Description |
|---|---|
| `results/final/singleobjective_comparison.csv` | `metodo, xopt_x/y/z, f1, d_min, n_eval, n_iter, exito` |
| `results/final/selected_solution_final.yaml` | Canonical **O★** (from SLSQP, validated against steepest descent) |
| `results/final/baseline_vs_optimized.csv` | CU2 baseline `[0.75, 0, 0.40]` vs O★ + % improvement |
| `results/final/kkt_certificate.txt` | KKT multipliers, gradient, Hessian, bordered Hessian, verdict |
| `results/final/plots/steepest_descent_path.png` | Penalised `J̃` contour (x, z slice) + descent iterates |

- **Result:** all three solvers converge to the same **O★ = [0.50, ≈0, 0.25]**. The pure-effort minimum sits in a **corner with two active lower bounds** (x = 0.50 *and* z = 0.25), not just `x = 0.50`. The KKT certificate confirms it: multipliers `λ_x ≈ 4195`, `λ_z ≈ 837` (both ≥ 0, so `x = 0.50` is strictly active), `∂f1/∂y ≈ 0` (interior in y), and the reduced Hessian on the free y-direction `≈ 7676 > 0` (second-order sufficient condition) → minimum **certified**.
- **O★ vs CU3 knee:** they differ by ≈ 0.18 m. The single-objective optimum minimises effort (`+11 %` vs CU2) and arc length (`+2 %`) but **reduces clearance by ≈ 15 %** (0.127 → 0.108 m, still ≥ `d_safe = 0.10 m`). This is the expected single-objective vs multi-objective-compromise contrast — the CU3 knee balances all three objectives and keeps z = 0.43.

### Pillar 3 — ODE validation: Euler vs. RK4 (Unit 6.1–6.2)

Integrates the **forward** dynamics (`pin.aba`, the inverse of CU3's RNEA) with hand-coded **Euler** and **RK4** on the optimum's torque profile, validating dynamic feasibility before Gazebo. State `y = [q (6), q̇ (6)]` (12 states).

```bash
python3 scripts/validate_dynamics.py
```

| Output | Description |
|---|---|
| `results/final/dynamics_validation.csv` | `integrador, h, error_max, error_rms, estable` |
| `results/final/plots/euler_vs_rk4.png` | Joint tracking + error vs. h (log-log) |

- **Result:** over a bounded window of the via motion `[t_B, t_B + 1 s]`, compared against a **self-consistent fine-RK4 reference**, Euler converges as `O(h)` and RK4 as `O(h⁴)` — RK4 is 4 000× to 5 000 000× more accurate at the same `h`.
- **Stability finding:** integrating the full 6 s open-loop **diverges for both methods** — the nominal trajectory is an unstable solution of the manipulator ODE, so any integrator error grows exponentially. This motivates the closed-loop joint-trajectory controller used in Gazebo. The bounded window + self-consistent reference isolate the integrators' order from both the plant instability and the IK-reference noise.

### Configuration (`final_project:` block)

| Parameter | Default | Pillar | Description |
|---|---|---|---|
| `integration_n_values` | `[4,8,16,32,64,128]` | 1 | n values for the convergence study |
| `integration_reference_n` | `2048` | 1 | n for the fine effort reference |
| `effort_integration_method` | `trapezoid` | 1 | tabular rule used by `f1` (CU3 default) |
| `penalty_mu` | `1.0e5` | 2 | penalty weight (clearance + box) |
| `d_safe_extra` | `0.05` | 2 | `d_safe = obstacle_r_grip + d_safe_extra` |
| `sd_x0` | `[0.70, 0.0, 0.40]` | 2 | solver seed (CU2 via-point) |
| `sd_tol` / `sd_max_iter` | `1e-4` / `50` | 2 | steepest-descent tolerance / iterations |
| `ode_h_values` | `[0.02,0.01,0.005,0.002]` | 3 | integration steps to compare |
| `ode_horizon` | `1.0` | 3 | validation window length `[t_B, t_B+horizon]` |
| `ode_ref_h` | `1.0e-4` | 3 | step of the self-consistent RK4 reference |
