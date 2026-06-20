# ur5_pick_place

ROS 2 package for pick-and-place motion planning with a UR5 robot and Robotiq 2F-85 gripper in Gazebo Fortress. Implements Cartesian trajectory generation (clamped cubic spline and piecewise linear) with iterative QP-based inverse kinematics (Pinocchio + OsqpEigen). The simulation environment is fully self-contained — no external simulation package required.

---

## Table of Contents

1. [Requirements](#1-requirements)
2. [Workspace Setup](#2-workspace-setup)
3. [Building](#3-building)
4. [Usage](#4-usage)
5. [Package Structure](#5-package-structure)
6. [Parameters](#6-parameters)
7. [Output Data](#7-output-data)

---

## 1. Requirements

### Assumed already installed
- ROS 2 Humble
- Gazebo Fortress
- Pinocchio
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

---

## 2. Workspace Setup

```bash
mkdir -p ~/ur5_ws/src
cd ~/ur5_ws/src
```

Clone the required packages:

```bash
# Main package
git clone https://github.com/smorales2405/ur5_pick_place.git

# UR5 kinematics library (provides ur5_kinematics used for IK)
git clone https://github.com/DavidValdezUtec/ur5_simulation.git

# Robotiq 2F-85 gripper description (URDF, meshes, ros2_control macros)
git clone https://github.com/PickNikRobotics/ros2_robotiq_gripper.git
```

> **Note:** `ur_description` is installed via apt (`ros-humble-ur-description`) and does **not** need to be cloned.

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

---

## 5. Package Structure

```
ur5_pick_place/
├── config/
│   ├── pick_place_params.yaml          # All pick-and-place node parameters
│   └── ur5_robotiq_controllers.yaml    # ros2_control controller configuration
├── data/                               # CSV trajectory logs (auto-generated)
│   └── plots/                          # Exported figures
├── include/ur5_pick_place/
│   ├── ik_wrapper.hpp                  # IK wrapper interface (tool0 → gripper_tcp offset)
│   └── trajectory_generator.hpp        # Cartesian trajectory generator interface
├── launch/
│   ├── ur5_robotiq_gz.launch.py        # Simulation launch (Gazebo + controllers)
│   ├── simulation.launch.py            # Combined launch (simulation + pick-and-place)
│   └── pick_place.launch.py            # Pick-and-place node only
├── meshes/                             # Gazebo SDF models for the UTEC lab
│   ├── bidon/                          # Object to manipulate
│   ├── surgery_table/                  # Table model
│   └── ur5_base/                       # Robot pedestal
├── src/
│   ├── pick_place_node.cpp             # Main ROS 2 node
│   ├── ik_wrapper.cpp                  # TCP offset correction for gripper_tcp
│   ├── trajectory_generator.cpp        # Spline and linear interpolation
│   ├── plots_trajectory.m              # MATLAB: plot one trajectory CSV
│   └── compare_trajectories.m          # MATLAB: compare spline vs linear
├── urdf/
│   └── ur5_robotiq_2f85.urdf.xacro    # Combined UR5 + adapter + Robotiq 2F-85 URDF
└── worlds/
    └── lab_base_world.sdf              # Gazebo world (UTEC lab environment)
```

---

## 6. Parameters

All parameters are in `config/pick_place_params.yaml` and can be overridden at launch time.

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
| `point_via` | `[0.65, 0.0, -0.32]` | Arc via-point above the obstacle [m] |
| `point_B` | `[0.75, -0.48, -0.5]` | Place point [m] |
| `point_B_post` | `[0.1, -0.7, -0.54]` | Retreat point above place [m] |
| `home_joint_angles` | `[0, -π/2, π/2, -π/2, -π/2, 0]` | IK seed configuration [rad] |
| `ik_max_iterations` | `450` | Max IK solver iterations (3 × 150 internal cap) |
| `ik_alpha` | `0.5` | IK step size |
| `ik_weight_pos` | `1.0` | IK position error weight |
| `ik_weight_orient` | `1.0` | IK orientation error weight |
| `csv_output_dir` | `~/ur5_ws/src/ur5_pick_place/data` | Directory for trajectory CSV logs |

> **Coordinate frame note:** All Cartesian points are expressed in the Pinocchio world frame, where the robot base is at z = 0. In Gazebo, the robot base is spawned at z = 0.63 m. So `z_pinocchio = z_gazebo − 0.63`.

---

## 7. Output Data

Each execution of `pick_place_node` exports a timestamped CSV to `data/` with the following columns:

| Column | Description |
|---|---|
| `time_s` | Trajectory timestamp [s] |
| `tcp_x/y/z` | TCP Cartesian position [m] |
| `waypoint` | Keypoint tag (0=interpolated, 1=pre_A, 2=A, 3=via, 4=B, 5=post_B) |
| `vel_x/y/z` | TCP Cartesian velocity [m/s] (central differences) |
| `acc_x/y/z` | TCP Cartesian acceleration [m/s²] (central differences) |
| `q0…q5` | Joint positions [rad] |
| `dq0…dq5` | Joint velocities [rad/s] (central differences) |

### MATLAB analysis scripts

```
# Plot a single trajectory (6 figures: 3D path, position, velocity, acceleration, joint pos/vel):
src/plots_trajectory.m

# Compare clamped_spline vs piecewise_linear side by side (4 figures):
src/compare_trajectories.m
```

Open either script in MATLAB, set `FILE_OVERRIDE` to a specific CSV filename or leave it empty to auto-load the most recent one, then run.
