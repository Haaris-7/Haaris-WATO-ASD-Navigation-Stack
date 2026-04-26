# Autonomous Navigation Stack (ROS2 + Gazebo)

A full autonomous navigation stack for a simulated differential-drive robot
in **ROS2 Humble** and **Gazebo**. The robot navigates from point A to point
B around static obstacles using only a laser scanner and camera as sensor
inputs.

Built as my admissions submission for the **WATOnomous Autonomous Software
Division (ASD)**.

📺 **Demo video:** https://youtu.be/LgNThwbEUFw
📄 **Assignment spec:** https://wiki.watonomous.ca/admission_assignments/asd_admission_assignment/

---

## What it does

The stack runs as four interconnected ROS2 nodes implemented in C++:

| Node | Responsibility |
|---|---|
| **Costmap node** | Converts raw laser scans into local occupancy grids. |
| **Map Memory node** | Aggregates per-frame costmaps into a persistent global map. |
| **Planner node** | A* path search over the global costmap. |
| **Control node** | Pure Pursuit steering to track the planned path. |

The flow is: laser scan → local costmap → global map → A* path → Pure
Pursuit velocity commands → robot motion in Gazebo. Live state is
visualised through Foxglove.

---

## Stack

- **ROS2 Humble** (C++)
- **Gazebo** simulation
- **Foxglove** for visualisation
- **Docker** monorepo for reproducibility
- **CMake**, shell scripts, and a `watod` entrypoint for one-command runs

---

## Quick start

### Prerequisites

- Linux Ubuntu 22.04 or newer (or WSL on Windows, or macOS)
- Docker Engine
- Git

### Run it

```bash
git clone https://github.com/Haaris-7/<new-repo-name>.git
cd <new-repo-name>
./watod up
