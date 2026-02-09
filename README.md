# arm-ros-gaz-data
This repository contains a ROS 2 + Gazebo Classic simulation environment for controlled generation of multi-sensor data from a mobile robotic platform. The project is designed primarily for research, experimentation, and education in mobile robot autonomy, perception, and sensor fusion.

# ROS 2 Gazebo Sensor Simulation

This repository provides a minimal **ROS 2 + Gazebo Classic** simulation for a **single sensor setup**.
Each repository focuses on **one sensor only** (either IMU + Camera **or** Laser Scanner) to keep
experiments simple, reproducible, and easy to analyze.

The project is intended for **education, research, and benchmarking** of perception and sensor data
processing pipelines.

---

## Scope

Depending on the repository variant, the simulation includes **one of the following**:

### IMU + Camera project
- Inertial Measurement Unit (IMU) with configurable noise
- Monocular RGB camera
- Controlled planar motion to excite inertial measurements

### Laser Scanner project
- 2D laser scanner (LiDAR)
- Static or controlled-motion platform
- Deterministic range data generation

Each sensor configuration is maintained as a **separate repository** by design.

---

## Technologies

- ROS 2 Humble Hawksbill
- Gazebo Classic 11
- gazebo_ros plugins (native ROS 2 integration)
- Python (motion control, utilities)
- rosbag2 (optional data recording)

This project **does not use Ignition Gazebo** and **does not require a ROS–Gazebo bridge**.

---

## Repository Structure

```
project_root/
├── models/              # Sensor and platform models
├── worlds/              # Gazebo world files
├── launch/              # ROS 2 launch files
├── scripts/             # Optional motion control scripts
├── CMakeLists.txt
└── README.md
```

---

## Build

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

Set the Gazebo model path:

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:\
$HOME/ros2_ws/src/<repository_name>/models
```

---

## Run

```bash
ros2 launch <repository_name> <launch_file>.launch.py
```

---

## Example ROS 2 Topics

### IMU + Camera project
- /imu/data
- /camera/image_raw
- /camera/camera_info

### Laser Scanner project
- /scan

### Simulation
- /clock
- /odom (if motion enabled)

---

## Data Recording

```bash
ros2 bag record /imu/data /camera/image_raw /scan
```

(Record only the topics relevant to the given repository.)

---

## Design Notes

- One sensor per repository for clarity and reproducibility
- Gazebo Classic chosen for stability and mature ROS 2 support
- Motion is optional and used mainly to excite sensors
- Models are intentionally simple (no wheel dynamics)

---

## License

Public repository intended for educational and research use.
No warranty is provided.
