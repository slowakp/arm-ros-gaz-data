# arm-ros-gaz-data
This repository contains a ROS 2 + Gazebo Classic simulation environment for controlled generation of multi-sensor data from a mobile robotic platform. The project is designed primarily for research, experimentation, and education in mobile robot autonomy, perception, and sensor fusion.

# ROS 2 Sensor Simulation Repositories

This work consists of **two independent ROS 2 simulation projects**, each focused on a different sensor setup and simulation environment.
## Technologies

- ROS 2 Humble Hawksbill
- Gazebo Classic 11 / Gazebo Ignition
- gazebo_ros plugins (native ROS 2 integration)
- Python (motion control, utilities)
- rosbag2 (optional data recording)

---

## 1. Ignition Gazebo – Laser Scanner

- Simulation based on **Ignition Gazebo**
- Uses the predefined world `visualize_lidar.sdf`
- 2D laser scanner (LiDAR)
- ROS 2 communication via **ROS–Ignition bridge**
- Data exchange and control performed **exclusively from the command line**
- Published data:
  - laser scan / point cloud
  - TF frames
- Repository includes **three scripts** for basic LiDAR data analysis

This project focuses on command-line interaction, data bridging, and offline analysis of laser data.
```bash
ign gazebo -v 4 -r visualize_lidar.sdf
ros2 run ros_gz_bridge parameter_bridge /model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist
ros2 run ros_gz_bridge parameter_bridge /lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan --ros-args -r /lidar:=/laser_scan
ros2 run ros_gz_bridge parameter_bridge /lidar2@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan --ros-args -r /lidar2:=/laser_scan2
```


---

## 2. Gazebo Classic – IMU and Camera

- Simulation based on **Gazebo Classic**
- Custom simulation world with:
  - textured ground plane,
  - a floating box equipped with **IMU and RGB camera**
- Native ROS 2 integration via **gazebo_ros plugins** (no bridge)
- Recorded data:
  - camera images,
  - IMU measurements,
  - odometry
- The repository contains the complete ROS 2 package **`vision_gazebo`**


## Repository Structure

```
vision_gazebo/         
├── models/              # Sensor and platform models
├── worlds/              # Gazebo world files
├── launch/              # ROS 2 launch files
├── scripts/             # Optional motion control scripts
├── CMakeLists.txt
└── README.md
```
This project is intended for controlled generation of visual–inertial data in a reproducible simulation environment.

## Build

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

Set the Gazebo model path:

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:\
$HOME/ros2_ws/src/vision_gazebo/models
```

---

## Run

```bash
ros2 launch vision_gazebo vision_world.launch.py
```


---


## Data Recording

```bash
ros2 bag record /imu/data /camera/image_raw /scan
```

(Record only the topics relevant to the given repository.)

---

## Notes

- The two projects are **intentionally separated** to avoid mixing Ignition and Gazebo Classic workflows.
- Each repository is self-contained and can be used independently.

## License

Public repository intended for educational and research use.
No warranty is provided.
