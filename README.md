# Physical AI v1.1 — Utsuro-bune
# Physical AI v1.0 — Alien UFO Adventure

## Project Overview
This project demonstrates an AI-based obstacle avoidance robot using ROS 2 Jazzy and Gazebo Harmonic.

A Gray alien rides in a UFO, avoids Mount Fuji using AI and LiDAR, reaches the red torii gate, and stops automatically.

## Environment
- Ubuntu 24.04 (WSL2)
- ROS 2 Jazzy
- Gazebo Harmonic
- Python 3
- scikit-learn

## Features
- LiDAR-based obstacle detection
- Machine learning-based obstacle avoidance
- Differential-drive robot simulation
- Goal-directed navigation using odometry
- Automatic stop at the goal

## Project Structure
- `models/` - Robot model
- `worlds/` - Gazebo simulation world
- `scripts/` - AI controller and trained model

## How to Run
Start Gazebo:

```bash
cd ~/physical_ai_final
/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz sim worlds/robot_world.sdf
```

Start the ROS 2 bridge in another terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' \
  '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' \
  '/model/ai_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'
```

Start the AI controller in a third terminal:

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/ai_controller_success.py
```

## Demonstration
The alien UFO avoids Mount Fuji and navigates to the red torii gate.

## Result
The robot successfully reached the goal and stopped automatically.
  
