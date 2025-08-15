# ROS2(jazzy) Obstacle Avoidance 

---

## 📌 Overview

This project implements a patrol behavior for a mobile robot in ROS2, enabling it to move continuously around an environment while avoiding obstacles in real-time.
Using LiDAR sensor data from the robot, the system:

- Subscribes to /scan topic for obstacle detection.
- Publishes velocity commands to /rosbot_xl_base_controller/cmd_vel for navigation.
- Adjusts the robot’s path dynamically to avoid collisions.


## 🛠 Features

- Patrol movement: Continuous forward motion when no nearby obstacles are detected.
- Dynamic obstacle avoidance:
- Turns away if an obstacle is closer than 0.15 m on either side.
- Stops and rotates if an obstacle is closer than 0.55 m in front.
- Chooses the clearer side for rotation when blocked ahead.
- ROS2 launch integration for easy execution.

---

## 📂 Package Structure

```text
ros2_ws/
└── src/
    └── ob_avoid_pkg/
        ├── ob_avoid_pkg/
        │   ├── __init__.py
        │   ├── code.py          # Main obstacle avoidance node
        ├── launch/
        │   ├── launch_node.launch.py
        ├── package.xml
        ├── setup.py
```

---

## 🚀 How It Works
LaserScan Input

From /scan topic, the system monitors three key points:

Right side: msg.ranges[2246]

Front: msg.ranges[0]

Left side: msg.ranges[748]

Decision Logic

Move forward if:

front > 0.55 m


Turn away if:

left_side < 0.15 m → turn right
right_side < 0.15 m → turn left


Rotate in place if:

front < 0.55 m → rotate toward the clearer side

---

## 🖥 Installation & Setup
1. Create Workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

2. Clone or Create Package
ros2 pkg create --build-type ament_python ob_avoid_pkg \
  --dependencies rclpy std_msgs sensor_msgs geometry_msgs

3. Add Code

Place code.py in ob_avoid_pkg/ob_avoid_pkg/.

4. Create Launch File

In ob_avoid_pkg/launch/launch_node.launch.py:

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ob_avoid_pkg',
            executable='exe',
            output='screen',
            emulate_tty=True
        ),
    ])

5. Update setup.py

Ensure console_scripts includes:

'exe = ob_avoid_pkg.code:main'

6. Build & Source
cd ~/ros2_ws
colcon build --packages-select ob_avoid_pkg
source ~/ros2_ws/install/setup.bash

7. Run
ros2 launch ob_avoid_pkg launch_node.launch.py

---

## 🎯 Expected Behavior

If no obstacle ahead (front > 0.55 m) → Move forward.

If side obstacle < 0.15 m → Turn away from obstacle.

If front obstacle < 0.55 m → Rotate toward clearer side.

---

## 🔮 Future Improvements

Add PID control for smoother turns.

Integrate SLAM for mapping while patrolling.

Implement dynamic speed adjustments based on obstacle proximity.
