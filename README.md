# warehouse_bot
MSAI project: Autonomous forklift

This code originated from two tutorials by Ibrahim Mansur and Mike Hart:

* https://ibrahimmansur4.medium.com/building-a-differential-drive-robot-in-ros-2-from-urdf-to-gazebo-classic-simulation-23960714e2a9
* https://mikelikesrobots.github.io/blog/llm-robot-control/


Links to their Github code are found in those articles.

## How To Build And Run The Code

This code was developed with
* Linux Mint 22.3 (equivalent to Ubuntu 24.04)
* ROS2 Jazzy
* Python 3.12


To build:
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/rfsjsu/warehouse_bot.git
cd ~/ros2_ws
colcon build --packages-select warehouse_bot --symlink-install
```

To run Gazebo Sim and RViz:
```
cd ~/ros2_ws
source install/setup.bash
ros2 launch warehouse_bot.git complete.launch.py
```

## History / Current State

v0.1: Basic 3 wheeled bot with differential drive.  World is a small warehouse.  Camera and LiDAR stream data and can be visualized in rviz2.