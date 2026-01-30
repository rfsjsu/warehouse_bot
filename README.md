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
ros2 launch warehouse_bot complete.launch.py
```

## To Do

* Replace simple wheeled bot with a forklift model 
    * Partially Done: Model constructed, forklift doesn't move up/down.
    * Has one 2D LiDAR and 1 forward facing camera.
* Add collision detection.
    * The current forklift passes right through boxes and walls.
* Add ROSA node for LLM 3.5 control
    * Layout the world in a grid, the agent can go to a specified grid and report its location.
    * Couple with computer vision and the agent can say what it sees if asked.
* Set up Nav2
    * SLAM to make a map of the warehouse.
* Upgrade LiDAR
    * Current lidar is 2D, missing objects too low.
    * Upgrade to 3D lidar so we can get point cloud image of shelves.
* Add computer vision node
    * Recognize boxes and pallets with object detection.
    * Localization of objects on the map (e.g. "there is a box at grid 17")
    * Segmentation of pallets and recognize where the fork fits under the pallet.
* Setup framework for reinforcement learning
    * First attempt should be to get to a defined location and pose, i.e. properly lined up to pick up a pallet.

## History / Current State

v0.1: Basic 3 wheeled bot with differential drive.  World is a small warehouse.  Camera and LiDAR stream data and can be visualized in rviz2.