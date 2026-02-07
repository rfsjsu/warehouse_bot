# warehouse_bot: Nav2

The following are instructions on how to get map creation with SLAM and navigation working.


## Prerequisites

Installed
* Linux Mint 22.3 (equivalent to Ubuntu 24.04)
* ROS2 Jazzy
* Python 3.12
* ROS2 Jazzy slam_toolbox
* ROS2 Jazzy teleop for keyboard or gamepad
* Nav2 1.0.0


## Selecting your robot and world
This example works with the simple_bot and block_world_01.  Presently it doesn't work with the simple forklift model because the LiDAR is to high to detect the blocks in the room.

In navigation.launch.py select the robot and world:
```
# Current robots available
# - simple_bot.urdf : Small diff drive robot with 1 camera and 1 LiDAR
# - warehouse_bot.urdf : Same as simple_bot, scaled up to look like a 2 wheeled forklift
#
BOT_MODEL = 'simple_bot.urdf'
# BOT_MODEL = 'warehouse_bot.urdf'

# Current worlds available
# - test.world : Plane with nothing on it.
# - block_world_01 : 20x20 room with obstacle blocks for navigation training.
# - Depot/model.sdf : Warehouse scene. Collision not included.
#
WORLD_MODEL = 'block_world_01.world'
# WORLD_MODEL = 'Depot/model.sdf'
```

## Settings to make a map
In config/nav2_params.yaml disable map loading:
```
# The yaml_filename does not need to be specified since it going to be set by defaults in launch.
# If you'd rather set it in the yaml, remove the default "map" value in the tb3_simulation_launch.py
# file & provide full path to map below. If CLI map configuration or launch default is provided, that will be used.
# map_server:
#   ros__parameters:
#     yaml_filename: /home/rsf/forklift_project/ros2_ws/src/warehouse_bot/maps/block_world_01/block_world_01.yaml

```

In config/mapper_params_online_async.yaml disable map loading:
```
    # if you'd like to immediately start continuing a map at a given pose
    # or at the dock, but they are mutually exclusive, if pose is given
    # will use pose
    # map_file_name: /home/rsf/forklift_project/ros2_ws/src/warehouse_bot/maps/block_world_01/block_world_01
    # map_start_pose: [0.0, 0.0, 0.0]
    # map_start_at_dock: true
```

In a terminal launch navigation.launch.py:
```
ros2 launch warehouse_bot navigation.launch.py
```

In a second terminal launch your keyboard or gamepad teleop so you can drive the robot:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

or

```
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
```

Gazebo and RVis should start.  In RVis you should see an empty map but the LiDAR point cloud should the outline of some nearby objects and walls.  Set the view to top down so it's easier to see the map you will create.

Load in the slam toolbox panel.

Slowly drive around the world and you will see a map start to form as it detects objects and walls with the LiDAR.  You have to drive slowly and take wide turns, otherwise localization fails and the map will be ruined.  I believe this is a problem with the inferred localization by only relying on the differential drive trying to guess the robot's pose.  Adding an IMU sensor may help.

Once the map is save, type in a map name into the slam toolbox panel.

See [
Easy SLAM with ROS using slam_toolbox](https://www.youtube.com/watch?v=ZaiA3hWaRzE) for an example on how to make a map and save it.


## How to navigate on the map you just created.
In the files above where you commented out the map settings, remove the comments and set the paths and file names to match your map file name and path.

In a terminal run:
```
ros2 launch warehouse_bot navigation.launch.py
```

In a second terminal run
```
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true params_file:=/home/rsf/forklift_project/ros2_ws/src/warehouse_bot/config/nav2_params.yaml
```

In RViz load in the navigation panel.  If all the nodes started correctly the navigation panel Navigation, Localization, and Feedback shoule read a green "active".  You should now see the map and be able to set a destination pose and the robot should move to that point.  The robot will have trouble with paths that have a lot of turns.  As mentioned before in the previous section, it seems the robot has some trouble with localiztion and needs adjustment or an IMU sensor.

See [
Making robot navigation easy with Nav2 and ROS!](https://www.youtube.com/watch?v=jkoGkAd0GYk) for a good tutorial on how to use navigation in RViz.

## History / Current State

TBD