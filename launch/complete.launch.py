import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('warehouse_bot')
    
        # Path to the custom world file
    world_file_name = 'Depot/model.sdf' #'test.world'
    world_path = os.path.join(pkg_share, 'world', world_file_name)
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': ['-r ', world_path], 
        'on_exit_shutdown': 'true'}.items()
        # launch_arguments={'world': world_path}.items(),
    )


    # URDF file
    # urdf_file = os.path.join(pkg_share, 'urdf', 'warehouse_bot.urdf')
    # rviz_config = os.path.join(pkg_share, 'rviz', 'warehouse_bot.rviz')
    
    # with open(urdf_file, 'r') as infp:
    #     robot_desc = infp.read()

    # RViz configuration file
    rviz_config = os.path.join(pkg_share, 'rviz', 'warehouse_bot.rviz')
    
    # URDF file
    # urdf_file = os.path.join(pkg_share, 'urdf', 'warehouse_bot.urdf')
    # with open(urdf_file, 'r') as infp:
    #     robot_desc = infp.read()

    xacro_file = os.path.join(pkg_share, 'urdf', 'cangozpi_forklift/forklift.urdf.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
    )

    # Joint State Publisher GUI
    # Without this the forklift fork is stuck at the origin when changing
    # the fixed frame.
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # Spawn robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                  '-name', 'warehouse_bot',
                  'x', '0.0', '-z', '0.1'],
        output='screen'
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')]),
        launch_arguments={
            'slam_params_file': os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml'),
            'use_sim_time': 'true'}.items(),
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                  '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        joint_state_publisher_gui,
        bridge,
        spawn_entity,
        slam_launch,
        rviz,
    ])
