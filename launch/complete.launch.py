import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    voice_cmd = ExecuteProcess(
    cmd=[
        '/home/minsu/ros2_ws/.venv/bin/python',
        '/home/minsu/ros2_ws/src/warehouse_bot/warehouse_voice/warehouse_voice/voice_whisper_claude_cmdvel.py',
        '--ros-args',
        '-p', 'require_wake_word:=true',
        '-p', 'wake_word:=robot',
        '-p', 'chunk_sec:=3.0'
    ],
    output='screen', 
    emulate_tty=True 
    )    

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
    urdf_file = os.path.join(pkg_share, 'urdf', 'warehouse_bot.urdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'warehouse_bot.rviz')
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

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

    # Spawn robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                  '-name', 'warehouse_bot',
                  '-z', '0.1'],
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
                  '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
                  '/model/warehouse_bot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ],
        remappings=[
            ('/model/warehouse_bot/tf', '/tf')
        ],
        output='screen'
    )

    # ROS-Gazeo Translate 
    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/default/model/warehouse_bot/joint_state'
            '@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        remappings=[
            ('/world/default/model/warehouse_bot/joint_state', '/joint_states')
        ],
        output='screen'
    )

#    return LaunchDescription([
#        gazebo,
#        robot_state_publisher,
#        joint_state_publisher,
#        spawn_entity,
#        slam_launch,
#        rviz,
#    ])

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        bridge,
        spawn_entity,
        slam_launch,
        rviz,
        joint_state_bridge,
        voice_cmd,
    ])
