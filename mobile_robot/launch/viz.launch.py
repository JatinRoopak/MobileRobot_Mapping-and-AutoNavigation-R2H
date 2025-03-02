import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
import xacro

def generate_launch_description():

    package_name = 'mobile_robot'

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    
    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('mobile_robot'))
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    print(f"Xacro file path: {xacro_file}")
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control])

    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    gazebo_param_file = os.path.join(get_package_share_directory(package_name), 'config', 'gazebo_params.yaml')
    # Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'extra_gazebo_args': '--ros-args --param-file ' + gazebo_param_file}.items()
    )

    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    launch_arguments={
        'world': os.path.join(pkg_path, 'worlds', 'museum.world'),
        'verbose': 'true'
    }.items()
    )

    # Node to spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                  '-entity', 'robot'],
        output='screen'
    )

    # # Optional: Add joint_state_publisher_gui
    # joint_state_publisher_gui = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui'
    # )

    # Controllers (conditional on use_ros2_control)
    load_joint_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_broad'],
        output='screen',
        condition=IfCondition(use_ros2_control)
    )
    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'diff_cont'],
        output='screen',
        condition=IfCondition(use_ros2_control)
    )

    # twist_mux configuration file
    twist_mux_params = os.path.join(get_package_share_directory(package_name), 
                                   'config', 
                                   'twist_mux.yaml')
    
    # twist_mux node
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')],
        output='screen'
    )


    # SLAM toolbox
    slam_params_file = os.path.join(pkg_path, 'config', 'mapper_params_online_async.yaml')
    slam_toolbox = Node(
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    # Rviz 2
    rviz_config_file = os.path.join(pkg_path, 'config', 'rviz_config_nav2.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',  # Changed to true since we're using Gazebo
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',  # Changed to true since we're using Gazebo
            description='Use ros2 control if true'),
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        load_joint_broadcaster,
        load_diff_drive_controller,
        twist_mux_node,
        slam_toolbox,
        rviz_node,
        #joint_state_publisher_gui
    ])