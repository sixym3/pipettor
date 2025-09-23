#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUR',
        description='Serial port for Arduino communication'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Baudrate for serial communication'
    )
    
    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='Use fake hardware for testing without Arduino'
    )
    
    start_driver_arg = DeclareLaunchArgument(
        'start_driver',
        default_value='false',
        description='Whether to start the pipette driver node'
    )
    
    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    # Get launch configurations
    serial_port = LaunchConfiguration('serial_port')
    baudrate = LaunchConfiguration('baudrate')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    start_driver = LaunchConfiguration('start_driver')
    start_rviz = LaunchConfiguration('start_rviz')

    # Package paths
    pkg_pipette_description = FindPackageShare('pipette_description')
    pkg_urdf_launch = FindPackageShare('urdf_launch')
    
    # URDF and RViz config paths
    urdf_path = PathJoinSubstitution([pkg_pipette_description, 'urdf', 'pipette.urdf.xacro'])
    rviz_config_path = PathJoinSubstitution([pkg_pipette_description, 'rviz', 'pipette_demo.rviz'])

    # Launch URDF display (robot_state_publisher + joint_state_publisher + rviz)
    urdf_display_launch = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_urdf_launch, 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': 'pipette_description',
            'urdf_package_path': PathJoinSubstitution(['urdf', 'pipette.urdf.xacro']),
            'rviz_config': rviz_config_path,
            'jsp_gui': 'false'
        }.items(),
        condition=IfCondition(start_rviz)
    )

    # Launch pipette driver node
    pipette_driver_node = Node(
        package='pipette_driver',
        executable='pipette_driver_node',
        name='pipette_driver_node',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'baudrate': baudrate,
            'use_fake_hardware': use_fake_hardware,
        }],
        condition=IfCondition(start_driver)
    )

    # Joint state bridge is now provided by pipette_driver package
    # No need to launch it here - it's handled by pipette_moveit_config

    return LaunchDescription([
        # Launch arguments
        serial_port_arg,
        baudrate_arg,
        use_fake_hardware_arg,
        start_driver_arg,
        start_rviz_arg,
        
        # Nodes and launch files
        urdf_display_launch,
        pipette_driver_node,
    ])