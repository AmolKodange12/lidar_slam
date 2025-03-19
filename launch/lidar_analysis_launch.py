import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package path
    pkg_share = get_package_share_directory('lidar_slam')

    # Define Ignition Gazebo command
    gazebo_cmd = ExecuteProcess(
        cmd=['ign', 'gazebo', '-v', '4', '-r', 'visualize_lidar.sdf'],
        output='screen'
    )

    # Define ROS-Gazebo bridge for cmd_vel
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'],
        output='screen'
    )

    # Define ROS-Gazebo bridge for LIDAR topic
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/lidar2@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'],
        output='screen',
        remappings=[('/lidar2', '/laser_scan')]
    )
    
    # Define ROS-Gazebo bridge for Odometry topic
    odom_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=['/model/vehicle_blue/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry'],
    output='screen'
    )


    # Define Lidar Subscriber Node
    lidar_subscriber = Node(
        package='lidar_slam',
        executable='lidar_subscriber',
        name='lidar_subscriber',
        output='screen'
    )

    icp_node = Node(
        package='lidar_slam',
        executable='icp_node',
        name='icp_node',
        output='screen'
    )

    return LaunchDescription([
        gazebo_cmd,
        cmd_vel_bridge,
        lidar_bridge,
        odom_bridge,
        lidar_subscriber,
        icp_node
    ])

