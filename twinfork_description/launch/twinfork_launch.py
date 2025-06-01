import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('twinfork_description')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'robots', 'basic.urdf.xacro')

    # Launch arguments
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock'
    )
    declare_joint_gui = DeclareLaunchArgument(
        'joint_state_publisher_enabled', default_value='false',
        description='Enable joint_state_publisher_gui'
    )

    # LaunchConfiguration handles
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_joint_gui = LaunchConfiguration('joint_state_publisher_enabled')

    return LaunchDescription([
        declare_sim_time,
        declare_joint_gui,

        # Optional joint state publisher GUI
        Node(
            condition=IfCondition(use_joint_gui),
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # Robot state publisher with xacro processing
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': Command([
                    'xacro ', xacro_file,
                    ' use_sim_time:=', use_sim_time
                ])
            }]
        ),
    ])
