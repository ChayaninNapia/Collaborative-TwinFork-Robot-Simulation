# src/twinfork_description/launch/display.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('twinfork_description')
    xacro_path = os.path.join(pkg, 'urdf', 'robots', 'basic.urdf.xacro')

    # 1) เพิ่ม launch-arg เพื่อเลือกเปิด/ปิด GUI
    declare_gui_arg = DeclareLaunchArgument(
        'use_joint_state_gui',
        default_value='true',
        description='Whether to launch joint_state_publisher_gui'
    )

    # 2) อ่านค่าจาก arg
    use_gui = LaunchConfiguration('use_joint_state_gui')

    return LaunchDescription([
        declare_gui_arg,

        # 3) ถ้า use_gui==true ให้สั่ง joint_state_publisher_gui
        Node(
            condition=IfCondition(use_gui),
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),

        # 4) robot_state_publisher เหมือนเดิม
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command([
                    'xacro ', xacro_path,
                    ' length:=1.0',
                    ' width:=0.5',
                    ' height:=0.2',
                    ' mass:=2.0',
                    ' wheel_radius:=0.05',
                    ' wheel_pos_z:=0.03'
                ])
            }]
        ),

        # 5) RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg, 'rviz', 'display.rviz')]
        ),
    ])
