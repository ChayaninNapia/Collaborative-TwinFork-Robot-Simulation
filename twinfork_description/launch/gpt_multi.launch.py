import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_dir = get_package_share_directory('twinfork_description')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'robots', 'twinfork_namespace.xacro')

    def launch_setup(context, *args, **kwargs):
        ns = context.launch_configurations['namespace']
        param_file = context.launch_configurations['param_file']
        robot_color = context.launch_configurations['robot_color']
        use_sim_time = context.launch_configurations['use_sim_time']
        use_joint_gui = context.launch_configurations['joint_state_publisher_enabled']

        if ns.startswith('/'):
            ns = ns[1:]

        print(f"[DEBUG] Launching {ns} with param file: {param_file}")

        doc = xacro.process_file(
            xacro_file,
            mappings={
                'tf_prefix': ns,
                'robot_color': robot_color,
                'param_file': param_file
            }
        )
        robot_desc = doc.toprettyxml(indent='  ')

        nodes = []

        # Optional joint_state_publisher_gui
        if use_joint_gui.lower() == 'true':
            nodes.append(
                Node(
                    package='joint_state_publisher_gui',
                    executable='joint_state_publisher_gui',
                    name='joint_state_publisher_gui',
                    output='screen',
                )
            )

        # robot_state_publisher
        nodes.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                namespace=ns,
                output='both',
                parameters=[{
                    'use_sim_time': use_sim_time == 'true',
                    'robot_description': robot_desc
                }],


            )
        )

        return nodes

    # Declare arguments
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('robot_color', default_value='Gazebo/White'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('joint_state_publisher_enabled', default_value='false'),
        DeclareLaunchArgument('param_file', default_value=''),
        OpaqueFunction(function=launch_setup)
    ])
