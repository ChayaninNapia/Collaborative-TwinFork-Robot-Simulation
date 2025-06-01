import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_dir = get_package_share_directory('twinfork_description')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'robots', 'twinfork_namespace.xacro')
    
    def create_robot_description(context):
        ns = context.launch_configurations['namespace']
        param_file = context.launch_configurations['param_file']
        if ns.startswith('/'):
            ns = ns[1:]
        
        color = context.launch_configurations['robot_color']
        if not color:
            color = 'Gazebo/White'  
        
        print(f"[DEBUG] Launching {ns} with param file: {param_file}")
        print(f"[XACRO DEBUG] ns: {ns}, tf_prefix: {ns}, param_file: {param_file}")
            
        urdf_dir = os.path.join(pkg_dir, 'urdf', 'robots')
        xacro_file = os.path.join(urdf_dir, 'green.xacro')
        doc = xacro.process_file(
            xacro_file,
            mappings={
                'tf_prefix': ns,
                'robot_color': color,
                'param_file': param_file
            }
        )
        robot_desc = doc.toprettyxml(indent='  ')
        return [SetLaunchConfiguration('robot_description', robot_desc)]

    # Launch arguments
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock'
    )
    declare_joint_gui = DeclareLaunchArgument(
        'joint_state_publisher_enabled', default_value='false',
        description='Enable joint_state_publisher_gui'
    )
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace to push all topics to')
    
    declare_robot_color = DeclareLaunchArgument(
        'robot_color',
        default_value='',
        description='Color for the robot in Gazebo'
    )
    
    declare_param_file = DeclareLaunchArgument(
    'param_file',
    default_value='',
    description='Full path to controller param YAML file'
    )

    # LaunchConfiguration handles
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_joint_gui = LaunchConfiguration('joint_state_publisher_enabled')

    return LaunchDescription([
        declare_sim_time,
        declare_joint_gui,
        declare_namespace,
        declare_robot_color,
        declare_param_file,
        
        OpaqueFunction(function=create_robot_description),

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
            output='both',
            parameters=[{'use_sim_time': use_sim_time,
                        'robot_description': LaunchConfiguration('robot_description')}],
            namespace=LaunchConfiguration('namespace'),
        ),
    ])
