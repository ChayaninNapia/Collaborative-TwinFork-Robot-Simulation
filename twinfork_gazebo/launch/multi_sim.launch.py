import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    warehouse_pkg_dir = get_package_share_directory('twinfork_gazebo')
    warehouse_launch_path = os.path.join(warehouse_pkg_dir, 'launch')
    rviz_config_path = os.path.join(warehouse_pkg_dir, 'rviz', 'robot.rviz')

    # Add Here
    mir_description_dir = get_package_share_directory('twinfork_description')

    warehouse_world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([warehouse_launch_path, '/world_bringup.launch.py'])
    )

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace to push all topics to'
    )
    declare_robot_color = DeclareLaunchArgument(
        'robot_color',
        default_value='Gazebo/Green',
        description='Color for the robot in Gazebo'
    )
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )
    declare_joint_gui = DeclareLaunchArgument(
        'joint_state_publisher_enabled',
        default_value='false',
        description='Enable joint_state_publisher_gui'
    )

    Ai_Daeng_param_file=os.path.join(mir_description_dir, 'config', 'Ai_Daeng_lifter_controller.yaml')
    
    launch_Ai_Daeng_twinfork_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_description_dir, 'launch', 'multi_twinfork.launch.py')
        ),
        launch_arguments={
            'namespace': 'Ai_Daeng',
            'robot_color': 'Gazebo/Red',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'joint_state_publisher_enabled': LaunchConfiguration('joint_state_publisher_enabled'),
            'param_file': Ai_Daeng_param_file  
        }.items()
    )
    
    Ai_Khieow_param_file=os.path.join(mir_description_dir, 'config', 'Ai_Khieow_lifter_controller.yaml')
    
    launch_Ai_Khieow_twinfork_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_description_dir, 'launch', 'multi_twinfork.launch.py')
        ),
        launch_arguments={
            'namespace': 'Ai_Khieow',
            'robot_color': 'Gazebo/Green',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'joint_state_publisher_enabled': LaunchConfiguration('joint_state_publisher_enabled'),
            'param_file': Ai_Khieow_param_file  
        }.items()
    )

    launch_teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        namespace='',
        output='screen',
        prefix='xterm -e')
    
    spawn_Ai_Daeng_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_twinfork',
        namespace='Ai_Daeng',
        arguments=['-entity', 'twinfork_red',
                '-topic', 'robot_description',
                # กำหนดตำแหน่ง (x, y, z)
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.1',
                # กำหนด yaw (rad)
                '-Y', '1.57',
                ],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                 'robot_description': LaunchConfiguration('robot_description')}],
    )
    
    spawn_Ai_Khieow_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_twinfork',
        namespace='Ai_Khieow',
        arguments=['-entity', 'twinfork_green',
                '-topic', 'robot_description',
                # กำหนดตำแหน่ง (x, y, z)
                '-x', '1.0',
                '-y', '0.0',
                '-z', '0.1',
                # กำหนด yaw (rad)
                '-Y', '1.57',
                ],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                 'robot_description': LaunchConfiguration('robot_description')}],
    )
    
    spawn_khieow_after_daeng = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_Ai_Daeng_robot,
            on_exit=[spawn_Ai_Khieow_robot],
        )
    )

    
      # Create the RViz2 node
    rviz = Node(
         package='rviz2',
         executable='rviz2',
         name='rviz2',
         arguments=['-d', rviz_config_path],
         output='screen',
         parameters=[{'use_sim_time': True}]
    )
    
    Ai_Daeng_joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace='Ai_Daeng',
        arguments=["joint_state_broadcaster", "-c", "/Ai_Daeng/controller_manager"],
    )

    Ai_Daeng_robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace='Ai_Daeng',
        arguments=["effort_controller", "-c", "/Ai_Daeng/controller_manager"],
    )
    
    Ai_Khieow_joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace='Ai_Khieow',
        arguments=["joint_state_broadcaster", "-c", "/Ai_Khieow/controller_manager"],
    )

    Ai_Khieow_robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace='Ai_Khieow',
        arguments=["effort_controller", "-c", "/Ai_Khieow/controller_manager"],
    )
    
    red_controller = Node(
    package='twinfork_controller',
    executable='red_controller.py',
    name='red_controller',
    output='screen'  # <-- suppress terminal output
    )

    green_controller = Node(
        package='twinfork_controller',
        executable='green_controller.py',
        name='green_controller',
        output='screen'  # <-- suppress terminal output
    )
    
    virtual_controller = Node(
        package='twinfork_controller',
        executable='virtual_link_controller.py',
        name='virtual_controller',
        output='screen'  
    )
    
    mission_planner = Node(
        package='twinfork_controller',
        executable='mission_planner.py',
        name='mission_planner',
        output='screen' 
    )
    
    static_tf_daeng = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_daeng',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'Ai_Daeng/odom'],
        )
    static_tf_khieow = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_khieow',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'Ai_Khieow/odom'],
        )
    virtual_forklift_broadcaster = Node(
            package='twinfork_controller',
            executable='virtual_link.py',
            name='virtual_forklift_broadcaster',
            output='screen',
            parameters=[{'use_sim_time': True}]  # อย่าลืมใช้ sim time ด้วย
        )
    
    ld = LaunchDescription()
    
    ld.add_action(declare_robot_color)
    ld.add_action(declare_namespace)
    ld.add_action(declare_sim_time)
    ld.add_action(declare_joint_gui)
    
    
    # ld.add_action(launch_teleop)
    ld.add_action(warehouse_world_cmd)
    ld.add_action(virtual_forklift_broadcaster)
    ld.add_action(rviz)
    ld.add_action(static_tf_daeng)
    ld.add_action(static_tf_khieow)

    
    ld.add_action(launch_Ai_Daeng_twinfork_description)
    ld.add_action(spawn_Ai_Daeng_robot)
    ld.add_action(Ai_Daeng_joint_state_broadcaster_spawner)
    ld.add_action(Ai_Daeng_robot_controller_spawner)
    
    
    ld.add_action(launch_Ai_Khieow_twinfork_description)
    ld.add_action(spawn_Ai_Khieow_robot)
    ld.add_action(Ai_Khieow_robot_controller_spawner)
    ld.add_action(Ai_Khieow_joint_state_broadcaster_spawner)
    
    ld.add_action(red_controller)
    ld.add_action(green_controller)
    ld.add_action(virtual_controller)
    ld.add_action(mission_planner)

    return ld


