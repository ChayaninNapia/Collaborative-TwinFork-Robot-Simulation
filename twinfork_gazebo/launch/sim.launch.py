import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction


def generate_launch_description():
    warehouse_pkg_dir = get_package_share_directory('twinfork_gazebo')
    warehouse_launch_path = os.path.join(warehouse_pkg_dir, 'launch')
    rviz_config_path = os.path.join(warehouse_pkg_dir, 'rviz', 'robot.rviz')

    # Add Here
    mir_description_dir = get_package_share_directory('twinfork_description')

    warehouse_world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([warehouse_launch_path, '/world_bringup.launch.py'])
    )

    # Add Here
    launch_twinfork_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_description_dir, 'launch', 'twinfork_launch.py')
        )
    )

    launch_teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        namespace='',
        output='screen',
        prefix='xterm -e')
    
    spawn_robot = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    name='spawn_twinfork',
    arguments=['-entity', 'twinfork_robot',
               '-topic', 'robot_description'],
    output='screen',
    parameters=[{'use_sim_time': True}],
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
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["effort_controller", "--controller-manager", "/controller_manager"],
    )
    
    delayed_controller_loading = RegisterEventHandler(
    OnProcessExit(
        target_action=spawn_robot,
        on_exit=[
            TimerAction(
                period=3.0,
                actions=[
                    joint_state_broadcaster_spawner,
                    robot_controller_spawner,
                ]
            )
        ]
    )
)



    ld = LaunchDescription()

    # ld.add_action(launch_teleop)
    ld.add_action(warehouse_world_cmd)
    ld.add_action(launch_twinfork_description)
    ld.add_action(spawn_robot)
    ld.add_action(rviz)
    # ld.add_action(delayed_controller_loading)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(robot_controller_spawner)

    return ld

