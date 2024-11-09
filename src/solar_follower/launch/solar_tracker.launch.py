from launch import LaunchDescription
from launch.actions import RegisterEventHandler, ExecuteProcess, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('solar_follower')
    
    # Cargar URDF
    urdf_file = os.path.join(pkg_share, 'description', 'solar_panel.urdf')
    world_file = os.path.join(pkg_share, 'worlds', 'solar_world.world')
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Asegurarse de que Gazebo se inicie correctamente
    gazebo = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_file, 
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # GUI de Gazebo como proceso separado
    gazebo_gui = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # Spawn del robot con delay
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'solar_panel',
            '-file', urdf_file,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ]
    )

    delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn_entity]
    )

    # Controller Spawner
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            'joint_trajectory_controller'
        ]
    )

    # Solar Tracker Node
    solar_tracker = Node(
        package='solar_follower',
        executable='solar_tracker',
        name='solar_tracker',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        gazebo_gui,
        robot_state_publisher,
        delayed_spawn,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[controller_spawner]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=controller_spawner,
                on_exit=[solar_tracker]
            )
        )
    ])