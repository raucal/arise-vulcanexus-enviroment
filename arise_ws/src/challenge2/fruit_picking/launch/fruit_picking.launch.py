import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import TimerAction, EmitEvent
from launch.substitutions import LaunchConfiguration
from launch.events import Shutdown
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Definir múltiples nodos adicionales
    node1 = Node(
        package='fruit_picking',
        executable='detect_fruits',
        name='detect_fruits',
        output='screen'
    )

    node2 = Node(
        package='fruit_picking',
        executable='robot_pick',
        name='robot_pick',
        output='screen'
    )

    node3 = Node(
        package='battery_disassembly',
        executable='audio_interface',
        name='audio_interface',
        output='screen'
    )

    node4 = Node(
        package='fruit_picking',
        executable='intel_transforms',
        name='intel_transforms',
        output='screen'
    )

    node5 = Node(
        package='fruit_picking',
        executable='fruits_manager',
        name='fruits_manager',
        output='screen'
    )

    node6 = Node(
        package='fruit_picking',
        executable='arise_ai',
        name='arise_ai',
        output='screen'
    )

    node7 = Node(
        package='fruit_picking',
        executable='linear_guide_controller',
        name='linear_guide_controller',
        output='screen'
    )
    
    # Incluir el archivo launch XML
    return LaunchDescription([
        node1,
        node2,
        node3,
        node4,
        node5,
        node6,
        node7,
    ])
