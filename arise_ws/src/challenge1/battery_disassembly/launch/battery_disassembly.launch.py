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
    # PUENTE WEBSOCKET
    rosbridge_server_launch_file = os.path.join(
        get_package_share_directory('rosbridge_server'),
        'launch',
        'rosbridge_websocket_launch.xml'  
    )
    
    node0 = Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen'
        )

    # Definir múltiples nodos adicionales
    # node1 = Node(
    #     package='battery_disassembly',
    #     executable='ur10e_screw',
    #     name='ur10e_screw',
    #     output='screen'
    # )

    node2 = Node(
        package='battery_disassembly',
        executable='detect_screws',
        name='detect_screws',
        output='log'
    )

    node3 = Node(
        package='battery_disassembly',
        executable='audio_interface',
        name='audio_interface',
        output='screen'
    )

    node4 = Node(
        package='battery_disassembly',
        executable='hri_communication',
        name='hri_communication',
        output='screen'
    )

    node5 = Node(
        package='hl2_streamming',
        executable='body_stream',
        name='body_stream',
        output='log'
    )

    # CAMERA
    # realsense_launch_file = os.path.join(
    #     get_package_share_directory('realsense2_camera'),
    #     'launch',
    #     'rs_launch.py'  # Cambia a '.py' si el archivo es Python
    # )
    
    # # Verificar que el archivo existe
    # if not os.path.isfile(realsense_launch_file):
    #     raise FileNotFoundError(f"Archivo no encontrado: {realsense_launch_file}")

    # Incluir el archivo launch XML
    return LaunchDescription([
        # IncludeLaunchDescription(
        #     AnyLaunchDescriptionSource(rosbridge_server_launch_file)
        # ),
        # node1,
        node0,
        node2,
        node3,
        node4,
        node5
        # TimerAction(actions=[EmitEvent(event=Shutdown())], period=3.0),
        # IncludeLaunchDescription(
        #     AnyLaunchDescriptionSource(realsense_launch_file),
        #     launch_arguments={'depth_module.profile': '1280x720x30',
        #                       'rgb_camera.profile': '1280x720x30',
        #                       'enable_sync': 'true',
        #                       'align_depth.enable': 'true',
        #                       'enable_infra1': 'true'}.items()
        # )
    ])
