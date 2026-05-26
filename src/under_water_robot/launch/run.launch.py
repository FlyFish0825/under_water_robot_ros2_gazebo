import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    sdf_file = os.path.join(os.getcwd(), 'Run_UnderWater_Robot.sdf')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {sdf_file}'}.items(),
    )

    bridge_config_path = os.path.join(
        get_package_share_directory('under_water_robot'),
        'config', 'bridge_config.yaml'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config_path}],
        output='screen'
    )

    thrust_controler_server = Node(
        package='thrust_controler_server',
        executable='thrust_controler_server',
        output='screen'
    )

    pubilsh_robot_state = Node(
        package='thrust_controler_server',
        executable='pubilsh_robot_state',
        output='screen'
    )

    return LaunchDescription([
        bridge,
        thrust_controler_server,
        pubilsh_robot_state,
        gz_sim,
    ])
