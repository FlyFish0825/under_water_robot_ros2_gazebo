# 导入操作系统模块，用于文件路径操作
import os

# 导入 LaunchDescription，用于创建启动描述对象
from launch import LaunchDescription

# 导入 IncludeLaunchDescription，用于包含其他启动文件
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable

# 导入 PythonLaunchDescriptionSource，用于加载 Python 格式的启动文件
from launch.launch_description_sources import PythonLaunchDescriptionSource

# 导入 Node，用于创建 ROS 2 节点
from launch_ros.actions import Node

# 导入 get_package_share_directory，用于获取 ROS 包的共享目录路径
from ament_index_python.packages import get_package_share_directory

# 导入获取包前缀的工具

from ament_index_python.packages import get_package_share_directory, get_package_prefix 
# 导入 logging 模块
import rclpy
from rclpy.logging import get_logger

def find_exact_file(search_path, target_file):
    """
    在指定目录及其所有子目录中搜索精确的文件名
    search_path : 要搜索的根目录
    target_file : 要搜索的文件名（不包含路径）
    """
    absolute_search_path = os.path.abspath(search_path)
    for root, dirs, files in os.walk(absolute_search_path):
        if target_file in files:
            # 找到文件，拼接完整路径
            full_path = os.path.join(root, target_file)
            print(f"找到文件: {full_path}")
            return full_path
            
    print("未找到该文件。")
    return None







def generate_launch_description():
    # 1. 指定 Gazebo 的启动文件路径
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    get_logger('launch_logger').warn(f"ros_gz_sim path: {pkg_ros_gz_sim}")
    # 2. 启动 Gazebo 仿真服务 (gz_sim)    调用另一个launch文件
    # gz_args 指定要加载的世界文件和运行模式 (-r 表示立即运行)
    # 获取当前工作区的 SDF 文件路径
    sdf_file = os.path.join(os.getcwd(), 'Run_UnderWater_Robot.sdf')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {sdf_file}'
        }.items(),
    )


    plugin_path = find_exact_file('.','libTorquePlugin.so')
    if plugin_path is None:
        raise FileNotFoundError("未找到 libTorquePlugin.so 插件文件，请确保它存在于当前目录或子目录中。")
    else:
        get_logger('launch_logger').warn(f"找到插件文件: {plugin_path}")
        plugin_dir = os.path.dirname(plugin_path)

    # --- 关键新增：设置 Gazebo 插件搜索路径 ---
    set_plugin_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        value=[plugin_dir]
    )



    # 使用 ROS 2 日志记录器
    logger = get_logger('launch_logger')
    logger.info(f'Loading Gazebo simulation from: {os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")}')


    # 3. 配置 ROS 2 与 Gazebo 的桥接节点
    bridge_config_path = os.path.join(
        get_package_share_directory('under_water_robot'),
        'config',
        'bridge_config.yaml'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config_path,
        }],
        output='screen'
    )


    thrust_control_node = Node(
        package='thrust_controler_server',
        executable='thrust_controler_server',
        output='screen'
    )

   

    return LaunchDescription([
        set_plugin_path,
        bridge,
        thrust_control_node,
        gz_sim,
    ])