import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 路径配置
    config_pkg_prefix = get_package_share_directory('config_pkg')
    vins_estimator_prefix = get_package_share_directory('vins_estimator')
    
    default_config_path = os.path.join(config_pkg_prefix, 'config', 'euroc', 'euroc_config.yaml')

    # 声明参数
    config_arg = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Path to config'
    )
    vins_path_arg = DeclareLaunchArgument(
        'vins_path', default_value=vins_estimator_prefix,
        description='Path to vins_estimator'
    )
    
    # 强制开启 use_sim_time
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo/Rosbag) clock if true'
    )

    # ==========================================
    # 核心修复：前端节点
    # ==========================================
    feature_tracker_node = Node(
        package='feature_tracker',
        executable='feature_tracker',
        name='vins_feature_tracker',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_path'),
            'vins_folder': LaunchConfiguration('vins_path'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        remappings=[
            ('/feature', '/feature_tracker/feature')
        ]
    )

    # ==========================================
    # 后端节点
    # ==========================================
    estimator_node = Node(
        package='vins_estimator',
        executable='vins_estimator',
        name='vins_estimator',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_path'),
            'vins_folder': LaunchConfiguration('vins_path'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
        # Estimator 默认就是听 /feature_tracker/feature，所以不需要改它，改发件人就行
    )

    return LaunchDescription([
        config_arg,
        vins_path_arg,
        use_sim_time_arg,
        LogInfo(msg=['[VINS] Starting with Topic Remap: /feature -> /feature_tracker/feature']),
        feature_tracker_node,
        estimator_node
    ])