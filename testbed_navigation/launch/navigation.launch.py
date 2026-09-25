import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('testbed_navigation')
    params = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Order is imp here
    lifecycle_nodes = ['controller_server', 'planner_server',
                       'behavior_server', 'bt_navigator']

    common = [params, {'use_sim_time': use_sim_time}]

    return LaunchDescription([
        DeclareLaunchArgument('params_file',
            default_value=os.path.join(pkg, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(package='nav2_controller', executable='controller_server', name='controller_server', output='screen', parameters=common),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen', parameters=common),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen', parameters=common),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen', parameters=common),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation', output='screen', parameters=[{'use_sim_time': use_sim_time, 'autostart': True, 'node_names': lifecycle_nodes}]),
    ])