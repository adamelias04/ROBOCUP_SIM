# Nav2 pre shoddy v arene (podla nav2.launch.py z Gazebo verzie MRVK):
# controller, planner, behavior, bt_navigator, waypoint_follower a lifecycle
# manager, parametre podla zvoleneho plannera. Naviguje sa v `odom`, controller
# posiela /cmd_vel priamo do Isaacu. Spusta ho robocup.launch.py nav:=true.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PLANNER_FILES = {
    'dwb': 'nav2_params_dwb.yaml',
    'teb': 'nav2_params_teb.yaml',
    'tr': 'nav2_params_tr.yaml',
}


def launch_setup(context, *args, **kwargs):
    planner = LaunchConfiguration('planner').perform(context)
    if planner not in PLANNER_FILES:
        raise RuntimeError(f"planner:={planner}, expected one of {', '.join(PLANNER_FILES)}")
    params_file = os.path.join(get_package_share_directory('robocup_sim'),
                               'config', 'nav2', PLANNER_FILES[planner])
    cmd_vel_out = LaunchConfiguration('cmd_vel_out').perform(context)
    params = [params_file, {'use_sim_time': True}]

    controller = Node(
        package='nav2_controller', executable='controller_server', output='screen',
        parameters=params, remappings=[('cmd_vel', cmd_vel_out)],
    )
    planner_server = Node(
        package='nav2_planner', executable='planner_server', name='planner_server',
        output='screen', parameters=params,
    )
    behavior = Node(
        package='nav2_behaviors', executable='behavior_server', name='behavior_server',
        output='screen', parameters=params, remappings=[('cmd_vel', cmd_vel_out)],
    )
    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
        output='screen', parameters=params,
    )
    waypoint_follower = Node(
        package='nav2_waypoint_follower', executable='waypoint_follower',
        name='waypoint_follower', output='screen', parameters=params,
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['controller_server', 'planner_server', 'behavior_server',
                           'bt_navigator', 'waypoint_follower'],
        }],
    )
    return [controller, planner_server, behavior, bt_navigator,
            waypoint_follower, lifecycle_manager]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('planner', default_value='teb',
                              description='Lokalny planner: dwb | teb | tr'),
        DeclareLaunchArgument('cmd_vel_out', default_value='/cmd_vel',
                              description='Tema, na ktoru controller a behavior posielaju cmd_vel'),
        OpaqueFunction(function=launch_setup),
    ])
