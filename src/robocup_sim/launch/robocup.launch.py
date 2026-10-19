# ROS strana simulacie: popis robota, relay lidaru, RViz, ovladanie cez rqt
# a volitelne Nav2 + slam_toolbox. Spusta ho ../../../run.sh, ked Isaac
# publikuje /clock. Rucne:
#   ros2 launch robocup_sim robocup.launch.py [nav:=true planner:=teb|dwb|tr]
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('robocup_sim')
    model = os.path.join(get_package_share_directory('robocup_description'),
                         'urdf', 'robot.urdf.xacro')
    use_sim_time = {'use_sim_time': True}

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', model]), 'use_sim_time': True}],
    )
    joint_state_publisher = Node(
        package='joint_state_publisher', executable='joint_state_publisher',
        output='screen', parameters=[use_sim_time],
    )
    scan_relay = Node(
        package='robocup_sim', executable='scan_no_hit_to_inf.py', output='screen',
        parameters=[use_sim_time],
        remappings=[('scan_in', '/scan_raw'), ('scan_out', '/scan')],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', os.path.join(pkg, 'config', 'robocup.rviz')],
        parameters=[use_sim_time],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    steering = Node(
        package='rqt_robot_steering', executable='rqt_robot_steering', output='screen',
        parameters=[{'default_topic': '/cmd_vel'}],
        additional_env={'XDG_CONFIG_HOME': os.path.expanduser('~/.config/robocup_ws')},
        condition=IfCondition(LaunchConfiguration('steering')),
    )

    nav = LaunchConfiguration('nav')
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'nav2.launch.py')),
        launch_arguments={'planner': LaunchConfiguration('planner'),
                          'cmd_vel_out': '/cmd_vel'}.items(),
        condition=IfCondition(nav),
    )
    slam = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', output='screen',
        parameters=[os.path.join(pkg, 'config', 'slam', 'slam_toolbox.yaml'), use_sim_time],
        condition=IfCondition(PythonExpression(
            ["'", nav, "' == 'true' and '", LaunchConfiguration('slam'), "' == 'true'"])),
    )

    return LaunchDescription([
        DeclareLaunchArgument('rmw', default_value='rmw_fastrtps_cpp',
                              description='RMW, musi sediet s bridgeom v Isaacu'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', LaunchConfiguration('rmw')),
        SetEnvironmentVariable('ZENOH_CONFIG_OVERRIDE', ''),
        SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('steering', default_value='true',
                              description='rqt_robot_steering, publikuje /cmd_vel'),
        DeclareLaunchArgument('nav', default_value='false',
                              description='Nav2 + slam_toolbox, ciel cez 2D Goal Pose v RViz'),
        DeclareLaunchArgument('planner', default_value='teb',
                              description='Lokalny planner Nav2: dwb | teb | tr'),
        DeclareLaunchArgument('slam', default_value='true',
                              description='Pri nav:=true aj slam_toolbox (mapa pre RViz)'),
        robot_state_publisher,
        joint_state_publisher,
        scan_relay,
        rviz,
        steering,
        nav2,
        slam,
    ])
