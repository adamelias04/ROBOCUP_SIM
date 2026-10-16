# ROS 2 side of the RoboCup Isaac Sim simulation: robot description, the
# lidar relay, RViz and a steering GUI, optionally Nav2 + slam_toolbox.
# Started by ../../../run.sh once Isaac publishes /clock; by hand:
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
    # Isaac publishes no /joint_states; zeros keep the wheel TFs alive.
    joint_state_publisher = Node(
        package='joint_state_publisher', executable='joint_state_publisher',
        output='screen', parameters=[use_sim_time],
    )
    # Isaac's RTX lidar reports no-hit beams as 0.0; give consumers inf.
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
    # Own rqt settings dir: the shared ~/.config/ros.org/rqt_gui.ini remembers
    # the topic last used in another workspace (e.g. /shoddy/cmd_vel) and that
    # overrides default_topic.
    steering = Node(
        package='rqt_robot_steering', executable='rqt_robot_steering', output='screen',
        parameters=[{'default_topic': '/cmd_vel'}],
        additional_env={'XDG_CONFIG_HOME': os.path.expanduser('~/.config/robocup_ws')},
        condition=IfCondition(LaunchConfiguration('steering')),
    )

    # Navigation (nav:=true): Nav2 in `odom`, plus slam_toolbox, which only
    # provides map->odom and /map for RViz.
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
        # Isaac's ROS 2 bridge speaks Fast DDS, so pin it instead of inheriting
        # a shell set up for the real robot (e.g. rmw_zenoh_cpp).
        DeclareLaunchArgument('rmw', default_value='rmw_fastrtps_cpp',
                              description='RMW, must match the Isaac Sim bridge'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', LaunchConfiguration('rmw')),
        SetEnvironmentVariable('ZENOH_CONFIG_OVERRIDE', ''),
        # PRIME offload so RViz renders on the NVIDIA GPU.
        SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('steering', default_value='true',
                              description='rqt_robot_steering GUI publishing /cmd_vel'),
        DeclareLaunchArgument('nav', default_value='false',
                              description='Run Nav2 (+ slam_toolbox); goals via RViz 2D Goal Pose'),
        DeclareLaunchArgument('planner', default_value='teb',
                              description='Nav2 local planner: dwb | teb | tr'),
        DeclareLaunchArgument('slam', default_value='true',
                              description='With nav:=true, also run slam_toolbox (map for RViz)'),
        robot_state_publisher,
        joint_state_publisher,
        scan_relay,
        rviz,
        steering,
        nav2,
        slam,
    ])
