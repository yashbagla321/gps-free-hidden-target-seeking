"""GPS-free seeking, software-in-the-loop through modern Gazebo.

One launch = one seeded run. The evaluator requests launch shutdown at mission
end; the batch runner then verifies ROS-node exit and removes any Gazebo server
orphaned by the vendor wrapper before admitting the result:

    ros2 launch gps_free_seeking_gz gfs_seeking_gz.launch.py \
        seed:=7 output_name:=gz_seed7.csv dropout:=0.3 delay_s:=0.2
"""

import os
import time

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, Shutdown)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("gps_free_seeking_gz")
    gz = FindPackageShare("ros_gz_sim")
    args = [
        DeclareLaunchArgument("world", default_value=PathJoinSubstitution(
            [pkg, "worlds", "gfs_hidden_target.sdf"])),
        DeclareLaunchArgument("config", default_value=PathJoinSubstitution(
            [pkg, "config", "gfs_seeking_gz.yaml"])),
        DeclareLaunchArgument("seed", default_value="12"),
        DeclareLaunchArgument("mission_s", default_value="90.0"),
        DeclareLaunchArgument("dropout", default_value="0.0"),
        DeclareLaunchArgument("delay_s", default_value="0.0"),
        DeclareLaunchArgument("delay_jitter_s", default_value="0.0"),
        DeclareLaunchArgument("outlier_prob", default_value="0.0"),
        DeclareLaunchArgument("yaw_step_time", default_value="-1.0"),
        DeclareLaunchArgument("yaw_step_deg", default_value="0.0"),
        DeclareLaunchArgument("target_step_time", default_value="-1.0"),
        DeclareLaunchArgument("target_step_dx", default_value="0.0"),
        DeclareLaunchArgument("target_step_dy", default_value="0.0"),
        DeclareLaunchArgument("station_window_s", default_value="30.0"),
        DeclareLaunchArgument("scenario", default_value="manual"),
        DeclareLaunchArgument("fail_if_output_exists", default_value="true"),
        DeclareLaunchArgument("output_dir",
                              default_value="results/campaign2027/ros_gz"),
        DeclareLaunchArgument("output_name", default_value="gz_run.csv"),
        # Empty by default (no effect on citable runs); set to
        # "--headless-rendering" to enable the Sensors system's render
        # thread for worlds with a camera (e.g. gfs_hidden_target_video.sdf).
        DeclareLaunchArgument("extra_gz_args", default_value=""),
    ]
    cfg = LaunchConfiguration("config")
    as_float = lambda name: ParameterValue(
        LaunchConfiguration(name), value_type=float)
    as_int = lambda name: ParameterValue(
        LaunchConfiguration(name), value_type=int)
    as_bool = lambda name: ParameterValue(
        LaunchConfiguration(name), value_type=bool)
    # Unique transport partition per launch invocation: leaked gz servers
    # from earlier runs (common on WSL, where SIGTERM on the ruby wrapper
    # can orphan the server) can then never pollute this run's topics.
    partition = f"gfs_{os.getpid()}_{int(time.time())}"
    return LaunchDescription(args + [
        SetEnvironmentVariable("GZ_PARTITION", partition),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([gz, "launch", "gz_sim.launch.py"])),
            launch_arguments={"gz_args": ["-r -s ",
                                          LaunchConfiguration("extra_gz_args"),
                                          " ",
                                          LaunchConfiguration("world")]}.items(),
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gfs_bridge",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                "/model/gfs_vehicle/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                "/model/gfs_vehicle/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                "/gfs/truth_odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                # No-op unless the world has a "camera" sensor (video-capture
                # world only): bridges idle waiting for a topic that never
                # appears in the citable world.
                "/camera@sensor_msgs/msg/Image[gz.msgs.Image",
            ],
            output="screen",
            on_exit=Shutdown(),
        ),
        Node(
            package="gps_free_seeking_gz",
            executable="relay_emulator_node",
            parameters=[cfg, {
                "seed": as_int("seed"),
                "dropout": as_float("dropout"),
                "delay_s": as_float("delay_s"),
                "delay_jitter_s": as_float("delay_jitter_s"),
                "outlier_prob": as_float("outlier_prob"),
                "yaw_step_time": as_float("yaw_step_time"),
                "yaw_step_deg": as_float("yaw_step_deg"),
                "target_step_time": as_float("target_step_time"),
                "target_step_dx": as_float("target_step_dx"),
                "target_step_dy": as_float("target_step_dy"),
            }],
            output="screen",
            on_exit=Shutdown(),
        ),
        Node(
            package="gps_free_seeking_gz",
            executable="gfs_seeker_node",
            parameters=[cfg],
            remappings=[("/odom", "/model/gfs_vehicle/odometry"),
                        ("/cmd_vel", "/model/gfs_vehicle/cmd_vel")],
            output="screen",
            on_exit=Shutdown(),
        ),
        Node(
            package="gps_free_seeking_gz",
            executable="gz_evaluator_node",
            parameters=[cfg, {
                "mission_s": as_float("mission_s"),
                "station_window_s": as_float("station_window_s"),
                "seed": as_int("seed"),
                "scenario": LaunchConfiguration("scenario"),
                "output_dir": LaunchConfiguration("output_dir"),
                "output_name": LaunchConfiguration("output_name"),
                "fail_if_output_exists": as_bool("fail_if_output_exists"),
                "target_step_time": as_float("target_step_time"),
                "target_step_dx": as_float("target_step_dx"),
                "target_step_dy": as_float("target_step_dy"),
            }],
            output="screen",
            on_exit=Shutdown(),
        ),
    ])
