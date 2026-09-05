"""Nav2 for mmr_bot. Runs ON THE DESKTOP, on top of a robot that is already up.

Two ways to use it, and they differ only in where the map comes from.

    # 1. Build a map and navigate on it at the same time
    ros2 launch mmr_pkg robot.launch.py            # on the Pi
    ros2 launch mmr_pkg slam.launch.py             # here: odom + slam_toolbox
    ros2 launch mmr_pkg nav2.launch.py             # here

    # 2. Navigate a map you saved earlier
    ros2 launch mmr_pkg nav2.launch.py map:=$HOME/my_map.yaml

Then give it a goal: RViz's "2D Goal Pose" button, or

    ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \\
        '{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}}}'

WHAT THIS FILE IS
-----------------
A thin wrapper. It includes nav2_bringup's own navigation_launch.py rather
than starting the ten Nav2 servers by hand, because that list changes between
distros and a hand-rolled copy silently rots. All this file does is point Nav2
at config/nav2.yaml and, optionally, at a saved map.

WHO PUBLISHES map -> odom
-------------------------
Exactly one thing must, and it is an either/or:

    map:=""   (default)  slam_toolbox, from slam.launch.py. Mapping while
                         navigating. THE PREREQUISITE IS THAT slam.launch.py IS
                         ALREADY RUNNING -- Nav2 will come up either way and
                         then sit reporting that it cannot transform into map.
    map:=<file>          map_server + amcl, from nav2_bringup's
                         localization_launch.py. slam_toolbox must NOT be
                         running: two publishers of map -> odom fight, and the
                         robot jumps between their two estimates.

Neither case removes the need for odom -> base_footprint. Nav2 does not
produce odometry either; slam.launch.py's rf2o_laser_odometry is what supplies
it on this robot, and that is required in BOTH modes. If you navigate a saved
map, run slam.launch.py with odom_source:=rf2o and slam disabled, or run rf2o
yourself. See docs/slam-and-nav2.md.

WHY THE PARAMS FILE IS NOT OPTIONAL HERE
----------------------------------------
Nav2's stock parameters describe a differential-drive robot. Three of its
defaults do not error on a kiwi drive, they just quietly delete sideways
motion -- see the header of config/nav2.yaml. Launching Nav2 with upstream
defaults on this robot produces something that drives, turns, and never
strafes, which is a very slow bug to find.
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("mmr_pkg")
    nav2 = FindPackageShare("nav2_bringup")

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    map_yaml = LaunchConfiguration("map")

    # "is the map argument empty?" -- decided here, once, so the two mutually
    # exclusive localization sources below can never both start.
    using_saved_map = PythonExpression(["'", map_yaml, "' != ''"])

    args = [
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([pkg, "config", "nav2.yaml"]),
            description="Nav2 parameters. The default is tuned for this "
                        "robot's holonomic base; nav2_bringup's own file is "
                        "not a safe substitute"),
        DeclareLaunchArgument(
            "map", default_value="",
            description="Full path to a saved map .yaml. Empty means "
                        "slam_toolbox is providing map -> odom live"),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="true when navigating inside Gazebo"),
        DeclareLaunchArgument(
            "autostart", default_value="true",
            description="Bring the Nav2 lifecycle nodes straight to active"),
    ]

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [nav2, "launch", "navigation_launch.py"])),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": autostart,
        }.items(),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [nav2, "launch", "localization_launch.py"])),
        launch_arguments={
            "map": map_yaml,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": autostart,
        }.items(),
        condition=IfCondition(using_saved_map),
    )

    # Not a failure, just the thing people get wrong first. Nav2 activating
    # cleanly and then refusing every goal looks like a Nav2 problem; it is
    # almost always this.
    reminder = LogInfo(
        msg=("nav2: map:='' so NOTHING in this launch publishes map -> odom. "
             "slam_toolbox must already be running (ros2 launch mmr_pkg "
             "slam.launch.py), or goals will be rejected as untransformable."),
        condition=UnlessCondition(using_saved_map),
    )

    return LaunchDescription(args + [navigation, localization, reminder])
