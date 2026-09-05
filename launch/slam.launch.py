"""2D SLAM for mmr_bot. Runs ON THE DESKTOP, against a robot already up.

    ros2 launch mmr_pkg robot.launch.py          # on the Pi, first
    ros2 launch mmr_pkg slam.launch.py           # here

Starts:

    rf2o_laser_odometry      /scan -> odom -> base_footprint      (see below)
    slam_toolbox (async)     /scan + odom -> /map, map -> odom

Save the map when the room looks right:

    ros2 run nav2_map_server map_saver_cli -f ~/my_map

WHY THIS RUNS ON THE DESKTOP
----------------------------
/scan is about 20 kB/s -- 500 points at 10 Hz -- so shipping it over WiFi is
cheap, while slam_toolbox's scan matching and loop closure are not cheap on a
Pi 5 that is already running the lidar driver, the camera and the bridge.
Moving the expensive half to the desktop costs almost no bandwidth. Nothing
here has to run on the Pi: no node in this file talks to hardware.

THE ODOMETRY PROBLEM, WHICH IS THE WHOLE STORY
----------------------------------------------
slam_toolbox does not invent odometry. It needs a TF odom -> base_footprint to
already exist and refines that estimate by matching scans. This robot has no
encoders, so nothing publishes odom -- and a robot with no odom source cannot
run slam_toolbox at all, regardless of how good the lidar is.

The honest fix is to derive odometry from the one sensor that does measure
motion: the lidar. rf2o_laser_odometry estimates planar motion from the range
flow between consecutive scans. That is a real measurement, in real metres,
which is why it is used here instead of integrating /cmd_vel -- integrating
the command would produce a number that looks like odometry but only reports
what we ASKED the robot to do. On a robot with no encoders and 7.7% packet
loss, those are very different things.

What rf2o costs you, stated plainly:

  * It fails in featureless spaces. A long blank corridor or the middle of an
    empty hall gives scan matching nothing to lock onto and the estimate
    slides. Encoders would not care.
  * It is not independent of slam_toolbox's own scan matching, so the two
    agreeing does not mean they are right.
  * At 10 Hz -- the C1's scan rate -- fast rotation between scans is where it
    degrades first. Turn slowly while mapping.

rf2o_laser_odometry is a WORKSPACE CHECKOUT, not a rosdep key, exactly like
sllidar_ros2. Clone MAPIRlab/rf2o_laser_odometry (branch ros2) next to this
package and build it. It is deliberately absent from package.xml: naming it
there makes `rosdep install` abort before it resolves anything else.

    odom_source:=none    if you have your own odom -> base_footprint

is there for the day this robot gets encoders. It is not a way to run without
odometry; with nothing publishing odom, slam_toolbox will sit and log
transform timeouts.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("mmr_pkg")

    scan_topic = LaunchConfiguration("scan_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")

    args = [
        DeclareLaunchArgument(
            "slam_params",
            default_value=PathJoinSubstitution(
                [pkg, "config", "slam_toolbox.yaml"]),
            description="YAML of slam_toolbox mapper parameters"),
        DeclareLaunchArgument(
            "scan_topic", default_value="/scan",
            description="LaserScan topic from the RPLIDAR C1"),
        DeclareLaunchArgument(
            "odom_source", default_value="rf2o",
            description="rf2o = derive odometry from the lidar; "
                        "none = something else already publishes "
                        "odom -> base_footprint"),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="true when mapping inside Gazebo"),
    ]

    # freq is 10.0, not rf2o's own default of 20.0. The C1 produces 10 scans a
    # second; asking for 20 does not create information, it just re-runs the
    # estimator on scans it has already used and burns CPU doing it.
    #
    # base_frame_id is base_footprint to match slam_toolbox's base_frame. If
    # these two disagree the TF tree forks and RViz shows the robot in two
    # places at once.
    odom = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        output="screen",
        parameters=[{
            "laser_scan_topic": scan_topic,
            "odom_topic": "/odom",
            "publish_tf": True,
            "base_frame_id": "base_footprint",
            "odom_frame_id": "odom",
            "init_pose_from_topic": "",
            "freq": 10.0,
            "use_sim_time": use_sim_time,
        }],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("odom_source"),
                              "' == 'rf2o'"])),
    )

    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        # async, not sync: sync blocks until every scan is processed, which on
        # a live robot means falling progressively further behind rather than
        # dropping a scan and staying current.
        parameters=[
            LaunchConfiguration("slam_params"),
            {"use_sim_time": use_sim_time,
             "scan_topic": scan_topic},
        ],
    )

    return LaunchDescription(args + [odom, slam])
