"""Desktop side of hardware teleop: RViz only. Runs ON THE PC.

    ros2 launch mmr_pkg teleop.launch.py                 # drive, no map
    ros2 launch mmr_pkg teleop.launch.py slam:=true      # drive AND build a map

Pair it with robot.launch.py on the Pi. The PC needs nothing else running --
/robot_description arrives over DDS from the Pi's robot_state_publisher and
RViz's RobotModel display reads it straight off the topic.

SEEING THE MAP WHILE YOU DRIVE
------------------------------
slam:=true starts slam.launch.py here as well (rf2o + slam_toolbox) and swaps
the RViz config from teleop.rviz to mapping.rviz, which adds a Map display and
anchors Fixed Frame to `map` so the map holds still and the robot moves
through it. Two terminals on the desktop, total:

    ros2 launch mmr_pkg teleop.launch.py slam:=true
    ros2 run mmr_pkg kb_teleop

slam.launch.py's own arguments come along with it, so scan_topic:=,
odom_source:=, use_sim_time:= and slam_params:= all work on this command line
too; `--show-args` lists them.

If you are already running slam.launch.py in its own terminal -- which is what
docs/slam-and-nav2.md describes, and is still the better place for it when you
want to watch its output -- do NOT also pass slam:=true, or two slam_toolbox
nodes will fight over map -> odom. Ask for the map view alone instead:

    ros2 launch mmr_pkg teleop.launch.py show_map:=true

Save the map when the room looks right:

    ros2 run nav2_map_server map_saver_cli -f ~/my_map

Turn slowly. rf2o derives odometry by matching consecutive scans and the C1
only produces 10 a second; spin fast and there is too little overlap between
scans to match, which is the main way a map comes out smeared.

THE KEYBOARD IS NOT STARTED HERE, DELIBERATELY
----------------------------------------------
Keyboard teleop reads raw keypresses from stdin. ros2 launch does not give its
child processes a controlling terminal, so launching it from here produces a
node that starts, looks healthy, publishes a steady stream of zeros and never
registers a single keystroke -- which is indistinguishable from a broken robot.
mmr_pkg's own node refuses to start in that situation rather than pretend. Run
it yourself, in its own terminal:

    ros2 run mmr_pkg kb_teleop

That window must keep FOCUS for the keys to register.

         q   w   e        w / x   forward / back
           a   d          a / d   strafe left / right
             x            q / e   turn left / right

    SPACE  stop now       + / -   faster / slower
    k      quit           [ / ]   turn slower / faster

Start slow. It defaults to 0.5 m/s and 1.0 rad/s, and with the bridge's default
normalisation that is about half PWM -- brisk for an indoor omni base.

If you want a second opinion on whether a problem is the keyboard node or the
bridge, teleop_twist_keyboard publishes the same /cmd_vel and is installed:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard

Hold SHIFT with that one. This robot is holonomic, and only its shifted layout
strafes; unshifted J/L turn instead.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("mmr_pkg")

    slam = LaunchConfiguration("slam")
    show_map = LaunchConfiguration("show_map")

    # Which RViz config to open, decided at launch time from the two flags.
    # slam:=true implies the map view -- starting SLAM and then not showing
    # its output would be a strange thing to ask for -- so either flag selects
    # mapping.rviz. Anyone passing rviz_config explicitly overrides all of it.
    rviz_file = PythonExpression([
        "'mapping.rviz' if ('", slam, "' == 'true' or '", show_map,
        "' == 'true') else 'teleop.rviz'",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "slam", default_value="false",
            description="Also start slam.launch.py (rf2o + slam_toolbox) "
                        "here, and show the map. Leave false if you are "
                        "already running slam.launch.py separately"),
        DeclareLaunchArgument(
            "show_map", default_value="false",
            description="Use the mapping RViz config without starting SLAM, "
                        "for when slam.launch.py runs in its own terminal"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([pkg, "rviz", rviz_file]),
            description="RViz config. Defaults to teleop.rviz, or "
                        "mapping.rviz when slam:= or show_map:= is true"),

        # launch_arguments is deliberately not passed, and that does NOT
        # mean slam.launch.py's arguments are unreachable from here. An
        # IncludeLaunchDescription shares the parent's launch context, so
        # slam_params, scan_topic, odom_source and use_sim_time all appear in
        # `ros2 launch mmr_pkg teleop.launch.py --show-args` and are settable
        # on this command line. Re-listing them here would only create a
        # second place for their defaults to drift out of step.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg, "launch", "slam.launch.py"])),
            condition=IfCondition(slam),
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
        ),
    ])
