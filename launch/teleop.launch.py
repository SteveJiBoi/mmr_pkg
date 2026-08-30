"""Desktop side of hardware teleop: RViz only. Runs ON THE PC.

    ros2 launch mmr_pkg teleop.launch.py

Pair it with robot.launch.py on the Pi. The PC needs nothing else running --
/robot_description arrives over DDS from the Pi's robot_state_publisher and
RViz's RobotModel display reads it straight off the topic.

THE KEYBOARD IS NOT STARTED HERE, DELIBERATELY
----------------------------------------------
teleop_twist_keyboard reads raw keypresses from stdin. ros2 launch does not
give its child processes a usable terminal, so launching it from here produces
a node that starts, prints its help text and then never registers a single
keystroke -- which looks exactly like a broken robot. Run it yourself, in its
own terminal:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard

That window must keep FOCUS for the keys to register.

    u i o        hold SHIFT for the same layout in holonomic mode,
    j k l        where J and L STRAFE left/right instead of turning.
    m , .        This robot is holonomic, so the shifted layout is
                 the one you actually want.

    k  stop          q/z  faster/slower (both)
    i  forward       w/x  faster/slower (linear only)
    ,  back          e/c  faster/slower (angular only)

    unshifted:  j/l  = TURN left/right
    SHIFTED:    J/L  = STRAFE left/right,  U/O/M/> = diagonals

Start slow. It defaults to 0.5 m/s and 1.0 rad/s, and with the bridge's default
normalisation that is about half PWM -- brisk for an indoor omni base.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("mmr_pkg")
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription([
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution(
                [pkg, "rviz", "teleop.rviz"]),
            description="RViz config with RobotModel + LaserScan + Image"),

        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
        ),
    ])
