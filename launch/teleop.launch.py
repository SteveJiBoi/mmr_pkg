"""Desktop side of hardware teleop: RViz only. Runs ON THE PC.

    ros2 launch mmr_pkg teleop.launch.py

Pair it with robot.launch.py on the Pi. The PC needs nothing else running --
/robot_description arrives over DDS from the Pi's robot_state_publisher and
RViz's RobotModel display reads it straight off the topic.

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
