"""RViz display for mmr_bot (Phase 1).

    ros2 launch mmr_pkg display.launch.py
    ros2 launch mmr_pkg display.launch.py gui:=false   # no slider window
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("mmr_pkg")

    model = LaunchConfiguration("model")
    rviz_config = LaunchConfiguration("rviz_config")
    gui = LaunchConfiguration("gui")

    # ParameterValue(..., value_type=str) is required: without it the launch
    # system tries to infer a type from the URDF text and mangles it.
    #
    # gazebo:=false keeps this a pure Phase 1 description - no <gazebo> or
    # <ros2_control> blocks at all. robot_state_publisher would ignore them
    # anyway, but there is no reason to hand RViz a URDF full of simulation
    # tags, and it keeps this launch file working if gz_ros2_control is not
    # even installed.
    robot_description = ParameterValue(
        Command(["xacro ", model, " gazebo:=false"]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            "model",
            default_value=PathJoinSubstitution([pkg, "urdf", "mmr_bot.urdf.xacro"]),
            description="Absolute path to the robot xacro"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([pkg, "rviz", "display.rviz"]),
            description="Absolute path to the RViz config"),
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="Start joint_state_publisher_gui (sliders)"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(gui),
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            condition=UnlessCondition(gui),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
        ),
    ])
