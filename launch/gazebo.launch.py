"""Gazebo Harmonic simulation of mmr_bot (Phase 2).

    ros2 launch mmr_pkg gazebo.launch.py
    ros2 launch mmr_pkg gazebo.launch.py headless:=true
    ros2 launch mmr_pkg gazebo.launch.py world:=/path/to/other.sdf

Then drive it:

    ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \\
        '{linear: {x: 0.2}, angular: {z: 0.0}}'

WHAT TALKS TO WHAT
------------------
    /cmd_vel  --> kiwi_drive_node --> /wheel_velocity_controller/commands
                                        |
                                        v
                        (ros2_control, inside Gazebo via gz_ros2_control)
                                        |
                                        v
                  wheel_0/1/2_joint velocity interfaces --> physics

    joint_state_broadcaster --> /joint_states --> robot_state_publisher --> /tf

Launch-file names verified against ros_gz `jazzy` branch (which the repo
README pairs with Gazebo Harmonic): the simulator launch file really is
`gz_sim.launch.py` in package `ros_gz_sim` with argument `gz_args`, and
`create` really does take `-topic`, `-name`, `-x/-y/-z`. Do not "modernise"
these to the `ign_` spellings; those are the deprecated aliases.
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("mmr_pkg")

    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")
    spawn_z = LaunchConfiguration("spawn_z")
    use_rviz = LaunchConfiguration("rviz")

    model = PathJoinSubstitution([pkg, "urdf", "mmr_bot.urdf.xacro"])

    # gazebo:=true pulls in ros2_control.xacro + gazebo.xacro. value_type=str
    # is required or the launch system tries to infer a type from the URDF
    # text and mangles it (same trap as display.launch.py).
    robot_description = ParameterValue(
        Command(["xacro ", model, " gazebo:=true"]), value_type=str)

    # Everything downstream of the simulator must run on /clock, or TF
    # timestamps will not line up with sensor data and RViz will show nothing.
    sim_time = {"use_sim_time": True}

    args = [
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution([pkg, "worlds", "mmr_world.sdf"]),
            description="Absolute path to the .sdf world"),
        DeclareLaunchArgument(
            "headless", default_value="false",
            description="Run gz-sim without the GUI (server only)"),
        DeclareLaunchArgument(
            # base_footprint sits exactly on the wheel contact plane, so 0.0 is
            # the geometrically correct drop height. Exposed because if you
            # ever see contact jitter on the first frame, lifting the robot a
            # few millimetres is the usual fix. It is NOT a robot dimension.
            "spawn_z", default_value="0.0",
            description="Height to spawn base_footprint at, metres"),
        DeclareLaunchArgument(
            "rviz", default_value="false",
            description="Also open RViz with the Phase 1 config"),
    ]

    # -- simulator ---------------------------------------------------------
    # -r starts the world unpaused. -v 4 is verbose; drop to -v 1 once things
    # work, but while bringing a new world up the plugin-load errors that
    # appear at -v 4 are exactly the ones you need to see.
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])),
        launch_arguments={"gz_args": [world, " -r -v 4"]}.items(),
        condition=UnlessCondition(headless),
    )
    gz_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])),
        launch_arguments={"gz_args": [world, " -r -v 4 -s --headless-rendering"]}.items(),
        condition=IfCondition(headless),
    )

    # -- description -------------------------------------------------------
    # Must start before `create`, because create reads the URDF off the
    # /robot_description topic that this node latches.
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}, sim_time],
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "/robot_description",
                   "-name", "mmr_bot",
                   "-z", spawn_z],
        parameters=[sim_time],
    )

    # -- gz <-> ros bridge -------------------------------------------------
    # Direction syntax: `[` is gz->ros, `]` is ros->gz, `@` is bidirectional.
    # /clock must be gz->ros or nothing downstream gets simulated time.
    # /scan's gz-side name is pinned by <topic>scan</topic> in gazebo.xacro,
    # so this does not depend on gz's scoped-name derivation.
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ],
        parameters=[sim_time],
    )

    # -- ros2_control ------------------------------------------------------
    # The controller_manager is created by the gz_ros2_control plugin when the
    # model spawns, so these spawners cannot run until `create` has exited.
    def spawner(name):
        return Node(package="controller_manager", executable="spawner",
                    arguments=[name], output="screen", parameters=[sim_time])

    controllers = RegisterEventHandler(OnProcessExit(
        target_action=spawn,
        on_exit=[spawner("joint_state_broadcaster"),
                 spawner("wheel_velocity_controller"),
                 spawner("arm_position_controller")],
    ))

    # -- our node ----------------------------------------------------------
    # publish_odom is false: Gazebo already knows ground truth, and wheel
    # odometry from a slipping omniwheel is the least trustworthy thing in the
    # system. Turn it on when you want to see how bad the drift is.
    kiwi = Node(
        package="mmr_pkg",
        executable="kiwi_drive_node",
        output="screen",
        parameters=[sim_time, {
            "publish_odom": False,
            "publish_tf": False,
        }],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", PathJoinSubstitution([pkg, "rviz", "display.rviz"])],
        parameters=[sim_time],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        args + [gz, gz_headless, rsp, spawn, bridge, controllers, kiwi, rviz])
