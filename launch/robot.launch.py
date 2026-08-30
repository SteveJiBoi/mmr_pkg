"""Real-hardware bringup for mmr_bot. Runs ON THE RASPBERRY PI.

    ros2 launch mmr_pkg robot.launch.py
    ros2 launch mmr_pkg robot.launch.py camera:=false          # lidar only
    ros2 launch mmr_pkg robot.launch.py esp32_ip:=10.229.5.249

Starts, on the robot:

    robot_state_publisher    URDF -> /tf, /robot_description
    joint_state_publisher    zeros, so the TF tree is complete (see below)
    sllidar_ros2             RPLIDAR C1 -> /scan
    v4l2_camera              USB camera -> /image_raw, /camera_info
    esp32_bridge             /cmd_vel   -> UDP to the ESP32

The desktop side (RViz + keyboard) is launch/teleop.launch.py.

This is Phase 1 geometry only: the xacro is expanded with gazebo:=false, so no
<gazebo> or <ros2_control> tags are emitted. On real hardware the ESP32 IS the
hardware interface -- there is no ros2_control hardware component for it, and
urdf/ros2_control.xacro deliberately emits a non-existent plugin name when
sim:=false so that anyone who wires it up by accident finds out immediately.

WHY joint_state_publisher WITH NO HARDWARE
------------------------------------------
robot_state_publisher can only publish TF for a joint if something tells it
that joint's position. Nothing does: the wheels have no encoders and the arm is
not driven yet. Without joint_state_publisher the TF tree stops dead at
base_link and RViz shows the chassis floating with no wheels and no laser_frame
-- so /scan has no transform and the whole display is empty.

Publishing zeros makes the tree complete and the geometry correct. The one
thing it is NOT is truthful about joint motion: the wheels will never appear to
turn. That is honest -- the robot genuinely does not know whether they are
turning. Do not "fix" it by faking wheel positions from cmd_vel; that invents
odometry the robot does not have.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("mmr_pkg")

    model = LaunchConfiguration("model")
    esp32_ip = LaunchConfiguration("esp32_ip")

    robot_description = ParameterValue(
        Command(["xacro ", model, " gazebo:=false"]), value_type=str)

    args = [
        DeclareLaunchArgument(
            "model",
            default_value=PathJoinSubstitution(
                [pkg, "urdf", "mmr_bot.urdf.xacro"]),
            description="Absolute path to the robot xacro"),

        # Taken from esp32/controller.py, which is this robot's own known-good
        # address -- not a guess. The ESP32 prints it on the serial console at
        # boot ("Ready. IP: ..."). It is a DHCP lease, so pin it in the router
        # or expect it to move.
        DeclareLaunchArgument(
            "esp32_ip", default_value="10.229.5.249",
            description="IPv4 address of the ESP32 running the omni firmware"),
        DeclareLaunchArgument(
            "esp32_port", default_value="1234",
            description="UDP port the ESP32 firmware listens on"),

        DeclareLaunchArgument(
            "serial_port", default_value="/dev/ttyUSB0",
            description="RPLIDAR C1 serial device"),
        DeclareLaunchArgument(
            "video_device", default_value="/dev/video0",
            description="USB camera device node"),

        DeclareLaunchArgument("lidar", default_value="true",
                              description="Start the RPLIDAR C1 driver"),
        DeclareLaunchArgument("camera", default_value="true",
                              description="Start the USB camera driver"),
        DeclareLaunchArgument("bridge", default_value="true",
                              description="Start the ESP32 UDP drive bridge"),
    ]

    description = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            output="screen",
        ),
    ]

    # sllidar_ros2, not rplidar_ros: the C1 needs the Slamtec driver, and it
    # runs at 460800 baud rather than the A1's 115200. Both are the launch
    # file's own defaults, so they are not repeated here.
    #
    # frame_id IS overridden. The driver defaults to "laser", but this URDF
    # calls the link "laser_frame", and a /scan stamped with a frame_id that is
    # not in the TF tree is invisible in RViz with only a warning to show for
    # it. Overriding the driver is better than adding a
    # static_transform_publisher to paper over the mismatch.
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("sllidar_ros2"), "launch",
             "sllidar_c1_launch.py"])),
        launch_arguments={
            "frame_id": "laser_frame",
            "serial_port": LaunchConfiguration("serial_port"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("lidar")),
    )

    # camera_frame_id is left at the driver default ("camera") ON PURPOSE.
    # There is no camera_link in the URDF yet -- the camera is modelled as part
    # of base_link's mesh (see urdf/sensors.xacro). RViz's *Image* display
    # needs no TF at all, so the video feed works regardless. Only the *Camera*
    # display needs a frame, and that one additionally needs a real intrinsic
    # calibration, which this camera does not have. See README.
    camera = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        output="screen",
        parameters=[{
            "video_device": LaunchConfiguration("video_device"),
            "pixel_format": "YUYV",
            "image_size": [640, 480],
        }],
        condition=IfCondition(LaunchConfiguration("camera")),
    )

    bridge = Node(
        package="mmr_pkg",
        executable="esp32_bridge",
        output="screen",
        parameters=[{
            "esp32_ip": esp32_ip,
            # value_type=int is required. A LaunchConfiguration substitutes to
            # a STRING, and the node declares esp32_port as an integer, so
            # without this the node dies at startup with a type mismatch.
            "esp32_port": ParameterValue(
                LaunchConfiguration("esp32_port"), value_type=int),
        }],
        condition=IfCondition(LaunchConfiguration("bridge")),
    )

    return LaunchDescription(args + description + [lidar, camera, bridge])
