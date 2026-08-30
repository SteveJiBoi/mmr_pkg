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

Each of the last three can be turned off independently:

    lidar:=false   camera:=false   bridge:=false

The desktop side is launch/teleop.launch.py (RViz) plus `ros2 run mmr_pkg
kb_teleop` in a terminal of its own -- keyboard teleop cannot be launched,
because launch gives its children no controlling terminal and the node would
read nothing while looking perfectly healthy.

ONE FAILURE DOES NOT TAKE THE REST DOWN. No node here has on_exit=Shutdown, so
an unplugged camera stops the camera and nothing else; the lidar and the drive
bridge carry on. That is the S14 requirement and it is the default behaviour --
worth stating because adding a single Shutdown handler would quietly undo it.

Bridge tuning lives in config/esp32_bridge.yaml, not in this file, so there is
one place to change a number. esp32_ip is the exception: it belongs on the
command line because it is per-robot and follows the DHCP lease.

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
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
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

        # ------------------------------------------------------------ drive
        # Taken from esp32/controller.py, which is this robot's own known-good
        # address -- not a guess. The ESP32 prints it on the serial console at
        # boot ("Ready. IP: ..."). It is a DHCP lease, so pin it in the router
        # or expect it to move.
        DeclareLaunchArgument(
            "esp32_ip", default_value="10.229.5.249",
            description="IPv4 address of the ESP32 running the omni firmware"),
        DeclareLaunchArgument(
            "bridge_params",
            default_value=PathJoinSubstitution(
                [pkg, "config", "esp32_bridge.yaml"]),
            description="YAML of bridge tuning parameters"),

        # ------------------------------------------------------------ lidar
        DeclareLaunchArgument(
            "serial_port", default_value="/dev/ttyUSB0",
            description="RPLIDAR C1 serial device"),

        # ----------------------------------------------------------- camera
        DeclareLaunchArgument(
            "video_device", default_value="/dev/video0",
            description="USB camera device node"),
        DeclareLaunchArgument(
            "pixel_format", default_value="YUYV",
            description="Camera pixel format; YUYV is what the C1 kit camera "
                        "reports via uvcvideo"),
        DeclareLaunchArgument(
            "image_width", default_value="640",
            description="Camera width in pixels"),
        DeclareLaunchArgument(
            "image_height", default_value="480",
            description="Camera height in pixels"),
        DeclareLaunchArgument(
            "camera_info_url", default_value="",
            description="file:// URL of an intrinsic calibration. Empty means "
                        "uncalibrated: the camera still streams, but "
                        "CameraInfo is all zeros"),

        # ------------------------------------------------------- on/off
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
    # runs at 460800 baud rather than the A1's 115200. Both are the included
    # launch file's own defaults, so they are deliberately NOT repeated here --
    # restating a working value is how it drifts.
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

    bridge = Node(
        package="mmr_pkg",
        executable="esp32_bridge",
        output="screen",
        # Order matters: the YAML sets the tuning, then the dict overrides the
        # one value that must come from the command line. Later wins.
        parameters=[
            LaunchConfiguration("bridge_params"),
            {"esp32_ip": esp32_ip},
        ],
        condition=IfCondition(LaunchConfiguration("bridge")),
    )

    return LaunchDescription(
        args + description
        + [lidar, OpaqueFunction(function=camera_setup), bridge])


def camera_setup(context, *_args, **_kwargs):
    """Builds the camera node once the launch arguments have real values.

    An OpaqueFunction rather than substitutions inline, for two reasons that
    both come down to needing an actual value rather than a promise of one:

      * image_size is a LIST OF INTEGERS. A LaunchConfiguration resolves to a
        string, and there is no clean way to spell "a list of two ints built
        from two separate string substitutions" in a parameter dict. Here they
        are just Python ints.
      * the calibration warning should appear only when there really is no
        calibration, which means comparing a resolved string to "".

    camera_frame_id is left at the driver default ("camera") ON PURPOSE. There
    is no camera_link in the URDF yet -- the camera is modelled as part of
    base_link's mesh (see urdf/sensors.xacro). RViz's *Image* display needs no
    TF at all, so the video feed works regardless. Only the *Camera* display
    needs a frame, and that one additionally needs a real intrinsic
    calibration, which this camera does not have. See README.
    """
    if LaunchConfiguration("camera").perform(context).lower() not in (
            "true", "1", "yes", "on"):
        return []

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    try:
        size = [int(arg("image_width")), int(arg("image_height"))]
    except ValueError:
        return [LogInfo(msg=(
            f"camera: image_width/image_height must be integers, got "
            f"{arg('image_width')!r} and {arg('image_height')!r}; "
            f"camera not started."))]

    info_url = arg("camera_info_url")
    actions = []
    if not info_url:
        # S11: say it plainly, once, and start anyway. An uncalibrated camera
        # is perfectly good for looking where you are driving; it is only
        # unusable for anything metric. Refusing to start would be worse than
        # useless, and saying nothing would leave the RViz *Camera* display
        # failing for no visible reason.
        actions.append(LogInfo(msg=(
            "camera: no camera_info_url, so /camera_info will be all zeros. "
            "The RViz *Image* display works regardless; the *Camera* display "
            "needs a real calibration and will not.")))

    actions.append(Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        output="screen",
        parameters=[{
            "video_device": arg("video_device"),
            "pixel_format": arg("pixel_format"),
            "image_size": size,
            "camera_info_url": info_url,
        }],
    ))
    return actions
