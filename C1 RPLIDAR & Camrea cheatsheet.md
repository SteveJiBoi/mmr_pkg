# Raspberry Pi + ROS 2 Jazzy
# RPLIDAR C1 + USB Camera Cheat Sheet

## SYSTEM

OS: Ubuntu 24.04
ROS: ROS 2 Jazzy
Workspace: ~/ros2_ws

---

# 1. SSH INTO RASPBERRY PI

From PC:

ssh steveji@10.229.5.237

Check Pi IP:

hostname -I

---

# 2. SOURCE ROS WORKSPACE

Run this in every new terminal:

source ~/ros2_ws/install/setup.bash

---

# 3. RPLIDAR C1

## Check USB

lsusb

Expected:

Silicon Labs CP2102N USB to UART Bridge Controller

Check serial port:

ls -l /dev/ttyUSB0

Check permissions:

groups

Make sure "dialout" is listed.

---

## Start C1

source ~/ros2_ws/install/setup.bash

ros2 launch sllidar_ros2 sllidar_c1_launch.py

IMPORTANT:
Use sllidar_ros2 for the C1.

DO NOT use:

ros2 run rplidar_ros rplidar_composition

The old rplidar_ros package does not support the C1 properly.

---

# 4. CHECK RPLIDAR SCAN

Open another terminal:

source ~/ros2_ws/install/setup.bash

ros2 topic list

Look for:

/scan

Check scan rate:

ros2 topic hz /scan

Expected:

~10 Hz

Check scan data:

ros2 topic echo /scan --once

---

# 5. RPLIDAR MOTOR CONTROL

Check services:

ros2 service list | grep motor

Expected:

/start_motor
/stop_motor

Check service type:

ros2 service type /start_motor
ros2 service type /stop_motor

Expected:

std_srvs/srv/Empty

Start motor:

ros2 service call /start_motor std_srvs/srv/Empty

Stop motor:

ros2 service call /stop_motor std_srvs/srv/Empty

To completely stop the driver:

Ctrl+C

in the terminal running the C1 launch.

---

# 6. RPLIDAR + RVIZ

Start C1 + RViz:

source ~/ros2_ws/install/setup.bash

ros2 launch sllidar_ros2 view_sllidar_c1_launch.py

This should start:

RPLIDAR C1
+
RViz2

The lidar data is:

/scan

---

# 7. USB CAMERA

## Check camera

lsusb

Expected:

icSpring icspring camera

Check video device:

ls -l /dev/video*

---

# 8. START CAMERA

Start camera:

source /opt/ros/jazzy/setup.bash

ros2 run v4l2_camera v4l2_camera_node

The camera currently works with:

Driver: uvcvideo
Resolution: 640x480
Format: YUYV

The calibration warning can be ignored for basic viewing.

---

# 9. CHECK CAMERA TOPICS

Open another terminal:

source /opt/ros/jazzy/setup.bash

ros2 topic list | grep image

Expected:

/image_raw
/camera_info

Check camera FPS:

ros2 topic hz /image_raw

Check image data:

ros2 topic echo /camera_info --once

---

# 10. VIEW CAMERA IN RVIZ

Start RViz:

rviz2

In RViz:

Add
→ By topic
→ /image_raw
→ Image

The camera image should appear.

---

# 11. USE RPLIDAR + CAMERA TOGETHER

Terminal 1:

source ~/ros2_ws/install/setup.bash
ros2 launch sllidar_ros2 sllidar_c1_launch.py

Terminal 2:

source /opt/ros/jazzy/setup.bash
ros2 run v4l2_camera v4l2_camera_node

Terminal 3:

rviz2

ROS topics:

RPLIDAR:
 /scan

Camera:
 /image_raw
 /camera_info

---

# 12. USEFUL TROUBLESHOOTING

Check ROS nodes:

ros2 node list

Check C1:

ros2 node info /sllidar_node

Check scan:

ros2 topic info /scan

Check camera:

ros2 topic info /image_raw

Check USB devices:

lsusb

Check serial port:

ls -l /dev/ttyUSB0

Check camera device:

ls -l /dev/video*

Check kernel messages:

sudo dmesg | tail -30

Watch USB events:

sudo dmesg -w

Check who is using the lidar:

sudo lsof /dev/ttyUSB0

---

# QUICK START

## C1

source ~/ros2_ws/install/setup.bash
ros2 launch sllidar_ros2 sllidar_c1_launch.py

## Camera

source /opt/ros/jazzy/setup.bash
ros2 run v4l2_camera v4l2_camera_node

## RViz

rviz2

## Check lidar

ros2 topic hz /scan

## Check camera

ros2 topic hz /image_raw


# CURRENT WORKING SETUP

Raspberry Pi
Ubuntu 24.04
ROS 2 Jazzy
        |
        +--- RPLIDAR C1
        |       |
        |      /scan
        |
        +--- USB Camera
                |
             /image_raw

Both can be visualized in RViz2.