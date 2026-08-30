cmake_minimum_required(VERSION 3.8)
project(mmr_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# --------------------------------------------------------------------------
# Python.
#
# This stays an ament_cmake package (it is description-first: meshes, urdf,
# launch and rviz all install as plain data). ament_cmake_python bolts the one
# Python library onto it, which is the documented way to ship both from a
# single package rather than splitting into mmr_pkg + mmr_pkg_py.
#
# ament_python_install_package installs mmr_pkg/ so that
# `from mmr_pkg.kiwi_kinematics import KiwiKinematics` works from anywhere.
# --------------------------------------------------------------------------
ament_python_install_package(${PROJECT_NAME})

# The node entry point. Installed to lib/${PROJECT_NAME} (NOT share/) because
# that is where `ros2 run` looks. scripts/kiwi_drive_node is a thin shim with
# a shebang; the logic lives in the installed package above.
install(PROGRAMS
  scripts/kiwi_drive_node
  scripts/esp32_bridge
  scripts/kb_teleop
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY urdf meshes launch rviz config worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests. All pure Python and needing no ROS graph -- kiwi_kinematics,
  # esp32_protocol, bridge_core and teleop_keys are deliberately free of ROS
  # imports -- so they run under `colcon test` on any machine, and under plain
  # `pytest` on one with no ROS at all.
  #
  # That is not a stylistic preference. The logic that decides when to stop the
  # robot lives in those modules SO THAT it can be executed by a test; when it
  # lived in the nodes, nothing imported them without rclpy, nothing tested
  # them, and a fatal typo shipped under a green suite.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_kiwi_kinematics test/test_kiwi_kinematics.py)
  ament_add_pytest_test(test_esp32_bridge test/test_esp32_bridge.py)
  ament_add_pytest_test(test_bridge_core test/test_bridge_core.py)
  ament_add_pytest_test(test_teleop_keys test/test_teleop_keys.py)
endif()

ament_package()
