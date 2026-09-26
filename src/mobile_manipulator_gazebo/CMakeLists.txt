cmake_minimum_required(VERSION 3.19)
project(mobile_manipulator_gazebo)
find_package(ament_cmake REQUIRED)
# Windows/VMware copies may lose POSIX executable bits; symlink-install needs them.
file(CHMOD
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/cmd_vel_guard.py
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/drive.py
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/acceptance_test.py
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/arm_control.py
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
install(DIRECTORY launch worlds config scripts DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/cmd_vel_guard.py scripts/drive.py scripts/acceptance_test.py scripts/arm_control.py scripts/save_slam_map.py scripts/slam_test_route.py
        DESTINATION lib/${PROJECT_NAME})
# Stage 6: lightweight teleoperation and read-only health checks.
install(PROGRAMS scripts/teleop_router.py scripts/teleop_keyboard.py scripts/system_health.py
        DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/teleop_core.py DESTINATION lib/${PROJECT_NAME})
# Final-stage optional Tk launcher; control interfaces remain in existing scripts.
install(PROGRAMS scripts/robot_panel.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/panel_runtime.py DESTINATION lib/${PROJECT_NAME})
ament_package()
