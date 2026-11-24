cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_autonomy)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config models worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/velocity_arbiter.py scripts/color_cube_locator.py scripts/m5_scene.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/sort_cubes.py DESTINATION lib/${PROJECT_NAME})
install(FILES
  scripts/room_alignment.py
  ../../tools/acceptance/grasp_contact_probe.py
  ../../tools/acceptance/m4_vision_grasp.py
  ../../tools/acceptance/m42_dock_grasp.py
  ../../tools/acceptance/m52_random_pick.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()
