cmake_minimum_required(VERSION 3.8)
project(robocup_sim)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/scan_no_hit_to_inf.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
