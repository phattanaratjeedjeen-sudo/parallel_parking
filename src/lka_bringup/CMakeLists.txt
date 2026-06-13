cmake_minimum_required(VERSION 3.5)
project(lka_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/park_planning.py
  scripts/log_data.py
  scripts/plot_results.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()