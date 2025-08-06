cmake_minimum_required(VERSION 3.5)
project(my_robot_description)

find_package(ament_cmake REQUIRED)

# Install Python script as an executable
install(PROGRAMS
  scripts/steering_controller.py
  scripts/keyboard_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install other shared files
install(DIRECTORY urdf launch rvizz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
