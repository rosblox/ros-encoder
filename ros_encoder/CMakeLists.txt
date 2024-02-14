
cmake_minimum_required(VERSION 3.8)
project(ros_encoder)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install launch and config files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  ${PROJECT_NAME}/multiturn_encoder_node.py
  ${PROJECT_NAME}/encoder_interface.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()