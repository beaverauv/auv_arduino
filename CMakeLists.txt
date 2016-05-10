cmake_minimum_required(VERSION 2.8.3)
project(auv_arduino)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rosserial_arduino
  rosserial_client
  std_msgs
)

catkin_package()

rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

rosserial_configure_client(
  DIRECTORY ard1
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_configure_client(
  DIRECTORY ard2     
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)



rosserial_add_client_target(ard1 firmware ALL)
rosserial_add_client_target(ard1 firmware-upload)
rosserial_add_client_target(ard2 firmware ALL)
rosserial_add_client_target(ard2 firmware-upload)

