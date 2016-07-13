cmake_minimum_required(VERSION 2.8.3)
project(auv_arduino)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rosserial_arduino
  rosserial_client
  std_msgs
  message_generation
  auv_motor_control
)

add_service_files(FILES
  InitESC.srv
)

add_message_files(FILES
)
generate_messages()

catkin_package(
  INCLUDE_DIRS include
#  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

rosserial_configure_client(
  DIRECTORY ard1
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)




rosserial_add_client_target(ard1 firmware ALL)
rosserial_add_client_target(ard1 firmware-upload)
