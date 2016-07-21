cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR} ../include)

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
add_definitions(-DUSB_CON)

generate_arduino_firmware(firmware
  SRCS firmware.cpp ${ROS_LIB_DIR}/time.cpp ../include/BlueRobotics_MS5837_Library/MS5837.cpp 
  BOARD mega2560
  PORT /dev/ttyACM0
)

