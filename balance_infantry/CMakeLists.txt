cmake_minimum_required(VERSION 3.21)
cmake_policy(SET CMP0123 NEW)
project(balance_robot LANGUAGES C CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CHASSIS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")
if(NOT CMAKE_C_COMPILER_ID STREQUAL "ARMClang")
  message(FATAL_ERROR "Firmware currently requires cmake/toolchains/armclang.cmake (legacy ARM libraries).")
endif()
# ROBOT_BOARD selects the chassis/gimbal source manifest, includes and macros.
include(cmake/Firmware.cmake)
