set(ROBOT_BOARD "chassis" CACHE STRING "Robot board: chassis or gimbal")
set_property(CACHE ROBOT_BOARD PROPERTY STRINGS chassis gimbal)
if(NOT ROBOT_BOARD MATCHES "^(chassis|gimbal)$")
  message(FATAL_ERROR "Unknown ROBOT_BOARD=${ROBOT_BOARD}; choose chassis or gimbal")
endif()
set(CHASSIS_FIRMWARE "lqr" CACHE STRING "Chassis application: lqr or mpc-probe")
set_property(CACHE CHASSIS_FIRMWARE PROPERTY STRINGS lqr mpc-probe)
if(ROBOT_BOARD STREQUAL "gimbal" AND NOT CHASSIS_FIRMWARE STREQUAL "lqr")
  message(FATAL_ERROR "mpc-probe is a chassis-only diagnostic; select ROBOT_BOARD=chassis")
endif()
set(board_include "${CHASSIS_ROOT}/platform/stm32f407/chassis/include")
set(hal_root "${CHASSIS_ROOT}/third_party/stm32_hal")
set(firmware_name chassis_firmware)
if(ROBOT_BOARD STREQUAL "gimbal")
  set(board_include "${CHASSIS_ROOT}/platform/stm32f407/gimbal/include")
  set(hal_root "${CHASSIS_ROOT}/third_party/stm32_hal_gimbal")
  set(firmware_name gimbal_firmware)
endif()
set(startup_options)
if(CHASSIS_FIRMWARE STREQUAL "mpc-probe")
  # Explicit test budget, not an assertion about measured heap peak.
  list(APPEND startup_options --pd "WHU_HEAP_SIZE SETA 0x8000")
endif()
add_custom_command(OUTPUT "${CMAKE_BINARY_DIR}/startup_stm32f407xx.o"
  COMMAND "${CHASSIS_ARMASM}" --cpu Cortex-M4.fp.sp --apcs=interwork ${startup_options}
    "${CHASSIS_ROOT}/platform/stm32f407/startup_stm32f407xx.s"
    -o "${CMAKE_BINARY_DIR}/startup_stm32f407xx.o"
  DEPENDS "${CHASSIS_ROOT}/platform/stm32f407/startup_stm32f407xx.s" VERBATIM)
set(startup "${CMAKE_BINARY_DIR}/startup_stm32f407xx.o")
set_source_files_properties("${startup}" PROPERTIES EXTERNAL_OBJECT TRUE GENERATED TRUE)
if(ROBOT_BOARD STREQUAL "gimbal")
  include("${CMAKE_CURRENT_LIST_DIR}/GimbalSources.cmake")
  include("${CMAKE_CURRENT_LIST_DIR}/RobotRuntime.cmake")
  # Keep the established target name for existing scripts; the output identifies
  # the selected board. Only this source list participates in the gimbal link.
  add_executable(chassis_firmware ${GIMBAL_FIRMWARE_SOURCES}
    ${ROBOT_RTOS_SOURCES} "${startup}"
    "${CHASSIS_ROOT}/platform/stm32f407/robot_link_layout_checks.c")
  set(gimbal_dir "${CHASSIS_ROOT}/src/gimbal")
  target_include_directories(chassis_firmware PRIVATE
    "${CHASSIS_ROOT}/src/gimbal/module"
    "${gimbal_dir}/application" "${gimbal_dir}/application/protocol"
    "${gimbal_dir}/bsp/boards" "${gimbal_dir}/components/devices"
    "${gimbal_dir}/components/algorithm" "${gimbal_dir}/components/support"
    "${gimbal_dir}/components/ui"
    "${CHASSIS_ROOT}/third_party/stm32_usb_device/Core/Inc"
    "${CHASSIS_ROOT}/third_party/stm32_usb_device/Class/CDC/Inc"
    "${CHASSIS_ROOT}/third_party/freertos/Source/include"
    "${CHASSIS_ROOT}/third_party/freertos/Source/portable/GCC/ARM_CM4F"
    "${CHASSIS_ROOT}/third_party/freertos/Source/CMSIS_RTOS")
  target_compile_definitions(chassis_firmware PRIVATE
    ROBOT_BOARD_GIMBAL=1 ARM_MATH_CM4 ARM_MATH_MATRIX_CHECK ARM_MATH_ROUNDING)
  target_link_libraries(chassis_firmware PRIVATE ${GIMBAL_FIRMWARE_LIBRARIES})
elseif(CHASSIS_FIRMWARE STREQUAL "lqr")
  include("${CMAKE_CURRENT_LIST_DIR}/ChassisSources.cmake")
  add_executable(chassis_firmware ${CHASSIS_FIRMWARE_SOURCES} "${startup}"
    "${CHASSIS_ROOT}/platform/stm32f407/chassis/protocol_layout_checks.c"
    "${CHASSIS_ROOT}/platform/stm32f407/robot_link_layout_checks.c")
  target_include_directories(chassis_firmware PRIVATE
    "${CHASSIS_ROOT}/third_party/stm32_hal/Inc/Legacy"
    "${CHASSIS_ROOT}/third_party/freertos/Source/include"
    "${CHASSIS_ROOT}/third_party/freertos/Source/portable/GCC/ARM_CM4F"
    "${CHASSIS_ROOT}/third_party/freertos/Source/CMSIS_RTOS"
    "${CHASSIS_ROOT}/src/chassis/bsp/boards" "${CHASSIS_ROOT}/src/chassis/application"
    "${CHASSIS_ROOT}/src/chassis/components/algorithm" "${CHASSIS_ROOT}/src/common/components/algorithm/Include"
    "${CHASSIS_ROOT}/src/chassis/components/controller" "${CHASSIS_ROOT}/src/chassis/components/devices"
    "${CHASSIS_ROOT}/src/chassis/components/UI"
    "${CHASSIS_ROOT}/src/chassis/service/CanService" "${CHASSIS_ROOT}/src/chassis/protocols"
    "${CHASSIS_ROOT}/src/chassis/module")
  target_compile_definitions(chassis_firmware PRIVATE ROBOT_BOARD_CHASSIS=1 ARM_MATH_CM4 ARM_MATH_MATRIX_CHECK ARM_MATH_ROUNDING)
  target_link_libraries(chassis_firmware PRIVATE
    "${CHASSIS_ROOT}/src/common/components/algorithm/AHRS.lib"
    "${CHASSIS_ROOT}/src/common/components/algorithm/arm_cortexM4lf_math.lib")
elseif(CHASSIS_FIRMWARE STREQUAL "mpc-probe")
  chassis_add_mpc(chassis_mpc float)
  add_executable(chassis_firmware "${startup}"
    "${CHASSIS_ROOT}/platform/stm32f407/chassis/mpc_probe_main.c"
    "${CHASSIS_ROOT}/tools/chassis/mpc/board_probe.c"
    "${CHASSIS_ROOT}/platform/stm32f407/chassis/src/system_stm32f4xx.c")
  target_include_directories(chassis_firmware PRIVATE "${CHASSIS_ROOT}/tools/chassis/mpc")
  target_link_libraries(chassis_firmware PRIVATE chassis_mpc)
  target_compile_definitions(chassis_firmware PRIVATE ROBOT_BOARD_CHASSIS=1)
  set_target_properties(chassis_firmware PROPERTIES LINKER_LANGUAGE CXX)
else()
  message(FATAL_ERROR "Unknown CHASSIS_FIRMWARE=${CHASSIS_FIRMWARE}")
endif()
target_include_directories(chassis_firmware PRIVATE
  "${board_include}" "${hal_root}/Inc" "${hal_root}/Inc/Legacy"
  "${CHASSIS_ROOT}/src/common/include"
  "${CHASSIS_ROOT}/src/common/protocols"
  "${CHASSIS_ROOT}/src/common/bsp/boards"
  "${CHASSIS_ROOT}/src/common/components/devices"
  "${CHASSIS_ROOT}/src/common/components/controller"
  "${CHASSIS_ROOT}/src/common/components/algorithm"
  "${CHASSIS_ROOT}/src/common/components/algorithm/Include"
  "${CHASSIS_ROOT}/third_party/cmsis/Device/ST/STM32F4xx/Include"
  "${CHASSIS_ROOT}/third_party/cmsis/Include")
target_compile_definitions(chassis_firmware PRIVATE USE_HAL_DRIVER STM32F407xx __FPU_PRESENT=1U)
target_compile_options(chassis_firmware PRIVATE -include "${CHASSIS_ROOT}/src/common/include/robot_board.h")
set_target_properties(chassis_firmware PROPERTIES OUTPUT_NAME "${firmware_name}")
# Keep source lines and variable types available in Ozone, including Release builds.
target_compile_options(chassis_firmware PRIVATE -std=gnu11 -g)
set(scatter "${CHASSIS_ROOT}/platform/stm32f407/stm32f407ig.sct")
target_link_options(chassis_firmware PRIVATE --scatter=${scatter}
  # Keep RW section sizes equal to runtime RAM spans for debugger write validation.
  --datacompressor=off
  --map --info=sizes,totals --list=${CMAKE_BINARY_DIR}/${firmware_name}.map)
set_property(TARGET chassis_firmware APPEND PROPERTY LINK_DEPENDS "${scatter}")
add_custom_command(TARGET chassis_firmware POST_BUILD
  COMMAND "${CHASSIS_FROMELF}" --i32combined "--output=${CMAKE_BINARY_DIR}/${firmware_name}.hex" "$<TARGET_FILE:chassis_firmware>"
  COMMAND "${CHASSIS_FROMELF}" --bin "--output=${CMAKE_BINARY_DIR}/${firmware_name}.bin" "$<TARGET_FILE:chassis_firmware>"
  VERBATIM)
message(STATUS "balance_robot: board=${ROBOT_BOARD}, application=${CHASSIS_FIRMWARE}, output=${firmware_name}")
