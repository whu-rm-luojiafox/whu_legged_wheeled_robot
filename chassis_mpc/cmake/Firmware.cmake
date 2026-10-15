include("${CMAKE_CURRENT_LIST_DIR}/FirmwareSources.cmake")
set(CHASSIS_FIRMWARE "lqr" CACHE STRING "Firmware application: lqr or mpc-probe")
set_property(CACHE CHASSIS_FIRMWARE PROPERTY STRINGS lqr mpc-probe)
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
if(CHASSIS_FIRMWARE STREQUAL "lqr")
  add_executable(chassis_firmware ${CHASSIS_FIRMWARE_SOURCES} "${startup}"
    "${CHASSIS_ROOT}/platform/stm32f407/protocol_layout_checks.c")
  target_include_directories(chassis_firmware PRIVATE
    "${CHASSIS_ROOT}/third_party/stm32_hal/Inc/Legacy"
    "${CHASSIS_ROOT}/third_party/freertos/Source/include"
    "${CHASSIS_ROOT}/third_party/freertos/Source/portable/GCC/ARM_CM4F"
    "${CHASSIS_ROOT}/third_party/freertos/Source/CMSIS_RTOS"
    "${CHASSIS_ROOT}/src/bsp/boards" "${CHASSIS_ROOT}/src/application"
    "${CHASSIS_ROOT}/src/components/algorithm" "${CHASSIS_ROOT}/src/components/algorithm/Include"
    "${CHASSIS_ROOT}/src/components/controller" "${CHASSIS_ROOT}/src/components/devices"
    "${CHASSIS_ROOT}/src/components/UI"
    "${CHASSIS_ROOT}/src/service/CanService" "${CHASSIS_ROOT}/src/protocols"
    "${CHASSIS_ROOT}/src/module/chassis")
  target_compile_definitions(chassis_firmware PRIVATE ARM_MATH_CM4 ARM_MATH_MATRIX_CHECK ARM_MATH_ROUNDING)
  target_link_libraries(chassis_firmware PRIVATE
    "${CHASSIS_ROOT}/src/components/algorithm/AHRS.lib"
    "${CHASSIS_ROOT}/src/components/algorithm/arm_cortexM4lf_math.lib")
elseif(CHASSIS_FIRMWARE STREQUAL "mpc-probe")
  chassis_add_mpc(chassis_mpc float)
  add_executable(chassis_firmware "${startup}"
    "${CHASSIS_ROOT}/platform/stm32f407/mpc_probe_main.c"
    "${CHASSIS_ROOT}/tools/chassis/mpc/board_probe.c"
    "${CHASSIS_ROOT}/platform/stm32f407/src/system_stm32f4xx.c")
  target_include_directories(chassis_firmware PRIVATE "${CHASSIS_ROOT}/tools/chassis/mpc")
  target_link_libraries(chassis_firmware PRIVATE chassis_mpc)
  set_target_properties(chassis_firmware PROPERTIES LINKER_LANGUAGE CXX)
else()
  message(FATAL_ERROR "Unknown CHASSIS_FIRMWARE=${CHASSIS_FIRMWARE}")
endif()
target_include_directories(chassis_firmware PRIVATE
  "${CHASSIS_ROOT}/platform/stm32f407/include" "${CHASSIS_ROOT}/third_party/stm32_hal/Inc"
  "${CHASSIS_ROOT}/third_party/cmsis/Device/ST/STM32F4xx/Include"
  "${CHASSIS_ROOT}/third_party/cmsis/Include")
target_compile_definitions(chassis_firmware PRIVATE USE_HAL_DRIVER STM32F407xx __FPU_PRESENT=1U)
# Keep source lines and variable types available in Ozone, including Release builds.
target_compile_options(chassis_firmware PRIVATE -std=gnu11 -g)
set(scatter "${CHASSIS_ROOT}/platform/stm32f407/stm32f407ig.sct")
target_link_options(chassis_firmware PRIVATE --scatter=${scatter}
  # Keep RW section sizes equal to runtime RAM spans for debugger write validation.
  --datacompressor=off
  --map --info=sizes,totals --list=${CMAKE_BINARY_DIR}/chassis_firmware.map)
set_property(TARGET chassis_firmware APPEND PROPERTY LINK_DEPENDS "${scatter}")
add_custom_command(TARGET chassis_firmware POST_BUILD
  COMMAND "${CHASSIS_FROMELF}" --i32combined "--output=${CMAKE_BINARY_DIR}/chassis_firmware.hex" "$<TARGET_FILE:chassis_firmware>"
  COMMAND "${CHASSIS_FROMELF}" --bin "--output=${CMAKE_BINARY_DIR}/chassis_firmware.bin" "$<TARGET_FILE:chassis_firmware>"
  VERBATIM)
