# One RTOS implementation, with FreeRTOSConfig.h supplied by the selected board.
set(ROBOT_RTOS_SOURCES
  "${CHASSIS_ROOT}/third_party/freertos/Source/croutine.c"
  "${CHASSIS_ROOT}/third_party/freertos/Source/event_groups.c"
  "${CHASSIS_ROOT}/third_party/freertos/Source/list.c"
  "${CHASSIS_ROOT}/third_party/freertos/Source/queue.c"
  "${CHASSIS_ROOT}/third_party/freertos/Source/stream_buffer.c"
  "${CHASSIS_ROOT}/third_party/freertos/Source/tasks.c"
  "${CHASSIS_ROOT}/third_party/freertos/Source/timers.c"
  "${CHASSIS_ROOT}/third_party/freertos/Source/CMSIS_RTOS/cmsis_os.c"
  "${CHASSIS_ROOT}/third_party/freertos/Source/portable/MemMang/heap_4.c"
  "${CHASSIS_ROOT}/third_party/freertos/Source/portable/GCC/ARM_CM4F/port.c")
