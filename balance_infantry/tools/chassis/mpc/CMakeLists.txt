cmake_minimum_required(VERSION 3.21)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(whu_chassis_mpc_probe LANGUAGES C CXX)
  include(CTest)
endif()
include("${CMAKE_CURRENT_LIST_DIR}/../../../cmake/ChassisMpc.cmake")
foreach(precision double float)
  chassis_add_mpc(chassis_mpc_${precision} ${precision})
  add_executable(mpc_probe_${precision} host_probe.cpp)
  target_link_libraries(mpc_probe_${precision} PRIVATE chassis_mpc_${precision})
  target_compile_features(mpc_probe_${precision} PRIVATE cxx_std_17)
  # This diagnostic executable intentionally inspects solver internals.
  target_include_directories(mpc_probe_${precision} PRIVATE
    "${CHASSIS_ROOT}/src/chassis/components/controller/chassis_mpc/mpc_generated"
    "${CHASSIS_ROOT}/third_party/tinympc/port"
    "${CHASSIS_ROOT}/third_party/tinympc/include/Eigen")
  target_compile_definitions(mpc_probe_${precision} PRIVATE
    WHU_TINY_FIXED_RHO WHU_TINY_QUIET EIGEN_DONT_VECTORIZE EIGEN_MAX_ALIGN_BYTES=0)
  if(precision STREQUAL "float")
    target_compile_definitions(mpc_probe_${precision} PRIVATE WHU_TINY_FLOAT)
  endif()
  set_target_properties(mpc_probe_${precision} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
  if(MINGW)
    target_link_options(mpc_probe_${precision} PRIVATE -static)
  endif()
endforeach()
find_package(Python3 COMPONENTS Interpreter)
if(BUILD_TESTING AND Python3_Interpreter_FOUND)
  add_test(NAME mpc_reference_qp COMMAND "${Python3_EXECUTABLE}"
    "${CHASSIS_ROOT}/tools/chassis/simulation/compare_lqr_mpc.py"
    --build-dir "${CMAKE_BINARY_DIR}" --output-dir "${CMAKE_BINARY_DIR}/reports")
endif()
