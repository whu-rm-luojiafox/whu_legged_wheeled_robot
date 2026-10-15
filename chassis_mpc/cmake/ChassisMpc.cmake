include_guard(GLOBAL)
get_filename_component(CHASSIS_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)

# The C API is public. Eigen, TinyMPC types and generated caches stay private.
function(chassis_add_mpc target precision)
  set(controller_dir "${CHASSIS_ROOT}/src/components/controller/chassis_mpc")
  set(vendor "${CHASSIS_ROOT}/third_party/tinympc")
  add_library(${target} STATIC
    "${controller_dir}/chassis_mpc.cpp"
    "${controller_dir}/mpc_generated/chassis_tiny_data.cpp"
    "${vendor}/port/admm.cpp")
  target_include_directories(${target} PUBLIC "${controller_dir}")
  target_include_directories(${target} PRIVATE
    "${controller_dir}/mpc_generated" "${vendor}/port" "${vendor}/include/Eigen")
  target_compile_definitions(${target} PRIVATE
    WHU_TINY_FIXED_RHO WHU_TINY_QUIET EIGEN_DONT_VECTORIZE EIGEN_MAX_ALIGN_BYTES=0)
  if(precision STREQUAL "float")
    target_compile_definitions(${target} PRIVATE WHU_TINY_FLOAT)
  elseif(NOT precision STREQUAL "double")
    message(FATAL_ERROR "Unknown MPC precision: ${precision}")
  endif()
  # ARMClang 6.7 accepts C++17 as c++1z; its CMake feature table predates C++17.
  if(CMAKE_CXX_COMPILER_ID STREQUAL "ARMClang")
    target_compile_options(${target} PRIVATE -std=c++1z -Wno-return-type-c-linkage)
  else()
    target_compile_features(${target} PRIVATE cxx_std_17)
  endif()
endfunction()
