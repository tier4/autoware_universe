# --- acados_mpt - temporal kinematic bicycle MPC (acados) ---

find_package(Python3 REQUIRED COMPONENTS Interpreter)
message(STATUS "autoware_trajectory_optimizer: acados temporal MPC codegen Python: ${Python3_EXECUTABLE}")

# 1. Environment and path setup
set(ACADOS_SOURCE_DIR "$ENV{ACADOS_SOURCE_DIR}")
if(NOT ACADOS_SOURCE_DIR)
  set(ACADOS_SOURCE_DIR "/opt/acados")
endif()

# 2. Code generation (Python → C via t_renderer)
# Generate into binary dir so source tree is never modified; stub header stays in source for clang-tidy/IDE.
set(TEMPORAL_MODEL kinematic_bicycle_temporal)
set(GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/c_generated_code")

set(GENERATED_TEMPORAL_MPC_FILES
  ${GEN_DIR}/acados_solver_${TEMPORAL_MODEL}.h
  ${GEN_DIR}/acados_solver_${TEMPORAL_MODEL}.c
  ${GEN_DIR}/${TEMPORAL_MODEL}_model/${TEMPORAL_MODEL}_expl_ode_fun.c
  ${GEN_DIR}/${TEMPORAL_MODEL}_model/${TEMPORAL_MODEL}_expl_vde_adj.c
  ${GEN_DIR}/${TEMPORAL_MODEL}_model/${TEMPORAL_MODEL}_expl_vde_forw.c
  ${GEN_DIR}/${TEMPORAL_MODEL}_model/${TEMPORAL_MODEL}_model.h
  ${GEN_DIR}/${TEMPORAL_MODEL}_constraints/${TEMPORAL_MODEL}_constr_h_fun.c
  ${GEN_DIR}/${TEMPORAL_MODEL}_constraints/${TEMPORAL_MODEL}_constr_h_fun_jac_uxt_zt.c
)

set_source_files_properties(${GENERATED_TEMPORAL_MPC_FILES} PROPERTIES GENERATED TRUE)

get_filename_component(PACKAGE_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}" ABSOLUTE)
set(_template_path "${ACADOS_SOURCE_DIR}/interfaces/acados_template")
set(_env_vars
  "ACADOS_SOURCE_DIR=${ACADOS_SOURCE_DIR}"
  "PYTHONPATH=${_template_path}:${PACKAGE_ROOT_DIR}:$ENV{PYTHONPATH}"
)

# 3. Generation step (runs whenever generator inputs change or outputs are missing).
# Writes ${CMAKE_CURRENT_BINARY_DIR}/c_generated_code/ (not the source tree).
add_custom_command(
  OUTPUT ${GENERATED_TEMPORAL_MPC_FILES}
  COMMAND ${CMAKE_COMMAND} -E env ${_env_vars}
    ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/generators/path_tracking_mpc_temporal.py
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/generators/path_tracking_mpc_temporal.py
    ${CMAKE_CURRENT_SOURCE_DIR}/generators/bicycle_model_temporal.py
  COMMENT "Generating acados temporal MPC solver code (path_tracking_mpc_temporal.py)"
)

add_custom_target(generate_acados_code_temporal DEPENDS ${GENERATED_TEMPORAL_MPC_FILES})

add_library(acados_interface_temporal SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/src/acados_interface.cpp
  ${GENERATED_TEMPORAL_MPC_FILES}
)

add_dependencies(acados_interface_temporal generate_acados_code_temporal)

target_include_directories(acados_interface_temporal PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
  ${ACADOS_SOURCE_DIR}/include
  ${ACADOS_SOURCE_DIR}/include/acados
  ${ACADOS_SOURCE_DIR}/include/acados_c
  ${ACADOS_SOURCE_DIR}/include/blasfeo/include
  ${ACADOS_SOURCE_DIR}/include/hpipm/include
)

target_link_libraries(acados_interface_temporal PUBLIC acados)

if(CMAKE_C_COMPILER_ID STREQUAL "GNU" OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  set_source_files_properties(${GENERATED_TEMPORAL_MPC_FILES} PROPERTIES
    COMPILE_OPTIONS "-Wno-unused-variable;-Wno-unused-parameter;-Wno-unused-function")
endif()
