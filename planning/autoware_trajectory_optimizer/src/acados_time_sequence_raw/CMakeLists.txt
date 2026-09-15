# --- acados time-sequence raw optimizer (pose-only kinematic bicycle) ---
# Independent of src/acados_mpt (Temporal MPT): different model name and generated symbols.

set(ACADOS_SOURCE_DIR "$ENV{ACADOS_SOURCE_DIR}")
if(NOT ACADOS_SOURCE_DIR)
  set(ACADOS_SOURCE_DIR "/opt/acados")
endif()

set(TIME_SEQ_MODEL kinematic_bicycle_time_seq)
set(GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/c_generated_code")

set(GENERATED_TIME_SEQ_FILES
  ${GEN_DIR}/acados_solver_${TIME_SEQ_MODEL}.h
  ${GEN_DIR}/acados_solver_${TIME_SEQ_MODEL}.c
  ${GEN_DIR}/${TIME_SEQ_MODEL}_model/${TIME_SEQ_MODEL}_expl_ode_fun.c
  ${GEN_DIR}/${TIME_SEQ_MODEL}_model/${TIME_SEQ_MODEL}_expl_vde_adj.c
  ${GEN_DIR}/${TIME_SEQ_MODEL}_model/${TIME_SEQ_MODEL}_expl_vde_forw.c
  ${GEN_DIR}/${TIME_SEQ_MODEL}_model/${TIME_SEQ_MODEL}_model.h
  ${GEN_DIR}/${TIME_SEQ_MODEL}_constraints/${TIME_SEQ_MODEL}_constr_h_fun.c
  ${GEN_DIR}/${TIME_SEQ_MODEL}_constraints/${TIME_SEQ_MODEL}_constr_h_fun_jac_uxt_zt.c
)

set_source_files_properties(${GENERATED_TIME_SEQ_FILES} PROPERTIES GENERATED TRUE)
get_filename_component(PACKAGE_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}" ABSOLUTE)
set(_template_path "${ACADOS_SOURCE_DIR}/interfaces/acados_template")
set(_env_vars
  "ACADOS_SOURCE_DIR=${ACADOS_SOURCE_DIR}"
  "PYTHONPATH=${_template_path}:${PACKAGE_ROOT_DIR}:$ENV{PYTHONPATH}"
)

add_custom_command(
  OUTPUT ${GENERATED_TIME_SEQ_FILES}
  COMMAND ${CMAKE_COMMAND} -E env ${_env_vars}
    ${ACADOS_SOURCE_DIR}/.venv/bin/python3
    ${CMAKE_CURRENT_SOURCE_DIR}/generators/generate_solver.py
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/generators/generate_solver.py
    ${CMAKE_CURRENT_SOURCE_DIR}/generators/vehicle_model.py
  COMMENT "Generating acados time-sequence raw optimizer solver"
)

add_custom_target(generate_acados_code_time_seq DEPENDS ${GENERATED_TIME_SEQ_FILES})

add_library(acados_interface_time_seq SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/src/acados_solver_wrapper.cpp
  ${GENERATED_TIME_SEQ_FILES}
)

add_dependencies(acados_interface_time_seq generate_acados_code_time_seq)

target_include_directories(acados_interface_time_seq PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
  $<INSTALL_INTERFACE:include>
  PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}
  ${ACADOS_SOURCE_DIR}/include
  ${ACADOS_SOURCE_DIR}/include/acados
  ${ACADOS_SOURCE_DIR}/include/acados_c
  ${ACADOS_SOURCE_DIR}/include/blasfeo/include
  ${ACADOS_SOURCE_DIR}/include/hpipm/include
)

target_compile_features(acados_interface_time_seq PUBLIC cxx_std_17)
target_link_libraries(acados_interface_time_seq PUBLIC acados)

if(CMAKE_C_COMPILER_ID STREQUAL "GNU" OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  set_source_files_properties(${GENERATED_TIME_SEQ_FILES} PROPERTIES
    COMPILE_OPTIONS "-Wno-unused-variable;-Wno-unused-parameter;-Wno-unused-function")
endif()
