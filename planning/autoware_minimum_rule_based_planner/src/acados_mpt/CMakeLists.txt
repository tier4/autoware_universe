# --- acados_mpt - time-axis kinematic feasibility OCP (acados) ---

# 1. Environment and Path Setup
set(ACADOS_SOURCE_DIR "$ENV{ACADOS_SOURCE_DIR}")
if(NOT ACADOS_SOURCE_DIR)
  set(ACADOS_SOURCE_DIR "/opt/acados")
endif()

# 2. Code generation (Python -> C via t_renderer)
# Generate into binary dir so the source tree is never modified.
set(TIME_MODEL kinematic_bicycle_time)
set(GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/c_generated_code")

set(GENERATED_TIME_OCP_FILES
  ${GEN_DIR}/acados_solver_${TIME_MODEL}.h
  ${GEN_DIR}/acados_solver_${TIME_MODEL}.c
  ${GEN_DIR}/${TIME_MODEL}_model/${TIME_MODEL}_expl_ode_fun.c
  ${GEN_DIR}/${TIME_MODEL}_model/${TIME_MODEL}_expl_vde_adj.c
  ${GEN_DIR}/${TIME_MODEL}_model/${TIME_MODEL}_expl_vde_forw.c
  ${GEN_DIR}/${TIME_MODEL}_model/${TIME_MODEL}_model.h
)

set_source_files_properties(${GENERATED_TIME_OCP_FILES} PROPERTIES GENERATED TRUE)

get_filename_component(PACKAGE_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}" ABSOLUTE)
set(_template_path "${ACADOS_SOURCE_DIR}/interfaces/acados_template")
set(_env_vars
  "ACADOS_SOURCE_DIR=${ACADOS_SOURCE_DIR}"
  "PYTHONPATH=${_template_path}:${PACKAGE_ROOT_DIR}:$ENV{PYTHONPATH}"
)

# 3. Define the Generation Target (generator writes to CWD/c_generated_code -> binary dir)
add_custom_command(
  OUTPUT ${GENERATED_TIME_OCP_FILES}
  COMMAND ${CMAKE_COMMAND} -E env ${_env_vars}
    ${ACADOS_SOURCE_DIR}/.venv/bin/python3
    ${CMAKE_CURRENT_SOURCE_DIR}/generators/path_kinematic_feasibility.py
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/generators/path_kinematic_feasibility.py
    ${CMAKE_CURRENT_SOURCE_DIR}/generators/bicycle_model_time.py
  COMMENT "Generating acados time-axis kinematic feasibility OCP code"
)

add_custom_target(generate_acados_code_time DEPENDS ${GENERATED_TIME_OCP_FILES})

# 4. acados_interface_time library
add_library(acados_interface_time SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/src/acados_interface_time.cpp
  ${GENERATED_TIME_OCP_FILES}
)

add_dependencies(acados_interface_time generate_acados_code_time)

target_include_directories(acados_interface_time PUBLIC
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

target_link_libraries(acados_interface_time PUBLIC acados)

# 5. Suppress warnings on generated C code
if(CMAKE_C_COMPILER_ID STREQUAL "GNU" OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  set_source_files_properties(${GENERATED_TIME_OCP_FILES} PROPERTIES
    COMPILE_OPTIONS "-Wno-unused-variable;-Wno-unused-parameter;-Wno-unused-function")
endif()

# 6. Install
# The header is consumed only inside this package (private to the
# minimum-rule-based path optimizer plugin), so we install the shared
# library only.
install(TARGETS acados_interface_time
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)
