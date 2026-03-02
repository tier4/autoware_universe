# acados_mpc - Longitudinal-only MPC solver using acados
# Requires acados installed (e.g. at /opt/acados via ansible-playbook autoware.dev_env.setup_acados)
# and ACADOS_SOURCE_DIR set (env or parent). Parent must find_package(acados) before add_subdirectory.

set(ACADOS_SOURCE_DIR "$ENV{ACADOS_SOURCE_DIR}")
if(NOT ACADOS_SOURCE_DIR)
  set(ACADOS_SOURCE_DIR "/opt/acados")
endif()

# Validate required acados components
foreach(_check "bin/t_renderer" ".venv/bin/python3" "interfaces/acados_template")
  if(NOT EXISTS "${ACADOS_SOURCE_DIR}/${_check}")
    message(FATAL_ERROR "${_check} not found in ${ACADOS_SOURCE_DIR}. "
      "Run: ansible-playbook autoware.dev_env.setup_acados --ask-become-pass (or set ACADOS_SOURCE_DIR)")
  endif()
endforeach()

get_filename_component(PKG_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
set(GENERATORS_DIR "${PKG_SOURCE_DIR}/generators")
set(GEN_DIR "${GENERATORS_DIR}/c_generated_code")

set(_template_path "${ACADOS_SOURCE_DIR}/interfaces/acados_template")
set(_env_vars
  "ACADOS_SOURCE_DIR=${ACADOS_SOURCE_DIR}"
  "PYTHONPATH=$ENV{PYTHONPATH}:${_template_path}"
)

# --- Longitudinal-only MPC code generation ---
set(LON_MODEL "longitudinal_only")
set(LON_GENERATED_FILES
  ${GEN_DIR}/acados_solver_${LON_MODEL}.h
  ${GEN_DIR}/acados_solver_${LON_MODEL}.c
  ${GEN_DIR}/acados_sim_solver_${LON_MODEL}.h
  ${GEN_DIR}/acados_sim_solver_${LON_MODEL}.c
  ${GEN_DIR}/${LON_MODEL}_model/${LON_MODEL}_expl_ode_fun.c
  ${GEN_DIR}/${LON_MODEL}_model/${LON_MODEL}_expl_vde_adj.c
  ${GEN_DIR}/${LON_MODEL}_model/${LON_MODEL}_expl_vde_forw.c
  ${GEN_DIR}/${LON_MODEL}_model/${LON_MODEL}_model.h
)
set_source_files_properties(${LON_GENERATED_FILES} PROPERTIES GENERATED TRUE)

add_custom_command(
  OUTPUT ${LON_GENERATED_FILES}
  COMMAND ${CMAKE_COMMAND} -E env ${_env_vars}
  ${ACADOS_SOURCE_DIR}/.venv/bin/python3 mpc_longitudinal_only.py
  WORKING_DIRECTORY ${GENERATORS_DIR}
  DEPENDS ${GENERATORS_DIR}/mpc_longitudinal_only.py ${GENERATORS_DIR}/longitudinal_only_model.py
  COMMENT "Generating longitudinal-only MPC solver code..."
  VERBATIM
)

# --- acados_longitudinal_interface library (longitudinal-only OCP) ---
add_library(acados_longitudinal_interface SHARED
  acados_longitudinal_interface.cpp
  ${LON_GENERATED_FILES}
)

target_include_directories(acados_longitudinal_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${GEN_DIR}>
  ${ACADOS_SOURCE_DIR}/include
  ${ACADOS_SOURCE_DIR}/include/acados
  ${ACADOS_SOURCE_DIR}/include/acados_c
  ${ACADOS_SOURCE_DIR}/include/blasfeo/include
  ${ACADOS_SOURCE_DIR}/include/hpipm/include
)

target_link_libraries(acados_longitudinal_interface PRIVATE acados)

# Suppress warnings on generated C code
if(CMAKE_C_COMPILER_ID STREQUAL "GNU" OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  set_source_files_properties(${LON_GENERATED_FILES} PROPERTIES
    COMPILE_OPTIONS "-Wno-all;-Wno-extra;-Wno-unused-function")
endif()
