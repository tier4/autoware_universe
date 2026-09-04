include_guard(GLOBAL)

set(ACADOS_SOURCE_DIR "$ENV{ACADOS_SOURCE_DIR}" CACHE PATH "Path to the acados installation")
if(NOT ACADOS_SOURCE_DIR)
  set(ACADOS_SOURCE_DIR "/opt/acados" CACHE PATH "Path to the acados installation" FORCE)
endif()

function(diffusion_planner_add_acados_solver output_target)
  find_library(ACADOS_LIBRARY acados HINTS "${ACADOS_SOURCE_DIR}/lib")
  find_library(BLASFEO_LIBRARY blasfeo HINTS "${ACADOS_SOURCE_DIR}/lib")
  find_library(HPIPM_LIBRARY hpipm HINTS "${ACADOS_SOURCE_DIR}/lib")
  set(acados_python "${ACADOS_SOURCE_DIR}/.venv/bin/python3")

  if(NOT ACADOS_LIBRARY OR NOT BLASFEO_LIBRARY OR NOT HPIPM_LIBRARY OR
     NOT EXISTS "${acados_python}")
    message(WARNING
      "acados was not found under ${ACADOS_SOURCE_DIR}; "
      "trajectory optimization support will be disabled")
    set(${output_target} "" PARENT_SCOPE)
    return()
  endif()

  set(acados_model diffusion_planner_optimizer)
  set(generated_root "${CMAKE_CURRENT_BINARY_DIR}/acados_generated")
  set(generated_dir "${generated_root}/c_generated_code")
  file(MAKE_DIRECTORY "${generated_root}")
  set(generated_files
    "${generated_dir}/acados_solver_${acados_model}.h"
    "${generated_dir}/acados_solver_${acados_model}.c"
    "${generated_dir}/${acados_model}_model/${acados_model}_expl_ode_fun.c"
    "${generated_dir}/${acados_model}_model/${acados_model}_expl_vde_forw.c"
    "${generated_dir}/${acados_model}_model/${acados_model}_expl_vde_adj.c"
    "${generated_dir}/${acados_model}_constraints/${acados_model}_constr_h_fun.c"
    "${generated_dir}/${acados_model}_constraints/${acados_model}_constr_h_fun_jac_uxt_zt.c"
  )

  set_source_files_properties(${generated_files} PROPERTIES GENERATED TRUE)
  add_custom_command(
    OUTPUT ${generated_files}
    COMMAND ${CMAKE_COMMAND} -E env
      "ACADOS_SOURCE_DIR=${ACADOS_SOURCE_DIR}"
      "PYTHONPATH=${ACADOS_SOURCE_DIR}/interfaces/acados_template:${CMAKE_CURRENT_SOURCE_DIR}/scripts:$ENV{PYTHONPATH}"
      "${acados_python}" "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_solver.py"
    WORKING_DIRECTORY "${generated_root}"
    DEPENDS
      "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_solver.py"
      "${CMAKE_CURRENT_SOURCE_DIR}/scripts/vehicle_model.py"
    COMMENT "Generating acados trajectory optimization solver"
    VERBATIM
  )

  add_custom_target(generate_diffusion_planner_acados_code DEPENDS ${generated_files})
  add_library(autoware_diffusion_planner_acados SHARED
    src/optimization/acados_solver_wrapper.cpp
    ${generated_files}
  )
  add_dependencies(
    autoware_diffusion_planner_acados
    generate_diffusion_planner_acados_code
  )

  target_include_directories(autoware_diffusion_planner_acados
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    PRIVATE
      "${generated_root}"
      "${generated_dir}"
      "${ACADOS_SOURCE_DIR}/include"
      "${ACADOS_SOURCE_DIR}/include/acados"
      "${ACADOS_SOURCE_DIR}/include/acados_c"
      "${ACADOS_SOURCE_DIR}/include/blasfeo/include"
      "${ACADOS_SOURCE_DIR}/include/hpipm/include"
  )
  target_link_libraries(autoware_diffusion_planner_acados PRIVATE
    "${ACADOS_LIBRARY}"
    "${BLASFEO_LIBRARY}"
    "${HPIPM_LIBRARY}"
  )
  set_target_properties(autoware_diffusion_planner_acados PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    BUILD_RPATH "${ACADOS_SOURCE_DIR}/lib"
    INSTALL_RPATH "${ACADOS_SOURCE_DIR}/lib"
  )

  if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    set_source_files_properties(${generated_files} PROPERTIES
      COMPILE_OPTIONS "-Wno-unused-variable;-Wno-unused-parameter;-Wno-unused-function"
    )
  endif()

  install(TARGETS autoware_diffusion_planner_acados
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  set(${output_target} autoware_diffusion_planner_acados PARENT_SCOPE)
endfunction()
