# Ensure optional acados-generated sources exist so CMake OUTPUT lists stay general.
#
# When NH == 0, AcadosOcpSolver.generate does not emit *_constraints/*.c. Creating
# placeholder translation units keeps parallel builds deterministic without
# requiring NH-specific CMake. Placeholders must not be empty: -Werror=pedantic
# rejects empty TUs (ISO C).
#
# Required cache/vars (pass with -D):
#   GEN_DIR, TEMPORAL_MODEL

if(NOT DEFINED GEN_DIR OR NOT DEFINED TEMPORAL_MODEL)
  message(FATAL_ERROR "GEN_DIR and TEMPORAL_MODEL must be set")
endif()

set(_optional_sources
  "${GEN_DIR}/${TEMPORAL_MODEL}_constraints/${TEMPORAL_MODEL}_constr_h_fun.c"
  "${GEN_DIR}/${TEMPORAL_MODEL}_constraints/${TEMPORAL_MODEL}_constr_h_fun_jac_uxt_zt.c"
  "${GEN_DIR}/${TEMPORAL_MODEL}_constraints/${TEMPORAL_MODEL}_constr_h_0_fun.c"
  "${GEN_DIR}/${TEMPORAL_MODEL}_constraints/${TEMPORAL_MODEL}_constr_h_0_fun_jac_uxt_zt.c"
)

foreach(_src IN LISTS _optional_sources)
  set(_needs_stub FALSE)
  if(NOT EXISTS "${_src}")
    set(_needs_stub TRUE)
  else()
    # Refresh comment-only / prior empty stubs; never overwrite real acados codegen.
    file(READ "${_src}" _existing LIMIT 256)
    if(_existing MATCHES "Placeholder: acados did not emit")
      set(_needs_stub TRUE)
    endif()
  endif()

  if(_needs_stub)
    get_filename_component(_dir "${_src}" DIRECTORY)
    get_filename_component(_base "${_src}" NAME_WE)
    file(MAKE_DIRECTORY "${_dir}")
    # Typedef keeps the TU non-empty under -Wpedantic; unused by the solver when NH==0.
    file(WRITE "${_src}"
      "/* Placeholder: acados did not emit this file (e.g. NH == 0). */\n"
      "typedef int ${_base}_placeholder;\n")
  endif()
endforeach()
