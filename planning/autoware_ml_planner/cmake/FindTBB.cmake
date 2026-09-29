# Prefer official oneTBB Config over mrt_cmake_modules' legacy FindTBB.
# Legacy FindTBB reads tbb/tbb_stddef.h, which oneTBB removed in favor of
# tbb/version.h (Ubuntu 24.04 / ROS 2 Jazzy).

if(TARGET TBB::tbb)
  set(TBB_FOUND TRUE)
  return()
endif()

# Avoid recursion into this same FindTBB.cmake while probing Config mode.
set(_ml_planner_tbb_cmake_module_path_backup "${CMAKE_MODULE_PATH}")
set(CMAKE_MODULE_PATH "")
find_package(TBB CONFIG QUIET)
set(CMAKE_MODULE_PATH "${_ml_planner_tbb_cmake_module_path_backup}")
unset(_ml_planner_tbb_cmake_module_path_backup)

if(TBB_FOUND)
  return()
endif()

find_path(TBB_INCLUDE_DIRS NAMES tbb/tbb.h)
if(NOT TBB_INCLUDE_DIRS)
  set(TBB_FOUND FALSE)
  return()
endif()

set(_ml_planner_tbb_ver_hdr "")
if(EXISTS "${TBB_INCLUDE_DIRS}/tbb/version.h")
  set(_ml_planner_tbb_ver_hdr "${TBB_INCLUDE_DIRS}/tbb/version.h")
elseif(EXISTS "${TBB_INCLUDE_DIRS}/oneapi/tbb/version.h")
  set(_ml_planner_tbb_ver_hdr "${TBB_INCLUDE_DIRS}/oneapi/tbb/version.h")
elseif(EXISTS "${TBB_INCLUDE_DIRS}/tbb/tbb_stddef.h")
  set(_ml_planner_tbb_ver_hdr "${TBB_INCLUDE_DIRS}/tbb/tbb_stddef.h")
endif()

if(_ml_planner_tbb_ver_hdr)
  file(READ "${_ml_planner_tbb_ver_hdr}" _ml_planner_tbb_version_file)
  string(REGEX REPLACE ".*#define TBB_VERSION_MAJOR ([0-9]+).*" "\\1"
    TBB_VERSION_MAJOR "${_ml_planner_tbb_version_file}")
  string(REGEX REPLACE ".*#define TBB_VERSION_MINOR ([0-9]+).*" "\\1"
    TBB_VERSION_MINOR "${_ml_planner_tbb_version_file}")
  string(REGEX REPLACE ".*#define TBB_INTERFACE_VERSION ([0-9]+).*" "\\1"
    TBB_INTERFACE_VERSION "${_ml_planner_tbb_version_file}")
  set(TBB_VERSION "${TBB_VERSION_MAJOR}.${TBB_VERSION_MINOR}")
  unset(_ml_planner_tbb_version_file)
endif()
unset(_ml_planner_tbb_ver_hdr)

find_library(TBB_LIBRARY NAMES tbb)
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TBB
  REQUIRED_VARS TBB_INCLUDE_DIRS TBB_LIBRARY
  VERSION_VAR TBB_VERSION
)

if(TBB_FOUND)
  set(TBB_LIBRARIES ${TBB_LIBRARY})
  if(NOT TARGET TBB::tbb)
    add_library(TBB::tbb UNKNOWN IMPORTED)
    set_target_properties(TBB::tbb PROPERTIES
      IMPORTED_LOCATION "${TBB_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${TBB_INCLUDE_DIRS}"
    )
  endif()
endif()
