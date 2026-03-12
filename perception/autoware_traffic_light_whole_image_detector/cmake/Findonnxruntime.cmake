# Find ONNX Runtime (Microsoft inference engine for .onnx models)
# Sets ONNXRUNTIME_FOUND, ONNXRUNTIME_INCLUDE_DIRS, ONNXRUNTIME_LIBRARIES
# Install: sudo apt install libonnxruntime-dev (Ubuntu 22.04+ universe)

find_path(ONNXRUNTIME_INCLUDE_DIR
  NAMES onnxruntime_cxx_api.h
  PATH_SUFFIXES include
  HINTS $ENV{ONNXRUNTIME_ROOT} $ENV{ONNXRUNTIME_HOME}
  PATHS /usr/local /usr
)

find_library(ONNXRUNTIME_LIBRARY
  NAMES onnxruntime libonnxruntime
  PATH_SUFFIXES lib lib64 x86_64-linux-gnu
  HINTS $ENV{ONNXRUNTIME_ROOT} $ENV{ONNXRUNTIME_HOME}
  PATHS /usr/local /usr
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(onnxruntime
  FOUND_VAR ONNXRUNTIME_FOUND
  REQUIRED_VARS ONNXRUNTIME_LIBRARY ONNXRUNTIME_INCLUDE_DIR
)

if(ONNXRUNTIME_FOUND)
  set(ONNXRUNTIME_INCLUDE_DIRS ${ONNXRUNTIME_INCLUDE_DIR})
  set(ONNXRUNTIME_LIBRARIES ${ONNXRUNTIME_LIBRARY})
  if(NOT TARGET onnxruntime::onnxruntime)
    add_library(onnxruntime::onnxruntime UNKNOWN IMPORTED)
    set_target_properties(onnxruntime::onnxruntime PROPERTIES
      IMPORTED_LOCATION "${ONNXRUNTIME_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_INCLUDE_DIR}"
    )
  endif()
endif()
