include_guard(GLOBAL)

function(ml_planner_find_dependencies)
  find_package(CUDAToolkit REQUIRED)

  find_library(ML_PLANNER_NVINFER_LIBRARY NAMES nvinfer)
  find_library(ML_PLANNER_NVONNXPARSER_LIBRARY NAMES nvonnxparser)
  if(NOT ML_PLANNER_NVINFER_LIBRARY OR
     NOT ML_PLANNER_NVONNXPARSER_LIBRARY)
    message(FATAL_ERROR
      "TensorRT libraries (nvinfer and nvonnxparser) are required by ${PROJECT_NAME}")
  endif()

  add_library(MLPlanner::nvinfer UNKNOWN IMPORTED)
  set_target_properties(MLPlanner::nvinfer PROPERTIES
    IMPORTED_LOCATION "${ML_PLANNER_NVINFER_LIBRARY}"
  )
  add_library(MLPlanner::nvonnxparser UNKNOWN IMPORTED)
  set_target_properties(MLPlanner::nvonnxparser PROPERTIES
    IMPORTED_LOCATION "${ML_PLANNER_NVONNXPARSER_LIBRARY}"
  )

  if(ML_PLANNER_VERBOSE_DEPENDENCIES)
    message(STATUS "CUDA toolkit: ${CUDAToolkit_VERSION}")
    message(STATUS "TensorRT inference library: ${ML_PLANNER_NVINFER_LIBRARY}")
    message(STATUS "TensorRT ONNX parser: ${ML_PLANNER_NVONNXPARSER_LIBRARY}")
  endif()

  set(has_onnxruntime FALSE)
  if(ENABLE_ONNXRUNTIME)
    find_package(onnxruntime QUIET)

    if(TARGET onnxruntime::onnxruntime)
      add_library(MLPlanner::onnxruntime ALIAS onnxruntime::onnxruntime)
      set(has_onnxruntime TRUE)
    elseif(TARGET onnxruntime)
      add_library(MLPlanner::onnxruntime ALIAS onnxruntime)
      set(has_onnxruntime TRUE)
    else()
      find_path(ML_PLANNER_ONNXRUNTIME_INCLUDE_DIR
        NAMES onnxruntime_cxx_api.h
        HINTS /usr/local/include /usr/local/include/onnxruntime
      )
      find_library(ML_PLANNER_ONNXRUNTIME_LIBRARY
        NAMES onnxruntime
        HINTS /usr/local/lib /usr/local/lib64
      )
      if(ML_PLANNER_ONNXRUNTIME_INCLUDE_DIR AND
         ML_PLANNER_ONNXRUNTIME_LIBRARY)
        add_library(MLPlanner::onnxruntime UNKNOWN IMPORTED)
        set_target_properties(MLPlanner::onnxruntime PROPERTIES
          IMPORTED_LOCATION "${ML_PLANNER_ONNXRUNTIME_LIBRARY}"
          INTERFACE_INCLUDE_DIRECTORIES "${ML_PLANNER_ONNXRUNTIME_INCLUDE_DIR}"
        )
        set(has_onnxruntime TRUE)
      endif()
    endif()

    if(has_onnxruntime)
      message(STATUS "ONNX Runtime is available; enabling ONNX Runtime inference support")
    else()
      message(WARNING
        "ONNX Runtime was not found; ONNX Runtime inference support will be disabled")
    endif()
  endif()

  set(ML_PLANNER_HAS_ONNXRUNTIME ${has_onnxruntime} PARENT_SCOPE)
endfunction()
