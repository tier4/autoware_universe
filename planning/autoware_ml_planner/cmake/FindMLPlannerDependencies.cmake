include_guard(GLOBAL)

function(ml_planner_find_dependencies)
  set(ML_PLANNER_CUDA_AVAIL OFF)
  set(ML_PLANNER_TRT_AVAIL OFF)
  set(has_onnxruntime FALSE)

  # Use legacy FindCUDA (not CUDAToolkit alone): on CPU-only hosts FindCUDAToolkit can
  # fatal-error when nvcc is missing, while FindCUDA only sets CUDA_FOUND to FALSE
  # (same pattern as autoware_tensorrt_common and autoware_diffusion_planner).
  find_package(CUDA)
  if(CUDA_FOUND)
    find_library(ML_PLANNER_CUBLAS_LIBRARIES cublas HINTS
      ${CUDA_TOOLKIT_ROOT_DIR}/lib64
      ${CUDA_TOOLKIT_ROOT_DIR}/lib
    )
    unset(CUDA_cublas_device_LIBRARY CACHE)
    set(ML_PLANNER_CUDA_AVAIL ON)
  else()
    message(WARNING "CUDA NOT FOUND")
  endif()

  if(ML_PLANNER_CUDA_AVAIL)
    find_library(ML_PLANNER_NVINFER_LIBRARY NAMES nvinfer)
    find_library(ML_PLANNER_NVONNXPARSER_LIBRARY NAMES nvonnxparser)
    if(ML_PLANNER_NVINFER_LIBRARY AND ML_PLANNER_NVONNXPARSER_LIBRARY)
      set(ML_PLANNER_TRT_AVAIL ON)

      add_library(MLPlanner::nvinfer UNKNOWN IMPORTED)
      set_target_properties(MLPlanner::nvinfer PROPERTIES
        IMPORTED_LOCATION "${ML_PLANNER_NVINFER_LIBRARY}"
      )
      add_library(MLPlanner::nvonnxparser UNKNOWN IMPORTED)
      set_target_properties(MLPlanner::nvonnxparser PROPERTIES
        IMPORTED_LOCATION "${ML_PLANNER_NVONNXPARSER_LIBRARY}"
      )

      if(ML_PLANNER_VERBOSE_DEPENDENCIES)
        message(STATUS "CUDA toolkit root: ${CUDA_TOOLKIT_ROOT_DIR}")
        message(STATUS "CUDA libraries: ${CUDA_LIBRARIES}")
        message(STATUS "cuBLAS library: ${ML_PLANNER_CUBLAS_LIBRARIES}")
        message(STATUS "TensorRT inference library: ${ML_PLANNER_NVINFER_LIBRARY}")
        message(STATUS "TensorRT ONNX parser: ${ML_PLANNER_NVONNXPARSER_LIBRARY}")
      endif()
    else()
      message(WARNING "TensorRT is NOT Available")
    endif()
  endif()

  if(ML_PLANNER_CUDA_AVAIL AND ML_PLANNER_TRT_AVAIL AND ENABLE_ONNXRUNTIME)
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

  set(ML_PLANNER_CUDA_AVAIL ${ML_PLANNER_CUDA_AVAIL} PARENT_SCOPE)
  set(ML_PLANNER_TRT_AVAIL ${ML_PLANNER_TRT_AVAIL} PARENT_SCOPE)
  set(ML_PLANNER_HAS_ONNXRUNTIME ${has_onnxruntime} PARENT_SCOPE)
  set(ML_PLANNER_CUBLAS_LIBRARIES ${ML_PLANNER_CUBLAS_LIBRARIES} PARENT_SCOPE)
endfunction()
