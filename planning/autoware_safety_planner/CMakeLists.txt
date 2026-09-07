cmake_minimum_required(VERSION 3.14)
project(autoware_safety_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

# 生成ヘッダは include/${PROJECT_NAME}/safety_planner_parameters.hpp に置かれる
generate_parameter_library(safety_planner_parameters
  param/safety_planner_parameter_struct.yaml
)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 周期入力 (PlannerContext) と射影ビュー ((s, l) 空間) の共通ヘルパー。
# node / rough_planner / trajectory_optimizer プラグインが共有する
# (プラグインの .so からも呼ぶので node 側ライブラリではなくここに置く)。
# SHARED にすると推移的な .so のリンクだけで数十分掛かるので STATIC
ament_auto_add_library(${PROJECT_NAME}_common STATIC
  src/context.cpp
  src/utils/sl_view_utils.cpp
  src/utils/trajectory_conversion.cpp
)
target_link_libraries(${PROJECT_NAME}_common
  safety_planner_parameters
)
target_compile_options(${PROJECT_NAME}_common PRIVATE -Wno-deprecated-declarations)

ament_auto_add_library(${PROJECT_NAME}_constraint_plugins SHARED
  src/constraint_generator/lane_following_drivable_area.cpp
  src/constraint_generator/obstacle_stop.cpp
  src/constraint_generator/simple_drivable_area.cpp
  src/constraint_generator/vehicle_kinematics.cpp
)
target_link_libraries(${PROJECT_NAME}_constraint_plugins
  safety_planner_parameters
  ${PROJECT_NAME}_common
)
target_compile_options(${PROJECT_NAME}_constraint_plugins PRIVATE -Wno-deprecated-declarations)


# ---------------------------------------------------------------------------------------------
# acados code generation (Python -> C)。NlpTrajectoryOptimizer が駆動する OCP の生成物。
# 構造 (ステージ数・行レイアウト・スラック・コストの型) だけが codegen 時に決まり、数値は
# すべて実行時に注入する (generators/nlp_time_ocp.py)。生成物はソースツリーへ書かない。
# ---------------------------------------------------------------------------------------------
set(ACADOS_SOURCE_DIR "$ENV{ACADOS_SOURCE_DIR}")
if(NOT ACADOS_SOURCE_DIR)
  set(ACADOS_SOURCE_DIR "/opt/acados")
endif()
find_package(acados REQUIRED)

set(NLP_MODEL_NAME rbp_nlp_time)
set(NLP_GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/c_generated_code_nlp_time")
set(NLP_GENERATED_FILES
  ${NLP_GEN_DIR}/acados_solver_${NLP_MODEL_NAME}.h
  ${NLP_GEN_DIR}/acados_solver_${NLP_MODEL_NAME}.c
  ${NLP_GEN_DIR}/${NLP_MODEL_NAME}_model/${NLP_MODEL_NAME}_model.h
  ${NLP_GEN_DIR}/${NLP_MODEL_NAME}_model/${NLP_MODEL_NAME}_expl_ode_fun.c
  ${NLP_GEN_DIR}/${NLP_MODEL_NAME}_model/${NLP_MODEL_NAME}_expl_vde_adj.c
  ${NLP_GEN_DIR}/${NLP_MODEL_NAME}_model/${NLP_MODEL_NAME}_expl_vde_forw.c
  ${NLP_GEN_DIR}/${NLP_MODEL_NAME}_constraints/${NLP_MODEL_NAME}_constraints.h
  ${NLP_GEN_DIR}/${NLP_MODEL_NAME}_constraints/${NLP_MODEL_NAME}_constr_h_fun.c
  ${NLP_GEN_DIR}/${NLP_MODEL_NAME}_constraints/${NLP_MODEL_NAME}_constr_h_fun_jac_uxt_zt.c
  ${NLP_GEN_DIR}/${NLP_MODEL_NAME}_constraints/${NLP_MODEL_NAME}_constr_h_e_fun.c
  ${NLP_GEN_DIR}/${NLP_MODEL_NAME}_constraints/${NLP_MODEL_NAME}_constr_h_e_fun_jac_uxt_zt.c
)
set_source_files_properties(${NLP_GENERATED_FILES} PROPERTIES GENERATED TRUE)

add_custom_command(
  OUTPUT ${NLP_GENERATED_FILES}
  COMMAND ${CMAKE_COMMAND} -E env
    "ACADOS_SOURCE_DIR=${ACADOS_SOURCE_DIR}"
    "PYTHONPATH=${ACADOS_SOURCE_DIR}/interfaces/acados_template:${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}"
    ${ACADOS_SOURCE_DIR}/.venv/bin/python3
    ${CMAKE_CURRENT_SOURCE_DIR}/generators/nlp_time_ocp.py
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/generators/nlp_time_ocp.py
    ${CMAKE_CURRENT_SOURCE_DIR}/generators/time_bicycle_model.py
  COMMENT "Generating the acados solver for NlpTrajectoryOptimizer"
)
add_custom_target(generate_acados_code_nlp_time DEPENDS ${NLP_GENERATED_FILES})

# 軌道プランナープラグイン (制約コンパイル・rough_planner・optimizer はこの .so の実装詳細)
ament_auto_add_library(${PROJECT_NAME}_trajectory_planner_plugins SHARED
  src/trajectory_planner/frenet_sampler/frenet_sampler.cpp
  src/trajectory_planner/nlp_planner/constraints_compiler.cpp
  src/trajectory_planner/nlp_planner/rough_planner.cpp
  src/trajectory_planner/nlp_planner/rough_optimizer_trajectory_planner.cpp
  src/trajectory_planner/nlp_planner/bezier.cpp
  src/trajectory_planner/nlp_planner/nlp_trajectory_optimizer.cpp
  src/trajectory_planner/nlp_planner/ssc_corridor.cpp
  src/trajectory_planner/nlp_planner/ssc_qp_trajectory_optimizer.cpp
  ${NLP_GENERATED_FILES}
)
add_dependencies(${PROJECT_NAME}_trajectory_planner_plugins generate_acados_code_nlp_time)
target_link_libraries(${PROJECT_NAME}_trajectory_planner_plugins
  safety_planner_parameters
  ${PROJECT_NAME}_common
  acados
)
target_include_directories(${PROJECT_NAME}_trajectory_planner_plugins PRIVATE
  # 生成ヘッダはプラグインの実装詳細 (pimpl) なので PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}
  ${ACADOS_SOURCE_DIR}/include
  ${ACADOS_SOURCE_DIR}/include/acados
  ${ACADOS_SOURCE_DIR}/include/acados_c
  ${ACADOS_SOURCE_DIR}/include/blasfeo/include
  ${ACADOS_SOURCE_DIR}/include/hpipm/include
)
target_compile_options(${PROJECT_NAME}_trajectory_planner_plugins PRIVATE -Wno-deprecated-declarations)

# 生成 C の警告は我々が直せるものではない
if(CMAKE_C_COMPILER_ID STREQUAL "GNU" OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  set_source_files_properties(${NLP_GENERATED_FILES} PROPERTIES
    COMPILE_OPTIONS "-Wno-unused-variable;-Wno-unused-parameter;-Wno-unused-function")
endif()

# libacados の推移的な依存 (blasfeo / hpipm) は acados のインストールツリーにしかないので、
# 呼び出し側が LD_LIBRARY_PATH を設定しなくても解決できるよう rpath を焼く
get_target_property(_acados_location acados IMPORTED_LOCATION)
if(NOT _acados_location)
  get_target_property(_acados_location acados IMPORTED_LOCATION_RELEASE)
endif()
if(_acados_location)
  get_filename_component(_acados_lib_dir "${_acados_location}" DIRECTORY)
else()
  set(_acados_lib_dir "${ACADOS_SOURCE_DIR}/lib")
endif()
set_target_properties(${PROJECT_NAME}_trajectory_planner_plugins PROPERTIES
  BUILD_RPATH "${_acados_lib_dir}"
  INSTALL_RPATH "${_acados_lib_dir}")

# libacados.so 自身は rpath を持たないので、その NEEDED (hpipm / blasfeo / qpOASES_e) は
# ldconfig の既定パス (/usr/local/lib) から拾われる。そこに別ビルドの libhpipm.so が居ると
# libacados と構造体レイアウトが食い違い、ソルバー生成で SIGSEGV する。
# プラグイン自身の DT_NEEDED に acados ツリーの実体を絶対パスで足し、
# DT_RPATH (--disable-new-dtags: LD_LIBRARY_PATH より先に引かれる) で固定して、
# libacados が読まれる前に必ず同じツリーの hpipm / blasfeo を載せる。
foreach(_acados_dep hpipm blasfeo qpOASES_e)
  if(EXISTS "${_acados_lib_dir}/lib${_acados_dep}.so")
    target_link_libraries(${PROJECT_NAME}_trajectory_planner_plugins
      "${_acados_lib_dir}/lib${_acados_dep}.so")
  endif()
endforeach()
target_link_options(${PROJECT_NAME}_trajectory_planner_plugins PRIVATE
  "LINKER:--disable-new-dtags")

pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  src/safety_planner.cpp
  src/safety_planner_node.cpp
)
target_link_libraries(${PROJECT_NAME}_lib
  safety_planner_parameters
  ${PROJECT_NAME}_common
)
# generate_parameter_library の生成ヘッダが deprecated な parameter_traits の別名を使う
target_compile_options(${PROJECT_NAME}_lib PRIVATE -Wno-deprecated-declarations)

rclcpp_components_register_node(${PROJECT_NAME}_lib
  PLUGIN "autoware::safety_planner::SafetyPlannerNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
