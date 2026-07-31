cmake_minimum_required(VERSION 3.8)
project(ur5_dyn_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# SHA del commit con el que se compila, para la cabecera de trazabilidad de los
# CSV (FASE 3). Se captura en tiempo de BUILD: en tiempo de ejecucion el nodo
# corre desde el install space, donde no hay repositorio git que consultar.
execute_process(
  COMMAND git -C ${CMAKE_CURRENT_SOURCE_DIR} rev-parse --short HEAD
  OUTPUT_VARIABLE UR5_DYN_CONTROL_GIT_SHA
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET)
if(NOT UR5_DYN_CONTROL_GIT_SHA)
  set(UR5_DYN_CONTROL_GIT_SHA "unknown")
endif()
add_compile_definitions(UR5_DYN_CONTROL_GIT_SHA="${UR5_DYN_CONTROL_GIT_SHA}")
message(STATUS "ur5_dyn_control git SHA: ${UR5_DYN_CONTROL_GIT_SHA}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(ur5_kinematics REQUIRED)

# ── Libreria compartida (spline, dinamica, referencias, switcher, base) ──────
add_library(${PROJECT_NAME}_core SHARED
  src/cartesian_spline_trajectory.cpp
  src/quintic_spline.cpp
  src/quintic_spline_trajectory.cpp
  src/arc_length.cpp
  src/time_profile.cpp
  src/incision_trajectory.cpp
  src/joint_sweep_generator.cpp
  src/ur5_dynamics.cpp
  src/joint_reference_generator.cpp
  src/controller_switcher.cpp
  src/csv_logger.cpp
  src/torque_control_node_base.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PINOCCHIO_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_core PUBLIC
  ur5_kinematics::ur5_kinematics
  Eigen3::Eigen
  ${PINOCCHIO_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}_core PUBLIC
  rclcpp sensor_msgs std_msgs controller_manager_msgs ament_index_cpp)

# ── Nodos de control ──────────────────────────────────────────────────────────
add_executable(gz_gravity_comp_node src/gz_gravity_comp_node.cpp)
target_link_libraries(gz_gravity_comp_node ${PROJECT_NAME}_core)

add_executable(gz_fl_control_node src/gz_fl_control_node.cpp)
target_link_libraries(gz_fl_control_node ${PROJECT_NAME}_core)

add_executable(gz_smc_control_node src/gz_smc_control_node.cpp)
target_link_libraries(gz_smc_control_node ${PROJECT_NAME}_core)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS ${PROJECT_NAME}_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS gz_gravity_comp_node gz_fl_control_node gz_smc_control_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY urdf config launch worlds DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/activate_and_unpause.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY docs DESTINATION share/${PROJECT_NAME} OPTIONAL)

# ── Tests ─────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Compuerta G3: la gravedad NO va en el comando de torque del robot real.
  ament_add_gtest(test_gravity_policy test/test_gravity_policy.cpp)
  target_link_libraries(test_gravity_policy ${PROJECT_NAME}_core)

  # Supuestos A1 (hook de herramienta) y A2 (offset del TCP configurable).
  ament_add_gtest(test_tool_inertia_hook test/test_tool_inertia_hook.cpp)
  target_link_libraries(test_tool_inertia_hook ${PROJECT_NAME}_core)

  # FASE 1: spline quintico (derivadas analiticas, continuidad de jerk).
  ament_add_gtest(test_quintic_spline test/test_quintic_spline.cpp)
  target_link_libraries(test_quintic_spline ${PROJECT_NAME}_core)

  # FASE 3: limite de tasa, marcas de saturacion y hash de trazabilidad.
  ament_add_gtest(test_limits_and_traceability test/test_limits_and_traceability.cpp)
  target_link_libraries(test_limits_and_traceability ${PROJECT_NAME}_core)

  # FASE 1: Gauss-Legendre, longitud de arco, S-curve y trayectoria de incision.
  ament_add_gtest(test_incision_trajectory test/test_incision_trajectory.cpp)
  target_link_libraries(test_incision_trajectory ${PROJECT_NAME}_core)
endif()

ament_package()
