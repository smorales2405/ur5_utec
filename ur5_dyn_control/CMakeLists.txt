cmake_minimum_required(VERSION 3.8)
project(ur5_dyn_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Sin tipo de build, colcon compila SIN optimizacion, y eso aqui no es una
# preferencia: el lazo corre a 500 Hz y solveCare tarda 1.55 ms en Release
# contra 3.52 ms a -O0, o sea que a -O0 el LQR-SDRE NO cabe en su ciclo. El
# fallo es ademas SILENCIOSO —el tipo de build vive en la cache de CMake, asi
# que basta un `rm -rf build/` para perderlo sin que nada avise— y el unico
# sintoma es un test de tiempo que se cae. Se fija por defecto, sin pisar al
# que pida otro explicitamente en la linea de ordenes.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
  message(STATUS "ur5_dyn_control: CMAKE_BUILD_TYPE vacio -> Release")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# SHA del commit con el que se compila y si el paquete tenia cambios sin
# commitear, para la cabecera de trazabilidad de los CSV (FASE 3, B0.5). Se
# capturan en tiempo de BUILD —en ejecucion el nodo corre desde el install
# space, sin repositorio git— y en CADA build: el target `git_version`
# regenera git_version.hpp (cmake/git_version.cmake). Antes se capturaba al
# configurar, y colcon no reconfigura si este fichero no cambia.
set(GIT_VERSION_HPP ${CMAKE_CURRENT_BINARY_DIR}/generated/ur5_dyn_control/git_version.hpp)
add_custom_target(git_version ALL
  COMMAND ${CMAKE_COMMAND} -DSRC_DIR=${CMAKE_CURRENT_SOURCE_DIR}
          -DOUT=${GIT_VERSION_HPP} -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/git_version.cmake
  BYPRODUCTS ${GIT_VERSION_HPP}
  COMMENT "git_version.hpp (SHA y estado del arbol)")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
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
  src/diag_logger.cpp
  src/care_solver.cpp
  src/torque_control_node_base.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME}_core PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/generated)
add_dependencies(${PROJECT_NAME}_core git_version)
target_link_libraries(${PROJECT_NAME}_core PUBLIC
  ur5_kinematics::ur5_kinematics
  Eigen3::Eigen
  # Ver la nota en ur5_kinematics/CMakeLists.txt: en Pinocchio 4 las variables
  # PINOCCHIO_INCLUDE_DIRS / PINOCCHIO_LIBRARIES ya no bastan.
  pinocchio::pinocchio
)
ament_target_dependencies(${PROJECT_NAME}_core PUBLIC
  rclcpp sensor_msgs std_msgs geometry_msgs std_srvs controller_manager_msgs
  ament_index_cpp)

# ── Nodos de control ──────────────────────────────────────────────────────────
add_executable(gz_gravity_comp_node src/gz_gravity_comp_node.cpp)
target_link_libraries(gz_gravity_comp_node ${PROJECT_NAME}_core)

add_executable(gz_fl_control_node src/gz_fl_control_node.cpp)
target_link_libraries(gz_fl_control_node ${PROJECT_NAME}_core)

add_executable(gz_smc_control_node src/gz_smc_control_node.cpp)
target_link_libraries(gz_smc_control_node ${PROJECT_NAME}_core)

add_executable(gz_lqr_sdre_control_node src/gz_lqr_sdre_control_node.cpp)
target_link_libraries(gz_lqr_sdre_control_node ${PROJECT_NAME}_core)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS ${PROJECT_NAME}_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS gz_gravity_comp_node gz_fl_control_node gz_smc_control_node
                gz_lqr_sdre_control_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY urdf config launch worlds meshes DESTINATION share/${PROJECT_NAME})

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

  # FASE 4: solver CARE (funcion signo), estabilizabilidad y escalado por
  # inercia de la Q del LQR-SDRE.
  ament_add_gtest(test_care_solver test/test_care_solver.cpp)
  target_link_libraries(test_care_solver ${PROJECT_NAME}_core)
  # B4.2: LQI-SDRE (sintesis polinomica, pesos fisicos, anti-windup, coste).
  ament_add_gtest(test_lqi test/test_lqi.cpp)
  target_link_libraries(test_lqi ${PROJECT_NAME}_core)

  # Simetria de los launches: Gazebo solo valida el robot real si puede fijar
  # la MISMA configuracion. Cinco argumentos han faltado en un lado u otro y
  # cada vez la corrida de simulacion los ignoro en silencio.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_launch_arg_symmetry test/test_launch_arg_symmetry.py)
  # B0.2/B0.3/B0.5: arranca los tres nodos en dry_run (sin publicar par).
  ament_add_pytest_test(test_incision_common test/test_incision_common.py
    TIMEOUT 300)
  # B0.4: dinamica del nodo (C++) frente a la del evaluador offline (Python).
  add_executable(dump_dynamics test/dump_dynamics.cpp)
  target_link_libraries(dump_dynamics ${PROJECT_NAME}_core)
  ament_add_pytest_test(test_dynamics_cross test/test_dynamics_cross.py
    ENV DUMP_DYNAMICS=$<TARGET_FILE:dump_dynamics>
    TIMEOUT 120)
  # B4.3: ley SMC del nodo (C++) frente a la SmcLaw del evaluador (Python).
  add_executable(dump_smc_law test/dump_smc_law.cpp)
  target_link_libraries(dump_smc_law ${PROJECT_NAME}_core)
  ament_add_pytest_test(test_smc_law_cross test/test_smc_law_cross.py
    ENV DUMP_SMC_LAW=$<TARGET_FILE:dump_smc_law>
    TIMEOUT 120)
  # B4.2: la CARE de 18 estados frente a scipy.
  add_executable(dump_care test/dump_care.cpp)
  target_link_libraries(dump_care ${PROJECT_NAME}_core)
  ament_add_pytest_test(test_care_scipy test/test_care_scipy.py
    ENV DUMP_CARE=$<TARGET_FILE:dump_care>
    TIMEOUT 120)
endif()

ament_package()
