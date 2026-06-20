cmake_minimum_required(VERSION 3.8)
project(ur5_pick_place)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ur5_kinematics REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(OsqpEigen REQUIRED)

# ── IK wrapper library ────────────────────────────────────────────────────────
add_library(ik_wrapper SHARED
  src/ik_wrapper.cpp
)
target_include_directories(ik_wrapper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PINOCCHIO_INCLUDE_DIRS}
)
ament_target_dependencies(ik_wrapper PUBLIC rclcpp)
target_link_libraries(ik_wrapper PUBLIC
  ur5_kinematics::ur5_kinematics
  Eigen3::Eigen
  ${PINOCCHIO_LIBRARIES}
)

# ── Trajectory generator library ─────────────────────────────────────────────
add_library(trajectory_generator SHARED
  src/trajectory_generator.cpp
)
target_include_directories(trajectory_generator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(trajectory_generator PUBLIC Eigen3::Eigen)

# ── Main node ─────────────────────────────────────────────────────────────────
add_executable(pick_place_node src/pick_place_node.cpp)
target_include_directories(pick_place_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${PINOCCHIO_INCLUDE_DIRS}
)
target_link_libraries(pick_place_node
  ik_wrapper
  trajectory_generator
  Eigen3::Eigen
)
ament_target_dependencies(pick_place_node
  rclcpp rclcpp_action control_msgs trajectory_msgs sensor_msgs ament_index_cpp)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS ik_wrapper trajectory_generator pick_place_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch urdf worlds meshes DESTINATION share/${PROJECT_NAME})

ament_package()
