cmake_minimum_required(VERSION 3.8)
project(ur5_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# Pinocchio 4 exporta el directorio de cabeceras de COMPATIBILIDAD
# (include/pinocchio/deprecated, donde vive <pinocchio/multibody/model.hpp>)
# unicamente a traves del target importado pinocchio::pinocchio; la variable
# heredada PINOCCHIO_INCLUDE_DIRS solo apunta a include/ y PINOCCHIO_LIBRARIES
# quedo VACIA. Por eso hay que enlazar contra el target, no contra las
# variables. El target existe tambien en Pinocchio 2 y 3, asi que esto no ata
# el paquete a la version 4.
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/kinematics.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OsqpEigen_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  pinocchio::pinocchio
  Eigen3::Eigen
  OsqpEigen::OsqpEigen
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(pinocchio Eigen3 OsqpEigen)

install(EXPORT export_${PROJECT_NAME}
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_package()
