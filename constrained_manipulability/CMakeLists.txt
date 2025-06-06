cmake_minimum_required(VERSION 3.5)
project(constrained_manipulability)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(kdl_parser REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_eigen_kdl REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(urdf QUIET)
if(urdf_FOUND)
  add_definitions(-DHAS_URDF)
  include_directories(${urdf_INCLUDE_DIRS})
endif()

find_package(constrained_manipulability_interfaces REQUIRED)
find_package(octomap_filter_interfaces REQUIRED)
find_package(robot_collision_checking REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)

find_package(Boost REQUIRED)
# find_package(rosidl_default_generators REQUIRED)

find_library(EIGEN_CDDLIB eigen-cddlib)

### Libraries and executables ###

include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

set(dependencies
  rclcpp
  rclpy
  kdl_parser
  geometry_msgs
  geometric_shapes
  octomap_msgs
  pcl_ros
  sensor_msgs
  shape_msgs
  std_msgs
  tf2
  tf2_eigen
  tf2_eigen_kdl
  tf2_ros
  tf2_kdl
  urdf
  visualization_msgs
  constrained_manipulability_interfaces
  octomap_filter_interfaces
  robot_collision_checking
)

add_library(${PROJECT_NAME} SHARED
  src/constrained_manipulability.cpp
  src/constrained_manipulability_mod.cpp
  src/constrained_manipulability_utils.cpp
  src/polytope.cpp
  src/path_collision_checking.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${Eigen3_LIBRARIES}
  ${EIGEN_CDDLIB}
  ${OCTOMAP_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

add_executable(constrained_manipulability_node
  src/constrained_manipulability_main.cpp
)
target_link_libraries(constrained_manipulability_node
  ${PROJECT_NAME}
)
ament_target_dependencies(constrained_manipulability_node
  ${dependencies}
)

add_executable(constrained_manipulability_node_mod
  src/constrained_manipulability_mod_main.cpp
)
target_link_libraries(constrained_manipulability_node_mod
  ${PROJECT_NAME}
)
ament_target_dependencies(constrained_manipulability_node_mod
  ${dependencies}
)

add_executable(path_collision_checking
  src/path_collision_checking_main.cpp
)
target_link_libraries(path_collision_checking
  ${PROJECT_NAME}
)
ament_target_dependencies(path_collision_checking
  ${dependencies}
)

add_executable(abstract_scene_example
  examples/abstract_scene_example.cpp
)
target_link_libraries(abstract_scene_example
  ${PROJECT_NAME}
)
ament_target_dependencies(abstract_scene_example
  ${dependencies}
)

add_executable(slicing_polytope_example
  examples/slicing_polytope_example.cpp
)
target_link_libraries(slicing_polytope_example
  ${PROJECT_NAME}
)
ament_target_dependencies(slicing_polytope_example
  ${dependencies}
)

add_executable(dynamic_collision_world_node
  src/dynamic_collision_world_node.cpp
)
target_link_libraries(dynamic_collision_world_node
  ${PROJECT_NAME}
)
ament_target_dependencies(dynamic_collision_world_node
  ${dependencies}
)
ament_target_dependencies(${PROJECT_NAME} geometric_shapes)

add_executable(dynamic_scene_node
  src/dynamic_scene_node.cpp
)
target_link_libraries(dynamic_scene_node
  ${PROJECT_NAME}
)
ament_target_dependencies(dynamic_scene_node
  ${dependencies}
)


target_link_libraries(${PROJECT_NAME} Boost::boost)
target_link_libraries(dynamic_collision_world_node Boost::boost)
target_link_libraries(${PROJECT_NAME} geometric_shapes::geometric_shapes)
target_link_libraries(dynamic_collision_world_node geometric_shapes::geometric_shapes)


### Install ###

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS constrained_manipulability_node abstract_scene_example slicing_polytope_example
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS dynamic_collision_world_node dynamic_scene_node constrained_manipulability_node_mod path_collision_checking
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

install(PROGRAMS 
  scripts/cm_client.py
  scripts/lin_limit_pub.py
  DESTINATION lib/${PROJECT_NAME}
)

### Testing ###

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Polytope class test
  ament_add_gtest(polytope_test tests/polytope_test.cpp)
  target_include_directories(polytope_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(polytope_test
    ${dependencies}
  )
  target_link_libraries(polytope_test ${PROJECT_NAME})
  # Utility functions unit tests
  ament_add_gtest(utils_test tests/utils_test.cpp)
  target_include_directories(utils_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(utils_test
    ${dependencies}
  )
  target_link_libraries(utils_test ${PROJECT_NAME})
endif()

### Ament stuff ###

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  ${dependencies}
  Eigen3
  EIGEN_CDDLIB
  octomap
)

ament_package()