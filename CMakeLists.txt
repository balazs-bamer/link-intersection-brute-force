cmake_minimum_required(VERSION 3.5)
project(link-intersection-brute-force)
set(CMAKE_VERBOSE_MAKEFILE on)

set(CMAKE_C_COMPILER "/usr/bin/gcc")
set(CMAKE_CXX_COMPILER "/usr/bin/g++")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

# Default to C++14
#if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
#endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Og -ggdb)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
#find_package(mx_joint_controller_msgs REQUIRED)


include_directories(
        ${gazebo_INCLUDE_DIRS}
        ${gazebo_ros_INCLUDE_DIRS}
        include/link-intersection-brute-force
        3rdparty/stl_reader
        3rdparty/cpp-logger
        3rdparty/cpp-memory-manager
        /usr/include/eigen3
)

add_library(gazebo_plugin SHARED src/gazebo-plugin.cpp)

ament_target_dependencies(gazebo_plugin
  "gazebo_ros"
  "rclcpp"
  "sensor_msgs")
#  "mx_joint_controller_msgs")


add_executable(linkIntersectionBruteForce
        src/DumpModel.cpp
        src/StlWriter.cpp
        src/EigenMeshModel.cpp
        src/main-node.cpp)
ament_target_dependencies(linkIntersectionBruteForce rclcpp urdf)
target_link_libraries(linkIntersectionBruteForce gazebo_plugin)    # hack
target_include_directories(linkIntersectionBruteForce PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
        )
#link_directories(${GAZEBO_LIBRARY_DIRS})

add_custom_command(TARGET gazebo_plugin POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:gazebo_plugin> share/${PROJECT_NAME})
#add_custom_command(TARGET linkIntersectionBruteForce POST_BUILD
#        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:linkIntersectionBruteForce> lib/${PROJECT_NAME})

add_custom_command(
        TARGET gazebo_plugin POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_SOURCE_DIR}/model.config
        ${CMAKE_CURRENT_BINARY_DIR}/../../share/${PROJECT_NAME}/model.config)

add_custom_command(
        TARGET gazebo_plugin POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_SOURCE_DIR}/worlds/empty.world
        ${CMAKE_CURRENT_BINARY_DIR}/../../share/${PROJECT_NAME}/empty.world)

add_custom_command(
        TARGET gazebo_plugin POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_SOURCE_DIR}/urdf/px150_coll.sdf
        ${CMAKE_CURRENT_BINARY_DIR}/../../share/${PROJECT_NAME}/px150_coll.sdf)

file(GLOB files "meshes/px150_meshes/*coll.stl")
foreach(file ${files})
  get_filename_component(name ${file} NAME)
  add_custom_command(
          TARGET gazebo_plugin POST_BUILD
          COMMAND ${CMAKE_COMMAND} -E copy
          ${file}
          ${CMAKE_CURRENT_BINARY_DIR}/../../share/${PROJECT_NAME}/meshes/px150_meshes/${name})
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
