cmake_minimum_required(VERSION 2.8.3)
project(aliengo_base)

 
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
 
find_package(PCL 1.7 REQUIRED)
find_package(catkin 
    REQUIRED 
    COMPONENTS 
    roscpp
    rospy
    visualization_msgs
    geometry_msgs
    sensor_msgs
    trajectory_msgs
    nav_msgs
    tf
    tf2_ros
    urdf
    aliengo_msgs
 grid_map_core
  grid_map_ros
  grid_map_cv
  grid_map_filters
  grid_map_loader
  grid_map_msgs
  grid_map_octomap
  grid_map_rviz_plugin
  grid_map_visualization
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES champ_base
    CATKIN_DEPENDS 
    roscpp
    rospy
    visualization_msgs
    geometry_msgs
    sensor_msgs
    trajectory_msgs
    nav_msgs
    tf
    tf2_ros
    urdf
    aliengo_msgs
)
 

install(DIRECTORY include/libchamp/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#Download libchamp
set(LIBCHAMP_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/include/libchamp/include")
set(HEADER_URL "https://github.com/chvmp/libchamp")

if (NOT EXISTS "${LIBCHAMP_HEADERS}")
  message(STATUS "Downloading libchamp...")
  message(STATUS ${CMAKE_CURRENT_SOURCE_DIR}/include)
  execute_process(
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include
    COMMAND git clone ${HEADER_URL}
  )
endif()
##end of libchamp download

include_directories(
  include
  ${LIBCHAMP_HEADERS}
  ${catkin_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
##end of auto download

add_library(quadruped_controller src/quadruped_controller.cpp)
add_executable(quadruped_controller_node src/quadruped_controller_node.cpp)
target_link_libraries(quadruped_controller_node quadruped_controller ${catkin_LIBRARIES})
add_dependencies(quadruped_controller_node ${catkin_EXPORTED_TARGETS})
add_dependencies(quadruped_controller  ${catkin_EXPORTED_TARGETS})


add_library(quadruped_foot_controller src/quadruped_foot_controller.cpp)
add_executable(quadruped_foot_controller_node src/quadruped_foot_controller_node.cpp)
target_link_libraries(quadruped_foot_controller_node quadruped_foot_controller ${catkin_LIBRARIES})
add_dependencies(quadruped_foot_controller_node ${catkin_EXPORTED_TARGETS})
add_dependencies(quadruped_foot_controller  ${catkin_EXPORTED_TARGETS})


add_library(quadruped_awareness_controller src/quadruped_awareness_controller.cpp)
add_executable(quadruped_awareness_controller_node src/quadruped_awareness_controller_node.cpp)
target_link_libraries(quadruped_awareness_controller_node quadruped_awareness_controller ${catkin_LIBRARIES})
add_dependencies(quadruped_awareness_controller_node ${catkin_EXPORTED_TARGETS})
add_dependencies(quadruped_awareness_controller  ${catkin_EXPORTED_TARGETS})





install(TARGETS 
    quadruped_controller
    quadruped_controller_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
