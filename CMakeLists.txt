cmake_minimum_required(VERSION 3.14)
project(vl_traj_alignment)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_BUILD_TYPE "release")

find_package(Eigen3 3.3.7 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV 4.2 REQUIRED COMPONENTS)

find_package(
  beam REQUIRED 
  COMPONENTS 
    calibration
    cv
    depth
    utils
    matching
    mapping
    filtering
    optimization
)

set(catkin_build_depends
  tf2
  rosbag
  roscpp
  sensor_msgs
  geometry_msgs
)

find_package(
  catkin REQUIRED
  COMPONENTS
    ${catkin_build_depends}
)


catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS 
    ${catkin_build_depends}
  DEPENDS
    Boost
    CERES
    EIGEN3
    PCL
)


###########
## Build ##
###########
add_library(
  ${PROJECT_NAME}
  src/pose_lookup.cpp
  src/Scancontext.cpp
)

include_directories(
  include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_LIBRARY_DIRS}
)

target_link_libraries(
  ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    beam::utils
    beam::matching
    beam::filtering
    beam::mapping
    beam::calibration
    beam::cv
    beam::depth
    beam::optimization
)

#################
## Executables ##
#################
add_executable(${PROJECT_NAME}_main
	src/main.cpp
)
add_dependencies(${PROJECT_NAME}_main
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(${PROJECT_NAME}_main
  PUBLIC
    ${PROJECT_NAME}
)
target_link_libraries(${PROJECT_NAME}_main
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_bag_to_traj
	src/bag_to_traj.cpp
)
add_dependencies(${PROJECT_NAME}_bag_to_traj
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(${PROJECT_NAME}_bag_to_traj
  PUBLIC
    ${PROJECT_NAME}
)
target_link_libraries(${PROJECT_NAME}_bag_to_traj
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_con_slam_to_bag
	src/con_slam_to_bag.cpp
)
add_dependencies(${PROJECT_NAME}_con_slam_to_bag
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(${PROJECT_NAME}_con_slam_to_bag
  PUBLIC
    ${PROJECT_NAME}
)
target_link_libraries(${PROJECT_NAME}_con_slam_to_bag
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_aloam_to_json
	src/aloam_to_json.cpp
)
add_dependencies(${PROJECT_NAME}_aloam_to_json
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(${PROJECT_NAME}_aloam_to_json
  PUBLIC
    ${PROJECT_NAME}
)
target_link_libraries(${PROJECT_NAME}_aloam_to_json
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_highlight_changes
	src/highlight_changes.cpp
)
add_dependencies(${PROJECT_NAME}_highlight_changes
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(${PROJECT_NAME}_highlight_changes
  PUBLIC
    ${PROJECT_NAME}
)
target_link_libraries(${PROJECT_NAME}_highlight_changes
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)