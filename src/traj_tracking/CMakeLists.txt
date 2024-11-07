cmake_minimum_required(VERSION 3.0.2)
project(traj_tracking)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/osqp)
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/osqp-eigen)

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES traj_tracking
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

message(STATUS "catkin_INCLUDE_DIRS: ${catkin_INCLUDE_DIRS}")

add_executable(${PROJECT_NAME}_osqp
  src/traj_tracking_osqp.cpp
)

target_link_libraries(traj_tracking_osqp
  OsqpEigen::OsqpEigen
)
