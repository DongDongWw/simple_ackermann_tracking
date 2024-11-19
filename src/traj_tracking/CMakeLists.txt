cmake_minimum_required(VERSION 3.0.2)
project(traj_tracking)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
)
find_package(Eigen3 REQUIRED)

add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/osqp)
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/osqp-eigen)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES traj_tracking
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs
)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

message(STATUS "catkin_INCLUDE_DIRS: ${catkin_INCLUDE_DIRS}")
message(STATUS "catkin_LIBRARIED: ${catkin_LIBRARIES}")

# add_executable(${PROJECT_NAME}_osqp
#   src/traj_tracking_osqp.cpp
# )
# target_link_libraries(traj_tracking_osqp
#   OsqpEigen::OsqpEigen
# )
add_executable(${PROJECT_NAME}_example
  src/example.cpp
  src/trajectory_tracker.cpp
)
target_link_libraries(${PROJECT_NAME}_example
  OsqpEigen::OsqpEigen
)

add_executable(${PROJECT_NAME}_gazebo
  src/traj_tracking_gazebo.cpp
  src/trajectory_tracker.cpp
)
target_link_libraries(${PROJECT_NAME}_gazebo
  ${catkin_LIBRARIES}
  OsqpEigen::OsqpEigen
)

