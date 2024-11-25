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

# # protobuffer
find_package(Protobuf 3 REQUIRED)
message(STATUS "Protobuf compiler: ${PROTOBUF_PROTOC_EXECUTABLE}")
message(STATUS "[PROTOBUF_INCLUDE_DIRS]" ${Protobuf_INCLUDE_DIRS})

SET(PROTO_DIR ${PROJECT_SOURCE_DIR}/proto/)
SET(PB_DIR ${PROJECT_SOURCE_DIR}/include/proto)
message("[PROTO_DIR]" ${PROTO_DIR})

LIST(APPEND PROTO_FLAGS -I${PROTO_DIR})
file(GLOB_RECURSE MSG_PROTOS ${PROTO_DIR}/*.proto)

foreach(msg ${MSG_PROTOS})
  get_filename_component(FIL_NAME ${msg} NAME_WE)
  get_filename_component(FILE_PATH_ABS ${msg} ABSOLUTE)
  string(REPLACE "${PROTO_DIR}" "" FILE_PATH_REL ${FILE_PATH_ABS})
  string(REPLACE ".proto" ".pb.cc" FILE_NAME_CC ${FILE_PATH_REL})
  string(REPLACE ".proto" ".pb.h" FILE_NAME_H ${FILE_PATH_REL})
  message("[pb file absolute path] " ${PB_DIR}/${FILE_NAME_H})
  message("[pb file absolute path] " ${PB_DIR}/${FILE_NAME_CC})
  add_custom_command(
    OUTPUT
    "${PB_DIR}/${FILE_NAME_H}"
    "${PB_DIR}/${FILE_NAME_CC}"
    COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS
    --cpp_out ${PB_DIR}
    -I ${PROTO_DIR}
    ${msg}
    DEPENDS ${msg}
    COMMENT "Running C++ protocol buffer compiler on ${msg}"
    VERBATIM
  )

  set(PROTO_SRC ${PROTO_SRC} ${PB_DIR}/${FILE_NAME_CC})
  set(PROTO_INC ${PROTO_INC} ${PB_DIR}/${FILE_NAME_H})
endforeach()

set_source_files_properties(${PROTO_SRC} ${PROTO_INC} PROPERTIES GENERATED TRUE)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES traj_tracking
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs
)

# build target
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${PB_DIR}
)
include_directories(${Protobuf_INCLUDE_DIRS})
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
  ${PROTO_SRC}
  ${PROTO_INC}
)
target_link_libraries(${PROJECT_NAME}_gazebo
  ${catkin_LIBRARIES}
  OsqpEigen::OsqpEigen
  protobuf::libprotobuf
)
