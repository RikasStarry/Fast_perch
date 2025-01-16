cmake_minimum_required(VERSION 3.0.2)
project(sub_topic)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR} include
  CATKIN_DEPENDS roscpp 
  LIBRARIES ${PROJECT_NAME}
  DEPENDS EIGEN3
)

include_directories(
  SYSTEM
  include
  ${EIGEN3_INCLUDE_DIR}
  ${DECOMP_UTIL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(sub
    src/sub.cpp
)

target_link_libraries(sub
  ${catkin_LIBRARIES}
)

add_executable(sub_node src/sub_node.cpp )

target_link_libraries(sub_node
    sub
    ${catkin_LIBRARIES} 
)
