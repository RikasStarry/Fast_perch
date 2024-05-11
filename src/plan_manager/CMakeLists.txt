cmake_minimum_required(VERSION 3.0.2)
project(plan_manager)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  traj_opt
  vis_utils
  traj_utils
  message_generation
  uneven_map
)

catkin_package(
  INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR}
  CATKIN_DEPENDS traj_opt roscpp traj_utils uneven_map
  LIBRARIES ${PROJECT_NAME}
  DEPENDS EIGEN3
)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${DECOMP_UTIL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# planning node
add_library(plan_manager
    src/plan_manager.cpp
)

target_link_libraries(plan_manager
  ${catkin_LIBRARIES}
)

add_executable(manager_node src/manager_node.cpp )

target_link_libraries(manager_node
    plan_manager
    ${catkin_LIBRARIES} 
)

add_executable(traj_server src/traj_server.cpp)
target_link_libraries(traj_server ${catkin_LIBRARIES})
