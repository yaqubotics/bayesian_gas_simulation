cmake_minimum_required(VERSION 2.8.3)
project(gaden_preprocessing_voronoi)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
    DEPENDS roscpp std_msgs nav_msgs geometry_msgs visualization_msgs EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_executable(preprocessing_voronoi src/preprocessing_voronoi.cpp)
add_dependencies(preprocessing_voronoi ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(preprocessing_save_distance src/preprocessing_save_distance.cpp)
add_dependencies(preprocessing_save_distance ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(preprocessing_voronoi
   ${catkin_LIBRARIES}
)

target_link_libraries(preprocessing_save_distance
   ${catkin_LIBRARIES}
)