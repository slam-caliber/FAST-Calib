cmake_minimum_required(VERSION 3.0.2)
<<<<<<< HEAD
project(fast_calib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  geometry_msgs
)

find_package(PCL 1.10 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs pcl_conversions pcl_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(CMAKE_BUILD_TYPE Release)

if (CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif()

add_executable(fast_calib src/main.cpp)

target_link_libraries(fast_calib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(multi_fast_calib src/multi_scene.cpp)
target_link_libraries(multi_fast_calib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
=======
project(my_fast_calib)
set(CMAKE_CXX_FLAGS "-g3 -O -Wall")
set(CMAKE_CXX_STANDARD 20)

find_package(gflags REQUIRED)
find_package(glog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    /opt/third_party/toml11-3.7.1/include
    )
include_directories(${PROJECT_SOURCE_DIR}/src)

set(CMAKE_BUILD_TYPE Release)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif()

add_compile_options(-Wno-sign-compare)

add_executable(my_fast_calib
    src/main.cpp)
target_link_libraries(my_fast_calib gflags yaml-cpp glog ${PCL_LIBRARIES} ${OpenCV_LIBS})
>>>>>>> master
