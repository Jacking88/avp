cmake_minimum_required(VERSION 2.8.3)
project(avp_mapping)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fPIC -Wno-dev")
add_definitions(-DDATASET_PATH="${PROJECT_SOURCE_DIR}/ipm_data/")

include(cmake/FindEigen.cmake)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME} SHARED
    src/pose_interpolation.cc
    src/avp_mapping.cc
    src/map.cc
    src/slot.cc
)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

add_executable(avp_mapping_cv src/avp_mapping_main.cc)
target_link_libraries(avp_mapping_cv ${PROJECT_NAME})

if ("$ENV{ROS_VERSION}" STREQUAL "1")
    message(STATUS "Build ros node")
    find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs nav_msgs tf)
    include_directories(${catkin_INCLUDE_DIRS})

    catkin_package()
    add_executable(avp_mapping_node src/avp_mapping_node_main.cc)
    target_link_libraries(avp_mapping_node ${PROJECT_NAME} ${catkin_LIBRARIES})
endif ()
