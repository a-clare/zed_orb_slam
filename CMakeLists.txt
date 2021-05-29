cmake_minimum_required(VERSION 3.10)
project(zed_orb_slam)

set(CMAKE_CXX_STANDARD 14)

add_subdirectory(Pangolin)

add_subdirectory(ORB_SLAM3/Thirdparty/DBoW2)
add_subdirectory(ORB_SLAM3/Thirdparty/g2o)
add_subdirectory(ORB_SLAM3)
add_subdirectory(Zed)
add_executable(zed_orb_slam main.cpp)