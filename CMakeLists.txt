cmake_minimum_required(VERSION 3.19)
project(zed_orb_slam)

set(CMAKE_CXX_STANDARD 14)

add_subdirectory(Pangolin)

add_executable(zed_orb_slam main.cpp)