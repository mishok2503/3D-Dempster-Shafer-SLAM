cmake_minimum_required(VERSION 3.26)
project(DSSLAM)

set(CMAKE_CXX_STANDARD 20)

add_executable(DSSLAM main.cpp)

target_include_directories(DSSSLAM PRIVATE include)
