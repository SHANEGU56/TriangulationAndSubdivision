cmake_minimum_required(VERSION 3.9)
project(landmarks_triangulation)

#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE debug)

set(CMAKE_CXX_COMPILER "/usr/bin/g++")

#set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS "-std=c++11 -stdlib=libc++")

# add src
add_subdirectory(src)