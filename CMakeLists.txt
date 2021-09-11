cmake_minimum_required(VERSION 3.2)

project(soundplaneclient)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

set(MLLIB_SOURCE_DIR "${CMAKE_SOURCE_DIR}/dep/madronalib")
set(SPLIB_SOURCE_DIR "${CMAKE_SOURCE_DIR}/dep/soundplanelib")
set(SPCLIENT_SOURCE_DIR "${CMAKE_SOURCE_DIR}/dep/soundplane")

add_subdirectory("${MLLIB_SOURCE_DIR}")
add_subdirectory("${SPLIB_SOURCE_DIR}")
# add_subdirectory("${SPCLIENT_SOURCE_DIR}")


add_subdirectory(src)
