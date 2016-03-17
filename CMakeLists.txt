cmake_minimum_required(VERSION 3.3)
project(assignment2)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DDEBUG -g -Wall -std=gnu++98")

include_directories(include)

add_executable (abt include/simulator.h src/simulator.cpp src/abt.cpp)
add_executable (gbn include/simulator.h src/simulator.cpp src/gbn.cpp)
add_executable (sr include/simulator.h src/simulator.cpp src/sr.cpp)
