cmake_minimum_required(VERSION 3.3.2)
project(drone-swarm)
add_executable(Tutorial tutorial.cxx)

include_directories(include/c_library_v2)
file(GLOB_RECURSE SOURCES "src/*.cpp")

add_executable(drones ${SOURCES})