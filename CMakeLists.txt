cmake_minimum_required(VERSION 3.14)
project(physics_benchmarks)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake_modules)
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(dart)
add_subdirectory(physx)
