cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
include_guard(GLOBAL)

add_library(Euler INTERFACE)
set(CMAKE_CXX_STANDARD 20)

target_include_directories(Euler INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/> 
)


