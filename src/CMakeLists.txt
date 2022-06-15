cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
include_guard(GLOBAL)

file(GLOB_RECURSE EulerLib_Headers *.h )

add_library(EulerLib INTERFACE ${EulerLib_Headers})
set(CMAKE_CXX_STANDARD 20)

    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/> 
target_include_directories(EulerLib INTERFACE
)


