cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
project (EulerLib LANGUAGES CXX)
include_guard(GLOBAL)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
option(EULER_BUILD_TEST ON)
option(EULER_BUILD_EXAMPLE OFF)

add_subdirectory(src)

if(${EULER_BUILD_TEST})
    include(CMake/googletest.cmake)
    enable_testing()
    add_subdirectory(tests)
endif()
