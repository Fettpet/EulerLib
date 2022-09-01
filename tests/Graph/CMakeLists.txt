cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
project (GraphEulerTests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(EULER_BUILD_TEST ON)

if(${EULER_BUILD_TEST})
    file(
        GLOB_RECURSE
        GraphEuler-tests
        "*.test.cpp")
target_link_libraries(GraphEulerTest PUBLIC gtest_main EulerLib)

    add_executable(GraphEulerTest ${GraphEuler-tests})
    add_test(GraphEulerTest GraphEulerTest)
endif()
