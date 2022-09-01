cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
project (GraphEulerTests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(
   GLOB_RECURSE
   GraphEuler-tests
   "*.test.cpp")

add_executable(GraphEulerTest ${GraphEuler-tests})
target_link_libraries(GraphEulerTest PUBLIC gtest_main EulerLib)
add_test(GraphEulerTest GraphEulerTest)

