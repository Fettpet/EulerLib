cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
project (SetEulerTests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(
   GLOB_RECURSE
   SetEuler-tests
   "*.test.cpp")

add_executable(SetEulerTests ${SetEuler-tests})
target_link_libraries(SetEulerTests PUBLIC gtest_main EulerLib)
add_test(SetEulerTests SetEulerTests)

