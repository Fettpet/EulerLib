cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
include_guard(GLOBAL)

file(GLOB_RECURSE EulerLib_Headers *.h )

add_library(EulerLib INTERFACE ${EulerLib_Headers})
set(CMAKE_CXX_STANDARD 20)

# Add strict warning flags for GCC/Clang compilers
# Only enable these when using GNU or Clang-compatible compilers
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
   target_compile_options(EulerLib INTERFACE
      -Wall
      -Wextra
      -Werror
   )
endif()

target_include_directories(EulerLib INTERFACE
   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
   $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

install(TARGETS EulerLib
        EXPORT ${PROJECT_NAME}_Targets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})


