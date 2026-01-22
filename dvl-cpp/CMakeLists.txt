cmake_minimum_required(VERSION 3.10.0)
project(dvl_cpp VERSION 0.1.0 LANGUAGES C CXX)

# Library
add_library(dvl
    src/dvl.cpp
)

target_include_directories(dvl
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

# Example executable
add_executable(dvl_example
    example.cpp
)

target_link_libraries(dvl_example
    PRIVATE dvl)

