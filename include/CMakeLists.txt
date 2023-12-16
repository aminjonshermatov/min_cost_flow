cmake_minimum_required(VERSION 3.26)

file(GLOB projectSRC
    "*/*.cpp"
    "*/*.hpp"
    "*.cpp"
    "*.hpp"
)

set(CMAKE_CXX_FLAGS "-O3")

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE .)