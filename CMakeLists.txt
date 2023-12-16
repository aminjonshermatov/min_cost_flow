cmake_minimum_required(VERSION 3.26)

set(PROJECT_NAME min_cost_flow)
project(${PROJECT_NAME} VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(GNUInstallDirs)

add_subdirectory(include)

target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_20)

include(CTest)
enable_testing()
add_subdirectory(tests)
add_subdirectory(third_party/googletest)


add_subdirectory(benchmarks)
add_subdirectory(third_party/benchmark)
