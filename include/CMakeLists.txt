cmake_minimum_required(VERSION 3.26)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE "${PROJECT_SOURCE_DIR}/include")
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_23)
target_include_directories(${PROJECT_NAME} INTERFACE include/)
