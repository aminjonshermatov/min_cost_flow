cmake_minimum_required(VERSION 3.26)

add_library(${LIBRARY_NAME} INTERFACE)
target_include_directories(${LIBRARY_NAME} INTERFACE "${PROJECT_SOURCE_DIR}/include")
target_compile_features(${LIBRARY_NAME} INTERFACE cxx_std_23)
target_include_directories(${LIBRARY_NAME} INTERFACE include/)
