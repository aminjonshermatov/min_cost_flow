cmake_minimum_required(VERSION 3.26)

add_executable(tests test_sum.cpp)

target_include_directories(tests
        PUBLIC ../third_party/googletest/googletest/include
)

target_link_libraries(tests
        gtest
        ${PROJECT_NAME})
