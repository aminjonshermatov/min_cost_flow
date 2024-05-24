cmake_minimum_required(VERSION 3.26)

include(GoogleTest)
add_executable(${PROJECT_NAME}_tests test_min_cost_flow.cpp)

target_include_directories(${PROJECT_NAME}_tests
        PUBLIC ../third_party/googletest/googletest/include
)

target_link_libraries(${PROJECT_NAME}_tests
        libmcf
        gtest)

gtest_discover_tests(${PROJECT_NAME}_tests)
