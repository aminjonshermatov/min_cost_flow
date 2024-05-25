cmake_minimum_required(VERSION 3.26)

include(GoogleTest)
add_executable(${LIBRARY_NAME}_tests test_min_cost_flow.cpp)

target_include_directories(${LIBRARY_NAME}_tests
        PUBLIC ../third_party/googletest/googletest/include
)

target_link_libraries(${LIBRARY_NAME}_tests
        ${LIBRARY_NAME}
        gtest)

gtest_discover_tests(${LIBRARY_NAME}_tests)
