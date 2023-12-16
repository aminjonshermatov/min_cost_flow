cmake_minimum_required(VERSION 3.26)

add_executable(${PROJECT_NAME}_benchmarks min_cost_flow_benchmark.cpp)

target_include_directories(${PROJECT_NAME}_benchmarks
        PUBLIC ../third-party/benchmark/include
        PUBLIC ../include
)

target_link_libraries(${PROJECT_NAME}_benchmarks
        benchmark
        ${PROJECT_NAME})
