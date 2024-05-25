cmake_minimum_required(VERSION 3.26)

add_executable(${LIBRARY_NAME}_benchmarks min_cost_flow_benchmark.cpp)

target_include_directories(${LIBRARY_NAME}_benchmarks
        PUBLIC ../third-party/benchmark/include
)

target_link_libraries(${LIBRARY_NAME}_benchmarks
        ${LIBRARY_NAME}
        benchmark)