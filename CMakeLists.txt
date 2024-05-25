cmake_minimum_required(VERSION 3.26)

set(LIBRARY_NAME libmcf)
project(${LIBRARY_NAME} VERSION 1.0.0 LANGUAGES CXX)

include(cmake/Ccache.cmake)
include(cmake/CompileOptions.cmake)

include(GNUInstallDirs)

add_subdirectory(include)

option(MCF_TESTS "Build MCF tests" OFF)
option(MCF_BENCHMARKS "Build MCF benchmarks" OFF)
option(MCF_DISABLE_VERIFY "Disable verify/asserts" OFF)

if (MCF_TESTS)
    add_subdirectory(third_party/googletest)
    include(CTest)
    enable_testing()
    add_subdirectory(tests)
endif ()

if (MCF_BENCHMARKS)
    add_subdirectory(third_party/googlebenchmark)
    add_subdirectory(benchmarks)
endif ()