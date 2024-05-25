set(EXAMPLE_TARGET ${LIBRARY_NAME}_examples)

add_executable(${EXAMPLE_TARGET} main.cpp)
target_link_libraries(${EXAMPLE_TARGET} ${LIBRARY_NAME})
