include(FetchContent)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.17.0
    GIT_SHALLOW TRUE
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

include(GoogleTest)

add_executable(grid_tests grid_tests.cpp)
set_target_properties(grid_tests PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
target_link_libraries(grid_tests PRIVATE lidar_lib GTest::gtest_main)
target_compile_options(grid_tests PRIVATE -Wall -Wextra -Wpedantic)

gtest_discover_tests(grid_tests PROPERTIES TIMEOUT 10 LABELS "grid")
