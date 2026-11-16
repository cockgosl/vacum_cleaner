cmake_minimum_required(VERSION 3.16)
project(lidar LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(SRCS
    src/lidar/grid.cpp
)

add_library(lidar_lib STATIC ${SRCS})
target_include_directories(lidar_lib PUBLIC "${PROJECT_SOURCE_DIR}/include")

option(BUILD_APP "Build the application" ON)

if(BUILD_APP)
    add_executable(lidar.out src/main.cpp)
    target_link_libraries(lidar.out PRIVATE lidar_lib)
endif()
