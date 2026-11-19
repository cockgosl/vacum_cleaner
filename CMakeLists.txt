cmake_minimum_required(VERSION 3.16)
project(lidar LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(SRCS
    src/mapping/grid.cpp
    src/mapping/lidar.cpp
    src/rooting/scheduler.cpp
    src/cleaner/vacuum_cleaner.cpp
    src/localization/scan_matcher.cpp
)

add_library(lidar_lib STATIC ${SRCS})
target_include_directories(lidar_lib PUBLIC "${PROJECT_SOURCE_DIR}/include")

add_library(lidar_io STATIC
    src/io/stl_parser.cpp
    src/io/scan_assembler.cpp
)
target_include_directories(lidar_io PUBLIC "${PROJECT_SOURCE_DIR}/include")

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_sources(lidar_io PRIVATE src/io/serial_port.cpp)
endif()

option(BUILD_APP "Build the application" ON)

if(BUILD_APP)
    add_executable(lidar.out src/main.cpp)
    target_link_libraries(lidar.out PRIVATE lidar_lib lidar_io)
endif()

include(CTest)
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
