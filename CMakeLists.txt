cmake_minimum_required(VERSION 3.2.3)
project (claw)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

include(CMakeToolsHelpers OPTIONAL)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 14)

add_executable(claw src/main.cpp)
target_link_libraries(claw)
include_directories()
