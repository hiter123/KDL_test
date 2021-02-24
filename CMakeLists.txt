cmake_minimum_required(VERSION 3.5)
project(KDL_test)
set(CMAKE_CXX_STANDARD 11)
find_package(orocos_kdl REQUIRED)
include_directories(${orocos_kdl_INCLUDE_DIRS})
find_package(kdl_parser REQUIRED)
include_directories(${kdl_parser_INCLUDE_DIRS})
add_executable(${PROJECT_NAME}  main.cpp)
target_link_libraries(${PROJECT_NAME} ${orocos_kdl_LIBRARIES} ${kdl_parser_LIBRARIES})


