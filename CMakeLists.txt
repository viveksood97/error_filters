project(error_filters)
cmake_minimum_required(VERSION 3.16)
find_package(Python3 COMPONENTS Intrepreter Development NumPy)
include_directories(${Python3_INCLUDE_DIRS} include)
add_subdirectory(src)

target_link_libraries(result ${Python3_LIBRARIES})