# CMakeList.txt : CMake project for Lab3-Graph, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

project ("Lab3-Graph")
set(CMAKE_CXX_STANDARD 23)


if(MSVC)
    if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
        string(REGEX REPLACE "/W[0-4]" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
    endif()
endif()

add_library(graph STATIC "include/graph.h" "include/graph.cpp")
target_include_directories(graph PUBLIC "include")
# Add source to this project's executable.

add_executable (Lab3-Graph "main.cpp")
target_link_libraries(Lab3-Graph graph)


# TODO: Add tests and install targets if needed.
