cmake_minimum_required(VERSION 3.5)
project(Strategy)
include_directories(${CMAKE_SOURCE_DIR})

set(ALL_STRATEGES 
    Scaratest/Scaratest.cpp
    Carrot/Carrot.cpp
    May/BoxMoving.cpp
# Strategies/Demo/Demo.cpp
)

add_library(Strategy SHARED Strategy.cpp ${ALL_STRATEGES}) 
target_link_libraries(Strategy Robot Scara May)