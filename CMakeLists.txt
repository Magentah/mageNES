cmake_minimum_required(VERSION 3.10)

project(NesEmu VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED true)

file(GLOB NesEmu_SOURCE
    "src/*.cpp"
    "src/*.h"    
)

add_executable(nesemu "src/main.cpp" "src/cpu.h" "src/cpu.cpp" "src/rom.h" "src/rom.cpp" "src/ram.h" "src/ram.cpp")
target_link_libraries(nesemu stdc++fs)