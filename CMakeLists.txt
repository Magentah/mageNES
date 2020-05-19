cmake_minimum_required(VERSION 3.10)

project(magenes VERSION 0.1)

if (WIN32)
    set(SDL2_DIR D:/Dev/Libraries/sdl2/devel)
endif()
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMAKE")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED true)

file(GLOB magenes_SOURCE
    "src/*.cpp"
    "src/*.h"
    "src/imgui/*.cpp"
    "src/imgui/*.h"
)

find_package(SDL2 REQUIRED)
find_package(OpenGL REQUIRED)
include_directories(${SDL2_INCLUDE_DIR})

add_executable(magenes ${magenes_SOURCE})
add_subdirectory(glad/)

if(WIN32)
    target_link_libraries(magenes PRIVATE  ${SDL2_LIBRARY} glad opengl32 ${CMAKE_DL_LIBS})
endif()
if (APPLE)
    target_link_libraries(magenes PRIVATE ${SDL2_LIBRARY} glad ${CMAKE_DL_LIBS})
endif()
if(UNIX AND NOT APPLE)
    target_link_libraries(magenes PRIVATE stdc++fs ${SDL2_LIBRARY} glad ${CMAKE_DL_LIBS})
endif()

# Copy dlls
if(WIN32)
	add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_directory
		"${PROJECT_SOURCE_DIR}/dlls"
		$<TARGET_FILE_DIR:${PROJECT_NAME}>)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_directory
		"${PROJECT_SOURCE_DIR}/tests"
		$<TARGET_FILE_DIR:${PROJECT_NAME}>/tests)
endif()

    