cmake_minimum_required(VERSION 3.10)

project(nesemu VERSION 0.1)

if (WIN32)
    set(SDL2_DIR D:/Dev/Libraries/sdl2/devel)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED true)

file(GLOB NesEmu_SOURCE
    "src/*.cpp"
    "src/*.h"
)

find_package(SDL2 REQUIRED)
find_package(OpenGL REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS})

add_executable(nesemu ${NesEmu_SOURCE})
add_subdirectory(glad/)

if(WIN32)
    target_link_libraries(nesemu PRIVATE  ${SDL2_LIBRARIES} glad opengl32 ${CMAKE_DL_LIBS})
endif()
if(UNIX)
    target_link_libraries(nesemu PRIVATE stdc++fs ${SDL2_LIBRARIES} glad ${CMAKE_DL_LIBS})
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

    