
cmake_minimum_required(VERSION 3.20)

project(LC_rotations_3d LANGUAGES CXX)
message(STATUS "Project: ${PROJECT_NAME} ${PROJECT_VERSION} (${CMAKE_BUILD_TYPE})")

set(LC_rotations_3d_MAIN_PROJECT OFF)
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    set(LC_rotations_3d_MAIN_PROJECT ON)
endif()

include(FetchContent)

add_library(LC_rotations_3d INTERFACE)

target_include_directories(LC_rotations_3d INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

set_target_properties(LC_rotations_3d PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

FetchContent_Declare(
    LC_hypercomplex
    GIT_REPOSITORY https://github.com/LucaCiucci/LC_hypercomplex
    GIT_TAG        origin/main
)

FetchContent_MakeAvailable(LC_hypercomplex)

target_link_libraries(LC_rotations_3d INTERFACE LC_hypercomplex)

if(LC_rotations_3d_MAIN_PROJECT)
    add_subdirectory(examples)
endif()