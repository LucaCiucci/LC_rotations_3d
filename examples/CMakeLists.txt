
add_executable(${PROJECT_NAME}_example main.cpp)

target_link_libraries(${PROJECT_NAME}_example ${PROJECT_NAME})

set_target_properties(${PROJECT_NAME}_example PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)