project(test)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} CURRENT_SRC)
add_executable(${PROJECT_NAME} ${CURRENT_SRC})
target_link_libraries(${PROJECT_NAME} PRIVATE mango)