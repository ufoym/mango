project(test)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} CURRENT_SRC)
add_executable(${PROJECT_NAME} ${CURRENT_SRC})
target_link_libraries(${PROJECT_NAME} PRIVATE mango opencv_imgcodecs)