set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../lib)

find_package( OpenCV REQUIRED )
aux_source_directory(. DIR_LIB_SRCS)
add_library(Maze SHARED ${DIR_LIB_SRCS})
target_link_libraries( Maze ${OpenCV_LIBS} )