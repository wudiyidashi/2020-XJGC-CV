cmake_minimum_required(VERSION 3.0.0)
project(helloworld VERSION 0.1.0)

include(CTest)
enable_testing()

find_package( OpenCV REQUIRED )
aux_source_directory(. DIR_SRCS)

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${DIR_SRCS})

add_executable(helloworld ${DIR_SRCS})

target_link_libraries( ${PROJECT_NAME} PUBLIC ${OpenCV_LIBS} )

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
