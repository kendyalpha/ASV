# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.8)

# 项目信息
project (testGPS)
set(CMAKE_CXX_STANDARD 17)


# 添加 include 子目录

set(HEADER_DIRECTORY ${HEADER_DIRECTORY} 
	"${PROJECT_SOURCE_DIR}/../../../math/eigen"
	"${PROJECT_SOURCE_DIR}/../../../third_party/serial/include"
	"${PROJECT_SOURCE_DIR}/../../serial_port/inc"
	"${PROJECT_SOURCE_DIR}/../../../communication/include"
    "../include/")

set(LIBRARY_DIRECTORY ${LIBRARY_DIRECTORY} 
    "/usr/lib"
   )
    
# thread 库
find_package(Threads REQUIRED)


# 指定生成目标
add_executable (testwind testwind.cc)
target_include_directories(testwind PRIVATE ${HEADER_DIRECTORY})
find_library(SERIAL_LIBRARY serial ${LIBRARY_DIRECTORY})
target_link_libraries(testwind PUBLIC ${SERIAL_LIBRARY})
target_link_libraries(testwind PUBLIC ${CMAKE_THREAD_LIBS_INIT})


add_executable (testwindsocket testwindsocket.cc)
target_include_directories(testwindsocket PRIVATE ${HEADER_DIRECTORY})
target_link_libraries(testwindsocket PUBLIC ${SERIAL_LIBRARY})
target_link_libraries(testwindsocket PUBLIC ${SERIAL_LIBRARY})
