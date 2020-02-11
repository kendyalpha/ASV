# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.8)

# 项目信息
project (testGPS)
set(CMAKE_CXX_STANDARD 17)


# 添加 include 子目录

set(HEADER_DIRECTORY ${HEADER_DIRECTORY} 
	"${PROJECT_SOURCE_DIR}/../../../../")

set(LIBRARY_DIRECTORY ${LIBRARY_DIRECTORY} 
    "/usr/lib"
   )
    
# thread 库
find_package(Threads REQUIRED)

set(SOURCE_FILES ${SOURCE_FILES} 
	"${PROJECT_SOURCE_DIR}/../../../../../common/logging/src/easylogging++.cc")  

# 指定生成目标
add_executable (testwind testwind.cc)
target_include_directories(testwind PRIVATE ${HEADER_DIRECTORY})
find_library(SERIAL_LIBRARY serial ${LIBRARY_DIRECTORY})
target_link_libraries(testwind PUBLIC ${SERIAL_LIBRARY})
target_link_libraries(testwind PUBLIC ${CMAKE_THREAD_LIBS_INIT})


add_executable (testwindsocket testwindsocket.cc ${SOURCE_FILES} )
target_include_directories(testwindsocket PRIVATE ${HEADER_DIRECTORY})
target_link_libraries(testwindsocket PUBLIC ${SERIAL_LIBRARY})
target_link_libraries(testwindsocket PUBLIC ${SERIAL_LIBRARY})
