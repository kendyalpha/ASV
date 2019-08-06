# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.8)

# 项目信息
project (testcommunication)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_BUILD_TYPE "Debug") # "Debug" or "Release" mode
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -Wextra -g -ggdb -pedantic")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -march=native -mavx")

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# 添加 include 子目录

set(HEADER_DIRECTORY ${HEADER_DIRECTORY} 
    "../include/"
   	"${PROJECT_SOURCE_DIR}/../../third_party/serial/include"
   	"${PROJECT_SOURCE_DIR}/../../timer/include"
    )

set(LIBRARY_DIRECTORY ${LIBRARY_DIRECTORY} 
    "/usr/lib"
    "/usr/local/lib"
   )
 
# 指定生成目标
add_executable (testcrc testcrc.cc)
target_include_directories(testcrc PRIVATE ${HEADER_DIRECTORY})

add_executable (testtcpserver testtcpserver.cc)
target_include_directories(testtcpserver PRIVATE ${HEADER_DIRECTORY})

add_executable (testtcpclient testtcpclient.cc)
target_include_directories(testtcpclient PRIVATE ${HEADER_DIRECTORY})

add_executable (testUAV testUAV.cc)
target_include_directories(testUAV PRIVATE ${HEADER_DIRECTORY})
find_library(SERIAL_LIBRARY serial ${LIBRARY_DIRECTORY})
target_link_libraries(testUAV PUBLIC ${SERIAL_LIBRARY})