# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.8)

# 项目信息
project (testlogging)
set(CMAKE_CXX_STANDARD 17)


# UNIX, WIN32, WINRT, CYGWIN, APPLE are environment 
# variables as flags set by default system
if(UNIX)
    message("This is a ${CMAKE_SYSTEM_NAME} system")
elseif(WIN32)
    message("This is a Windows System")
endif()

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -Wextra -g -ggdb -pedantic")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")


set(CMAKE_INCLUDE_CURRENT_DIR ON)

# thread 库
find_package(Threads REQUIRED)

# 添加 include 子目录
set(HEADER_DIRECTORY ${HEADER_DIRECTORY} 
	"/usr/include" )

set(LIBRARY_DIRECTORY ${LIBRARY_DIRECTORY} "/usr/lib")
set(SOURCE_FILES ${SOURCE_FILES} 
	"${CMAKE_CURRENT_SOURCE_DIR}/testlogger.cc")


# 指定生成目标
# add_executable (testsqlite functions.cc ${HEADER_FILES})
add_executable (testlog ${SOURCE_FILES} "${CMAKE_CURRENT_SOURCE_DIR}/../src/easylogging++.cc"   )
target_include_directories(testlog PRIVATE ${HEADER_DIRECTORY})
target_link_libraries(testlog ${CMAKE_THREAD_LIBS_INIT})


# Builds static library for Easylogging++
ADD_LIBRARY(EasyLogging STATIC "${CMAKE_CURRENT_SOURCE_DIR}/../src/easylogging++.cc")

add_executable (testlog1 ${SOURCE_FILES})
target_include_directories(testlog1 PRIVATE ${HEADER_DIRECTORY})
target_link_libraries(testlog1 EasyLogging)
target_link_libraries(testlog1 ${CMAKE_THREAD_LIBS_INIT})
