# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.10)

# 项目信息
project (testSpoke)
set(CMAKE_CXX_STANDARD 17)


# UNIX, WIN32, WINRT, CYGWIN, APPLE are environment 
# variables as flags set by default system
if(UNIX)
    message("This is a ${CMAKE_SYSTEM_NAME} system")
elseif(WIN32)
    message("This is a Windows System")
endif()

set(CMAKE_BUILD_TYPE "Debug") # "Debug" or "Release" mode
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -Wextra -g -ggdb -pedantic")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -mavx")


set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(LIBRARY_DIRECTORY ${LIBRARY_DIRECTORY} 
	"${PROJECT_SOURCE_DIR}/../ccore/"
    )
# thread库
find_package(Threads MODULE REQUIRED)
find_library(CLUSTER_LIBRARY pyclustering HINTS ${LIBRARY_DIRECTORY})

# 添加 include 子目录

set(HEADER_DIRECTORY ${HEADER_DIRECTORY} 
	"${PROJECT_SOURCE_DIR}/../ccore/include/"
	"${PROJECT_SOURCE_DIR}/../ccore/external/include"
	"${PROJECT_SOURCE_DIR}/../ccore/tst"
	)


# 指定生成目标
add_executable (testDBScan testdbscan.cc 
	"../ccore/tst/samples.cpp" 
	"../ccore/tst/utenv_check.cpp"
	"../ccore/external/include/gtest/gtest-all.cpp"
	)
target_include_directories(testDBScan PRIVATE ${HEADER_DIRECTORY})
target_link_libraries(testDBScan PUBLIC ${CLUSTER_LIBRARY})
target_link_libraries(testDBScan PUBLIC ${CMAKE_THREAD_LIBS_INIT})
