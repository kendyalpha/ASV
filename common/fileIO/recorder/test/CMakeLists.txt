# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.10)

# 项目信息
project (DataRecorder)
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


# 添加 include 子目录
set(HEADER_DIRECTORY ${HEADER_DIRECTORY} 
	"${PROJECT_SOURCE_DIR}/../../../../"
	"/usr/include" )

set(LIBRARY_DIRECTORY ${LIBRARY_DIRECTORY} "/usr/lib")


# thread 库
find_package(Threads REQUIRED)
find_package(Boost COMPONENTS  unit_test_framework REQUIRED)
find_library(SQLITE3_LIBRARY sqlite3 HINTS ${LIBRARY_DIRECTORY})


ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MODULE) 

# 指定生成目标

set(SOURCE_FILES ${SOURCE_FILES} 
	"${PROJECT_SOURCE_DIR}/../../../logging/src/easylogging++.cc"                            
	"${CMAKE_CURRENT_SOURCE_DIR}/testdatabase.cc")
add_executable (testdatabase ${SOURCE_FILES})
target_include_directories(testdatabase PRIVATE ${HEADER_DIRECTORY})
target_link_libraries(testdatabase PUBLIC ${SQLITE3_LIBRARY})
target_link_libraries(testdatabase PUBLIC ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})


