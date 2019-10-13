# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.10)

# 项目信息
project (testopenspace)
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
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")


set(CMAKE_INCLUDE_CURRENT_DIR ON)

# thread库
find_package(Threads MODULE REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Boost 1.71.0 COMPONENTS date_time REQUIRED)

# 添加 include 子目录

set(HEADER_DIRECTORY ${HEADER_DIRECTORY} 
	"${PROJECT_SOURCE_DIR}/../include"
	"${PROJECT_SOURCE_DIR}/../../../"
	)

set(LIBRARY_DIRECTORY ${LIBRARY_DIRECTORY} 
	"/usr/lib"
    )

set(SOURCE_FILES ${SOURCE_FILES} 
	"${PROJECT_SOURCE_DIR}/../common/logging/src/easylogging++.cc" )


# 指定生成目标

add_executable (testnode testnode2d.cc)
target_include_directories(testnode PRIVATE ${HEADER_DIRECTORY})


add_executable (testmp testopenmp.cc)
target_include_directories(testmp PRIVATE ${HEADER_DIRECTORY})
target_link_libraries(testmp PUBLIC OpenMP::OpenMP_CXX)
target_link_libraries(testmp PUBLIC ${Boost_LIBRARIES})