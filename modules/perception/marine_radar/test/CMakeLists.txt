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

set(CMAKE_BUILD_TYPE "Release") # "Debug" or "Release" mode
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -Wextra -g -ggdb -pedantic ")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -mavx")


set(CMAKE_INCLUDE_CURRENT_DIR ON)


# 添加 include 子目录

set(HEADER_DIRECTORY ${HEADER_DIRECTORY} 
	"${PROJECT_SOURCE_DIR}/../../../../"
	"${PROJECT_SOURCE_DIR}/../../../../common/math/pyclustering/ccore/include"
	)

set(LIBRARY_DIRECTORY ${LIBRARY_DIRECTORY} 
	"/usr/lib"
	"${PROJECT_SOURCE_DIR}/../../../../common/math/pyclustering/ccore/libs/"
     )


# thread库
find_package(Threads MODULE REQUIRED)
find_package(PythonLibs 2.7)
find_library(CLUSTER_LIBRARY pyclustering HINTS ${LIBRARY_DIRECTORY})

# 指定生成目标
add_executable (testTargetDetection testTargetDetection.cc )
target_include_directories(testTargetDetection PRIVATE ${HEADER_DIRECTORY})
target_include_directories(testTargetDetection PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(testTargetDetection PUBLIC ${PYTHON_LIBRARIES})
target_link_libraries(testTargetDetection PUBLIC ${CLUSTER_LIBRARY})
target_link_libraries(testTargetDetection PUBLIC ${CMAKE_THREAD_LIBS_INIT})



add_executable (testTargetTracking testTargetTracking.cc )
target_include_directories(testTargetTracking PRIVATE ${HEADER_DIRECTORY})
target_include_directories(testTargetTracking PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(testTargetTracking PUBLIC ${PYTHON_LIBRARIES})
target_link_libraries(testTargetTracking PUBLIC ${CLUSTER_LIBRARY})
target_link_libraries(testTargetTracking PUBLIC ${CMAKE_THREAD_LIBS_INIT})