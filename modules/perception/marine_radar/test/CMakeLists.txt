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
find_library(SQLITE3_LIBRARY sqlite3 HINTS ${LIBRARY_DIRECTORY})
find_library(CLUSTER_LIBRARY pyclustering HINTS ${LIBRARY_DIRECTORY})
set(RARE_LIBRARIES ${RARE_LIBRARIES} 
	"boost_system"
	"boost_filesystem"
	"boost_iostreams"
	"util"
	)
# 指定生成目标
add_executable (testTargetDetection testTargetTracking.cc )
target_include_directories(testTargetDetection PRIVATE ${HEADER_DIRECTORY})
target_link_libraries(testTargetDetection PUBLIC ${CLUSTER_LIBRARY})
target_link_libraries(testTargetDetection PUBLIC ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(testTargetDetection PUBLIC ${RARE_LIBRARIES})


add_executable (testRadarFiltering testRadarFiltering.cc )
target_include_directories(testRadarFiltering PRIVATE ${HEADER_DIRECTORY})
target_link_libraries(testRadarFiltering PUBLIC ${CLUSTER_LIBRARY})
target_link_libraries(testRadarFiltering PUBLIC ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(testRadarFiltering PUBLIC ${RARE_LIBRARIES})


add_executable (testTargetTracking_Radar testTargetTracking_Radar.cc )
target_include_directories(testTargetTracking_Radar PRIVATE ${HEADER_DIRECTORY})
target_link_libraries(testTargetTracking_Radar PUBLIC ${SQLITE3_LIBRARY})
target_link_libraries(testTargetTracking_Radar PUBLIC ${CLUSTER_LIBRARY})
target_link_libraries(testTargetTracking_Radar PUBLIC ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(testTargetTracking_Radar PUBLIC ${RARE_LIBRARIES})
