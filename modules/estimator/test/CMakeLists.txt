# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.10)

# 项目信息
project (testestimator)
set(CMAKE_CXX_STANDARD 17)

# UNIX, WIN32, WINRT, CYGWIN, APPLE are environment 
# variables as flags set by default system
if(UNIX)
    message("This is a ${CMAKE_SYSTEM_NAME} system")
elseif(WIN32)
    message("This is a Windows System")
endif()

set(CMAKE_BUILD_TYPE "Debug") # "Debug" or "Release" mode
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -Wextra -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

set(CMAKE_INCLUDE_CURRENT_DIR ON)


#GeographicLib库
find_package (GeographicLib 1.49 REQUIRED)


# 添加 include 子目录

set(HEADER_DIRECTORY ${HEADER_DIRECTORY} 
	"${PROJECT_SOURCE_DIR}/../../../"
	"/opt/mosek/8/tools/platform/linux64x86/h")


set(LIBRARY_DIRECTORY ${LIBRARY_DIRECTORY} 
	"/usr/lib"
	"/usr/local/lib"
	)

set(SOURCE_FILES ${SOURCE_FILES} 
	"${PROJECT_SOURCE_DIR}/../../../common/logging/src/easylogging++.cc")    
# 指定生成目标

set(RARE_LIBRARIES ${RARE_LIBRARIES} 
	"boost_system"
	"boost_filesystem"
	"boost_iostreams"
	"util"
	)
add_executable (testkalman testkalman.cc)
target_include_directories(testkalman PRIVATE ${HEADER_DIRECTORY})
target_link_libraries(testkalman ${RARE_LIBRARIES})


add_executable (testestimator testestimator.cc)
target_include_directories(testestimator PRIVATE ${HEADER_DIRECTORY})

 
add_executable (testAntenna testAntennadeviation.cc ${SOURCE_FILES})
target_include_directories(testAntenna PRIVATE ${HEADER_DIRECTORY})
find_library(SERIAL_LIBRARY serial ${LIBRARY_DIRECTORY})
find_library(SQLITE3_LIBRARY sqlite3 HINTS ${LIBRARY_DIRECTORY})
target_link_libraries(testAntenna ${SERIAL_LIBRARY})
target_link_libraries(testAntenna ${GeographicLib_LIBRARIES})
target_link_libraries(testAntenna ${SQLITE3_LIBRARY})
