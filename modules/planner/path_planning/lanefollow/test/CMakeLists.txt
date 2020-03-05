# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.10)

# 项目信息
project (testlos)
set(CMAKE_CXX_STANDARD 17)


# UNIX, WIN32, WINRT, CYGWIN, APPLE are environment 
# variables as flags set by default system
if(UNIX)
    message("This is a ${CMAKE_SYSTEM_NAME} system")
elseif(WIN32)
    message("This is a Windows System")
endif()

set(CMAKE_BUILD_TYPE "Release") # "Debug" or "Release" mode
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -Wextra -g -ggdb -pedantic")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -mavx")


set(CMAKE_INCLUDE_CURRENT_DIR ON)

# thread库
find_package(Threads MODULE REQUIRED)
find_library(SQLITE3_LIBRARY sqlite3 HINTS ${LIBRARY_DIRECTORY})

# 添加 include 子目录

set(HEADER_DIRECTORY ${HEADER_DIRECTORY} 
	"${PROJECT_SOURCE_DIR}/../../../../../"
	)

set(LIBRARY_DIRECTORY ${LIBRARY_DIRECTORY} 
	"/usr/lib"
    )

set(SOURCE_FILES ${SOURCE_FILES} 
	"${PROJECT_SOURCE_DIR}/../../../../../common/logging/src/easylogging++.cc" )


# 指定生成目标
set(RARE_LIBRARIES ${RARE_LIBRARIES} 
	"boost_system"
	"boost_filesystem"
	"boost_iostreams"
	"util"
	)
add_executable (testFrenetTrajectoryGenerator testFrenetTrajectoryGenerator.cc ${SOURCE_FILES} )
target_include_directories(testFrenetTrajectoryGenerator PRIVATE ${HEADER_DIRECTORY})
target_link_libraries(testFrenetTrajectoryGenerator PRIVATE ${RARE_LIBRARIES})


add_executable (test_experiment test_experimental.cc ${SOURCE_FILES} )
target_include_directories(test_experiment PRIVATE ${HEADER_DIRECTORY})
target_link_libraries(test_experiment PUBLIC ${SQLITE3_LIBRARY})


add_executable (testtransform testtransform.cc ${SOURCE_FILES} )
target_include_directories(testtransform PRIVATE ${HEADER_DIRECTORY})
