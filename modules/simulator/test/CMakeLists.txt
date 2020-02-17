# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.10)

# 项目信息
project (testsimulator)
set(CMAKE_CXX_STANDARD 17)


# UNIX, WIN32, WINRT, CYGWIN, APPLE are environment 
# variables as flags set by default system
if(UNIX)
    message("This is a ${CMAKE_SYSTEM_NAME} system")
elseif(WIN32)
    message("This is a Windows System")
endif()

set(CMAKE_BUILD_TYPE "Debug") # Debug or Release
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -Wextra -g -ggdb -pedantic")
# set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wextra -pedantic")

set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")


set(CMAKE_INCLUDE_CURRENT_DIR ON)


# thread库
find_package(Threads MODULE REQUIRED)
find_package(Boost COMPONENTS unit_test_framework REQUIRED)


# 添加 include 子目录


set(HEADER_DIRECTORY ${HEADER_DIRECTORY} 
	"${PROJECT_SOURCE_DIR}/../../../"
	"${PROJECT_SOURCE_DIR}/../../../common/math/eigen"
)

set(LIBRARY_DIRECTORY ${LIBRARY_DIRECTORY} 
	"/usr/lib")


set(RARE_LIBRARIES ${RARE_LIBRARIES} 
	"boost_system"
	"boost_filesystem"
	"boost_iostreams"
	"util"
	)
# 指定生成目标
add_executable (testsimulator testsimulator.cc)
target_include_directories(testsimulator PRIVATE ${HEADER_DIRECTORY})
target_link_libraries(testsimulator ${RARE_LIBRARIES})

