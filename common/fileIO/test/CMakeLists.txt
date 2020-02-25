# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.10)

# 项目信息
project (testfileio)
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
find_package(Boost COMPONENTS  unit_test_framework REQUIRED)


# 添加 include 子目录
set(HEADER_DIRECTORY ${HEADER_DIRECTORY} 
	"${PROJECT_SOURCE_DIR}/../../../"
	"/usr/include" )

set(LIBRARY_DIRECTORY ${LIBRARY_DIRECTORY} "/usr/lib")


# 指定生成目标

add_executable (testutility testutilityIO.cc)
target_include_directories(testutility PRIVATE ${HEADER_DIRECTORY})

add_executable (testcsv testcsv.cc)
target_include_directories(testcsv PRIVATE ${HEADER_DIRECTORY})


ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MODULE) 
add_executable (testjson testjson.cc)
target_include_directories(testjson PRIVATE ${HEADER_DIRECTORY})
target_link_libraries(testjson PUBLIC ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

