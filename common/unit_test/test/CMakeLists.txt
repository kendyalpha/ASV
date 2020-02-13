# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.10)

# 项目信息
project (unit_test)
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
find_package(Boost COMPONENTS unit_test_framework REQUIRED)

# 添加 include 子目录

ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MODULE) 
add_executable (boosttest boosttest.cc)
target_link_libraries(boosttest ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})