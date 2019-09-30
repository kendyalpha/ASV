# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.8)

# 项目信息
project (teststm32)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_BUILD_TYPE "Debug") # "Debug" or "Release" mode
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -Wextra -g -ggdb -pedantic")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -march=native -mavx")

set(CMAKE_INCLUDE_CURRENT_DIR ON)


find_package(Boost 1.71.0 COMPONENTS date_time REQUIRED)

# 添加 include 子目录

set(HEADER_DIRECTORY ${HEADER_DIRECTORY} 
    "../include/"
   	"${PROJECT_SOURCE_DIR}/../../timer/include"
   	"${PROJECT_SOURCE_DIR}/../../logging/include"
    )

set(LIBRARY_DIRECTORY ${LIBRARY_DIRECTORY} 
    "/usr/lib"
    "/usr/local/lib"
   )
 
set(SOURCE_FILES ${SOURCE_FILES} 
	"${PROJECT_SOURCE_DIR}/../../logging/src/easylogging++.cc" )


# 指定生成目标
add_executable (teststm32 teststm32.cc  ${SOURCE_FILES})
target_include_directories(teststm32 PRIVATE ${HEADER_DIRECTORY})
target_link_libraries(teststm32 ${Boost_LIBRARIES})