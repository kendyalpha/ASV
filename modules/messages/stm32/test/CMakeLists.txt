# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.8)

# 项目信息
project (teststm32)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_BUILD_TYPE "Debug") # "Debug" or "Release" mode
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -Wextra -g -ggdb -pedantic")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -march=native -mavx")

set(CMAKE_INCLUDE_CURRENT_DIR ON)



# 添加 include 子目录

set(HEADER_DIRECTORY ${HEADER_DIRECTORY} 
   	"${PROJECT_SOURCE_DIR}/../../../../"
    )

set(LIBRARY_DIRECTORY ${LIBRARY_DIRECTORY} 
    "/usr/lib"
    "/usr/local/lib"
   )
 
set(SOURCE_FILES ${SOURCE_FILES} 
	"${PROJECT_SOURCE_DIR}/../../../../common/logging/src/easylogging++.cc" )


find_package(Boost 1.71.0 COMPONENTS date_time REQUIRED)
find_library(SERIAL_LIBRARY serial ${LIBRARY_DIRECTORY})
# 指定生成目标
add_executable (teststm32 teststm32.cc  ${SOURCE_FILES})
target_include_directories(teststm32 PRIVATE ${HEADER_DIRECTORY})
target_link_libraries(teststm32 ${Boost_LIBRARIES})
target_link_libraries(teststm32 ${SERIAL_LIBRARY})
