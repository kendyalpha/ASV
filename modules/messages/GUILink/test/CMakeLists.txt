# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.8)

# 项目信息
project (testgui)
set(CMAKE_CXX_STANDARD 17)


# 添加 include 子目录

set(HEADER_DIRECTORY ${HEADER_DIRECTORY} 
	"${PROJECT_SOURCE_DIR}/../../../../")

set(LIBRARY_DIRECTORY ${LIBRARY_DIRECTORY} 
    "/usr/lib"
    "/usr/local/lib"
   )
 
set(SOURCE_FILES ${SOURCE_FILES} 
	"${PROJECT_SOURCE_DIR}/../../../../common/logging/src/easylogging++.cc" ) 

# 指定生成目标
find_package(Boost 1.71.0 COMPONENTS date_time REQUIRED)
find_library(SERIAL_LIBRARY serial ${LIBRARY_DIRECTORY})

add_executable (testgui ${SOURCE_FILES} testguilink.cc)
target_include_directories(testgui PRIVATE ${HEADER_DIRECTORY})
target_link_libraries(testgui ${SERIAL_LIBRARY})
target_link_libraries(testgui ${Boost_LIBRARIES})

