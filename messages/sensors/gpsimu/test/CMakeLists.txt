# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.8)

# 项目信息
project (testGPS)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_BUILD_TYPE "Debug") # "Debug" or "Release" mode
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -Wextra -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

#GeographicLib库
find_package (GeographicLib 1.49 REQUIRED)

# 添加 include 子目录
set(HEADER_DIRECTORY ${HEADER_DIRECTORY} 
	"${PROJECT_SOURCE_DIR}/../../../../")

set(LIBRARY_DIRECTORY ${LIBRARY_DIRECTORY} 
    "/usr/lib"
    "/usr/local/lib"
   )
    

set(SOURCE_FILES ${SOURCE_FILES} 
	"${PROJECT_SOURCE_DIR}/../../../../common/logging/src/easylogging++.cc")          
# 指定生成目标
add_executable (testGPS testGPS.cc)
target_include_directories(testGPS PRIVATE ${HEADER_DIRECTORY})
find_library(SERIAL_LIBRARY serial ${LIBRARY_DIRECTORY})
target_link_libraries(testGPS ${SERIAL_LIBRARY})
target_link_libraries(testGPS ${GeographicLib_LIBRARIES})


# 指定生成目标
add_executable (testGPSSave testGPSandSave.cc ${SOURCE_FILES} )
target_include_directories(testGPSSave PRIVATE ${HEADER_DIRECTORY})
find_library(SERIAL_LIBRARY serial HINTS ${LIBRARY_DIRECTORY})
find_library(SQLITE3_LIBRARY sqlite3 HINTS ${LIBRARY_DIRECTORY})
target_link_libraries(testGPSSave ${SERIAL_LIBRARY})
target_link_libraries(testGPSSave ${GeographicLib_LIBRARIES})
target_link_libraries(testGPSSave ${SQLITE3_LIBRARY})


# 指定生成目标
add_executable (testNMEA testNMEA.cc)
target_include_directories(testNMEA PRIVATE ${HEADER_DIRECTORY})
