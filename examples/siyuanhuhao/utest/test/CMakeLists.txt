# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.10)

# 项目信息
project (experiment)
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
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -march=native -mavx")

set(CMAKE_INCLUDE_CURRENT_DIR ON)


# 添加 include 子目录
message("PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")
set(HEADER_DIRECTORY ${HEADER_DIRECTORY} 
	"${PROJECT_SOURCE_DIR}/../../../../"
	"${PROJECT_SOURCE_DIR}/../../../../modules/messages/sensors/marine_radar/third_party/SDK_3.0.02/include/"
	"${PROJECT_SOURCE_DIR}/../../../../common/math/eigen"
	"/opt/mosek/9.0/tools/platform/linux64x86/h")

set(LIBRARY_DIRECTORY ${LIBRARY_DIRECTORY} 
    "/usr/lib"
    "/opt/mosek/9.0/tools/platform/linux64x86/bin"
   )

# thread 库
find_package(Threads REQUIRED)
find_package(GeographicLib 1.49 REQUIRED)
find_package(Boost 1.71.0 COMPONENTS date_time REQUIRED)
find_library(SERIAL_LIBRARY serial HINTS ${LIBRARY_DIRECTORY})
find_library(SQLITE3_LIBRARY sqlite3 HINTS ${LIBRARY_DIRECTORY})
find_library(MOSEK8_LIBRARY mosek64 HINTS ${LIBRARY_DIRECTORY})
find_library(NRPCLIENT_LIBRARY NRPClient HINTS ${LIBRARY_DIRECTORY})
find_library(NRPPPI_LIBRARY NRPPPI HINTS ${LIBRARY_DIRECTORY})

    
# 指定生成目标
set(SOURCE_FILES ${SOURCE_FILES} 
	"${PROJECT_SOURCE_DIR}/../../../../common/logging/src/easylogging++.cc"                            
	"${CMAKE_CURRENT_SOURCE_DIR}/run.cc")
add_executable (testASV ${SOURCE_FILES})
target_include_directories(testASV PRIVATE ${HEADER_DIRECTORY})
target_link_libraries(testASV PUBLIC ${SERIAL_LIBRARY})
target_link_libraries(testASV PUBLIC ${SQLITE3_LIBRARY})
target_link_libraries(testASV PUBLIC ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(testASV PUBLIC ${MOSEK8_LIBRARY})
target_link_libraries(testASV PUBLIC ${GeographicLib_LIBRARIES})
target_link_libraries(testASV PUBLIC ${Boost_LIBRARIES})
target_link_libraries(testASV PUBLIC ${NRPCLIENT_LIBRARY})
target_link_libraries(testASV PUBLIC ${NRPPPI_LIBRARY})