# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.10)

# 项目信息
project (testGPS)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_BUILD_TYPE "Debug") # "Debug" or "Release" mode
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -Wextra -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")


set(LIBRARY_DIRECTORY ${LIBRARY_DIRECTORY} 
    "/usr/lib"
    "/usr/local/lib"
   )
    
# 添加 include 子目录
set(HEADER_DIRECTORY ${HEADER_DIRECTORY} 
	"${PROJECT_SOURCE_DIR}/../../../../../")


set(SOURCE_FILES ${SOURCE_FILES} 
	"${PROJECT_SOURCE_DIR}/../../../../../common/logging/src/easylogging++.cc")     

#GeographicLib库
find_package (GeographicLib 1.49 REQUIRED)
find_package(Threads REQUIRED)
find_library(SERIAL_LIBRARY serial HINTS ${LIBRARY_DIRECTORY})
find_library(SQLITE3_LIBRARY sqlite3 HINTS ${LIBRARY_DIRECTORY})


# 指定生成目标
add_executable (testGPS testGPS.cc  ${SOURCE_FILES})
target_include_directories(testGPS PRIVATE ${HEADER_DIRECTORY})
target_link_libraries(testGPS ${SERIAL_LIBRARY})
target_link_libraries(testGPS ${GeographicLib_LIBRARIES})


# 指定生成目标
add_executable (testGPSSave testGPSandSave.cc ${SOURCE_FILES})
target_include_directories(testGPSSave PRIVATE ${HEADER_DIRECTORY})
target_link_libraries(testGPSSave ${SERIAL_LIBRARY})
target_link_libraries(testGPSSave ${GeographicLib_LIBRARIES})
target_link_libraries(testGPSSave ${SQLITE3_LIBRARY})


# 指定生成目标
add_executable (testNMEA testNMEA.cc)
target_include_directories(testNMEA PRIVATE ${HEADER_DIRECTORY})


# 指定生成目标
add_executable (testUTM example-UTMUPS.cc)
target_include_directories(testUTM PRIVATE ${HEADER_DIRECTORY})
target_link_libraries(testUTM ${GeographicLib_LIBRARIES})

# 指定生成目标
set(XSENS_LIB_DIR ${XSENS_LIB_DIR} 
	"${PROJECT_SOURCE_DIR}/../third_party/mtsdk/xspublic")   
add_executable (testIMU testIMU.cc ${SOURCE_FILES})
target_include_directories(testIMU PRIVATE ${HEADER_DIRECTORY} ${XSENS_LIB_DIR})
target_link_libraries(testIMU ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(testIMU "${XSENS_LIB_DIR}/xscontroller/libxscontroller.a")
target_link_libraries(testIMU "${XSENS_LIB_DIR}/xscommon/libxscommon.a")
target_link_libraries(testIMU "${XSENS_LIB_DIR}/xstypes/libxstypes.a")
target_link_libraries(testIMU dl)

