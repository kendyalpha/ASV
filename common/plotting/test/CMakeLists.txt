# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.10)

# 项目信息
project (MatplotlibCPP_Test)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_BUILD_TYPE "Debug") # "Debug" or "Release" mode
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -Wextra -g -ggdb -pedantic")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -mavx")



set(HEADER_DIRECTORY ${HEADER_DIRECTORY} 
	"${PROJECT_SOURCE_DIR}/../include"
	)



set(RARE_LIBRARIES ${RARE_LIBRARIES} 
	"boost_system"
	"boost_filesystem"
	"boost_iostreams"
	"util"
	)

add_executable(example-data-1d example-data-1d.cc)
target_include_directories(example-data-1d PUBLIC ${HEADER_DIRECTORY})
target_link_libraries(example-data-1d ${RARE_LIBRARIES})

add_executable(example-data-2d example-data-2d.cc)
target_include_directories(example-data-2d PUBLIC ${HEADER_DIRECTORY})
target_link_libraries(example-data-2d ${RARE_LIBRARIES})


add_executable(example-interactive example-interactive.cc)
target_include_directories(example-interactive PUBLIC ${HEADER_DIRECTORY})
target_link_libraries(example-interactive ${RARE_LIBRARIES})


add_executable(example-misc example-misc.cc)
target_include_directories(example-misc PUBLIC ${HEADER_DIRECTORY})
target_link_libraries(example-misc ${RARE_LIBRARIES})
