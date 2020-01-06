# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.10)

# 项目信息
project (MatplotlibCPP_Test)
set(CMAKE_CXX_STANDARD 17)


find_package(PythonLibs 2.7)

add_executable(animation fill_inbetween.cpp)
target_include_directories(animation PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(animation ${PYTHON_LIBRARIES})


 