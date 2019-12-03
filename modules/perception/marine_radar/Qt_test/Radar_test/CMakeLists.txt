cmake_minimum_required(VERSION 3.5)

project(Radar_test LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Qt5 library
find_package(Qt5 COMPONENTS Widgets REQUIRED)

set(HEADER_DIRECTORY ${HEADER_DIRECTORY}
        "${PROJECT_SOURCE_DIR}/../../third_party/SDK_3.0.02/include")



add_executable(Radar_test
  main.cpp
  marine_radar.cpp
  marine_radar.h
  marine_radar.ui
)

target_include_directories(Radar_test PRIVATE ${HEADER_DIRECTORY})
target_link_libraries(Radar_test PRIVATE Qt5::Widgets)
