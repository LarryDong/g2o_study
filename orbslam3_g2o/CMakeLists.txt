cmake_minimum_required(VERSION 2.8)
project(curve_fitting)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")


# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


# g2o
include_directories(${PROJECT_SOURCE_DIR}/Thirdparty/g2o)
include_directories("/usr/include/eigen3")


add_executable(curve_fitting curve_fitting.cpp)
target_link_libraries(curve_fitting
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
    ${OpenCV_LIBS}
)
