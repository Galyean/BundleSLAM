cmake_minimum_required(VERSION 3.0)
project(BundleSLAM)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS})
set(SOURCE_FILES
        src/FeaturePoint.cpp
        src/FeatureTrack.cpp
        )

add_library(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
add_subdirectory(test)