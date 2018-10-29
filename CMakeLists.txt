cmake_minimum_required(VERSION 3.0)
project(BundleSLAM)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -O3 -Wno-reorder")
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS})
set(SOURCE_FILES
        src/FeaturePoint.cpp
        src/FeatureTrack.cpp
        include/InitialSFM.h src/InitialSFM.cpp include/CameraBuffer.h src/Camerabuffer.cpp)

add_library(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
add_subdirectory(test)