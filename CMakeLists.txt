cmake_minimum_required(VERSION 3.10)
project(HD_WH)
set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP)

link_libraries(stdc++fs)

include_directories(${PCL_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}
                    ${OpenCV_INCLURE_DIRS})

add_definitions(${PCL_DEFINITIONS})

link_directories(${PCL_LIBRARY_DIRS})

add_library(lib_hys SHARED
src/pointcloud_processor.cpp)

target_link_libraries(lib_hys
                      ${PCL_LIBRARIES}
                      ${OpenCV_LIBRARIES})

add_executable(test_sor src/main_sor.cpp)

target_link_libraries(test_sor lib_hys)
