cmake_minimum_required (VERSION 4.0.0)
project (Transformation)

set(OpenCV_DIR D:/VSCode_Workspace/OpenCV-MinGW-Build-OpenCV-4.5.2-x64) 
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)

set(CMAKE_CXX_STANDARD 17)

include_directories("D:/VSCode_Workspace/Eigen3/include/eigen3")

add_executable (Transformation main.cpp)