cmake_minimum_required(VERSION 3.10)
project(BezierCurve)

set(OpenCV_DIR D:/VSCode_Workspace/OpenCV-MinGW-Build-OpenCV-4.5.2-x64/) 
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 14)

add_executable(BezierCurve main.cpp)

target_link_libraries(BezierCurve ${OpenCV_LIBRARIES})

# ---- Copy runtime DLLs next to the executable after build ----
# Without this, Windows would load libstdc++-6.dll from PATH (mingw64_/win32/UCRT),
# whose fpos<_Mbstatet> ABI is incompatible with the OpenCV DLLs built with the
# older fpos<int> ABI (mingw64-posix/MSVCRT). Copying the posix-flavor runtime DLLs
# into the same folder as BezierCurve.exe ensures they are found first at runtime.
set(MINGW_BIN "D:/VSCode_Workspace/mingw64-posix/bin")
set(OPENCV_BIN "D:/VSCode_Workspace/OpenCV-MinGW-Build-OpenCV-4.5.2-x64/x64/mingw/bin")

# Destination: $<TARGET_FILE_DIR:BezierCurve> → the build directory containing BezierCurve.exe
add_custom_command(TARGET BezierCurve POST_BUILD
    # MinGW posix runtime libraries
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${MINGW_BIN}/libstdc++-6.dll"
        "${MINGW_BIN}/libgcc_s_seh-1.dll"
        "${MINGW_BIN}/libwinpthread-1.dll"
        "$<TARGET_FILE_DIR:BezierCurve>"
    # OpenCV libraries needed at runtime
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${OPENCV_BIN}/libopencv_core452.dll"
        "${OPENCV_BIN}/libopencv_imgcodecs452.dll"
        "${OPENCV_BIN}/libopencv_imgproc452.dll"
        "${OPENCV_BIN}/libopencv_highgui452.dll"
        "${OPENCV_BIN}/libopencv_videoio452.dll"
        "${OPENCV_BIN}/opencv_videoio_ffmpeg452_64.dll"
        "$<TARGET_FILE_DIR:BezierCurve>"
    COMMENT "Copying runtime DLLs (mingw64-posix + OpenCV) next to BezierCurve.exe"
)
