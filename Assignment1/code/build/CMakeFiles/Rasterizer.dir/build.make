# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 4.0

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = D:\VSCode_Workspace\CMake\bin\cmake.exe

# The command to remove a file.
RM = D:\VSCode_Workspace\CMake\bin\cmake.exe -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = D:\Desktop\GAMES101\Assignment1\code

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = D:\Desktop\GAMES101\Assignment1\code\build

# Include any dependencies generated for this target.
include CMakeFiles/Rasterizer.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/Rasterizer.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/Rasterizer.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Rasterizer.dir/flags.make

CMakeFiles/Rasterizer.dir/codegen:
.PHONY : CMakeFiles/Rasterizer.dir/codegen

CMakeFiles/Rasterizer.dir/main.cpp.obj: CMakeFiles/Rasterizer.dir/flags.make
CMakeFiles/Rasterizer.dir/main.cpp.obj: CMakeFiles/Rasterizer.dir/includes_CXX.rsp
CMakeFiles/Rasterizer.dir/main.cpp.obj: D:/Desktop/GAMES101/Assignment1/code/main.cpp
CMakeFiles/Rasterizer.dir/main.cpp.obj: CMakeFiles/Rasterizer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=D:\Desktop\GAMES101\Assignment1\code\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Rasterizer.dir/main.cpp.obj"
	D:\VSCode_Workspace\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Rasterizer.dir/main.cpp.obj -MF CMakeFiles\Rasterizer.dir\main.cpp.obj.d -o CMakeFiles\Rasterizer.dir\main.cpp.obj -c D:\Desktop\GAMES101\Assignment1\code\main.cpp

CMakeFiles/Rasterizer.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/Rasterizer.dir/main.cpp.i"
	D:\VSCode_Workspace\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E D:\Desktop\GAMES101\Assignment1\code\main.cpp > CMakeFiles\Rasterizer.dir\main.cpp.i

CMakeFiles/Rasterizer.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/Rasterizer.dir/main.cpp.s"
	D:\VSCode_Workspace\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S D:\Desktop\GAMES101\Assignment1\code\main.cpp -o CMakeFiles\Rasterizer.dir\main.cpp.s

CMakeFiles/Rasterizer.dir/rasterizer.cpp.obj: CMakeFiles/Rasterizer.dir/flags.make
CMakeFiles/Rasterizer.dir/rasterizer.cpp.obj: CMakeFiles/Rasterizer.dir/includes_CXX.rsp
CMakeFiles/Rasterizer.dir/rasterizer.cpp.obj: D:/Desktop/GAMES101/Assignment1/code/rasterizer.cpp
CMakeFiles/Rasterizer.dir/rasterizer.cpp.obj: CMakeFiles/Rasterizer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=D:\Desktop\GAMES101\Assignment1\code\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/Rasterizer.dir/rasterizer.cpp.obj"
	D:\VSCode_Workspace\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Rasterizer.dir/rasterizer.cpp.obj -MF CMakeFiles\Rasterizer.dir\rasterizer.cpp.obj.d -o CMakeFiles\Rasterizer.dir\rasterizer.cpp.obj -c D:\Desktop\GAMES101\Assignment1\code\rasterizer.cpp

CMakeFiles/Rasterizer.dir/rasterizer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/Rasterizer.dir/rasterizer.cpp.i"
	D:\VSCode_Workspace\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E D:\Desktop\GAMES101\Assignment1\code\rasterizer.cpp > CMakeFiles\Rasterizer.dir\rasterizer.cpp.i

CMakeFiles/Rasterizer.dir/rasterizer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/Rasterizer.dir/rasterizer.cpp.s"
	D:\VSCode_Workspace\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S D:\Desktop\GAMES101\Assignment1\code\rasterizer.cpp -o CMakeFiles\Rasterizer.dir\rasterizer.cpp.s

CMakeFiles/Rasterizer.dir/Triangle.cpp.obj: CMakeFiles/Rasterizer.dir/flags.make
CMakeFiles/Rasterizer.dir/Triangle.cpp.obj: CMakeFiles/Rasterizer.dir/includes_CXX.rsp
CMakeFiles/Rasterizer.dir/Triangle.cpp.obj: D:/Desktop/GAMES101/Assignment1/code/Triangle.cpp
CMakeFiles/Rasterizer.dir/Triangle.cpp.obj: CMakeFiles/Rasterizer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=D:\Desktop\GAMES101\Assignment1\code\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/Rasterizer.dir/Triangle.cpp.obj"
	D:\VSCode_Workspace\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Rasterizer.dir/Triangle.cpp.obj -MF CMakeFiles\Rasterizer.dir\Triangle.cpp.obj.d -o CMakeFiles\Rasterizer.dir\Triangle.cpp.obj -c D:\Desktop\GAMES101\Assignment1\code\Triangle.cpp

CMakeFiles/Rasterizer.dir/Triangle.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/Rasterizer.dir/Triangle.cpp.i"
	D:\VSCode_Workspace\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E D:\Desktop\GAMES101\Assignment1\code\Triangle.cpp > CMakeFiles\Rasterizer.dir\Triangle.cpp.i

CMakeFiles/Rasterizer.dir/Triangle.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/Rasterizer.dir/Triangle.cpp.s"
	D:\VSCode_Workspace\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S D:\Desktop\GAMES101\Assignment1\code\Triangle.cpp -o CMakeFiles\Rasterizer.dir\Triangle.cpp.s

# Object files for target Rasterizer
Rasterizer_OBJECTS = \
"CMakeFiles/Rasterizer.dir/main.cpp.obj" \
"CMakeFiles/Rasterizer.dir/rasterizer.cpp.obj" \
"CMakeFiles/Rasterizer.dir/Triangle.cpp.obj"

# External object files for target Rasterizer
Rasterizer_EXTERNAL_OBJECTS =

Rasterizer.exe: CMakeFiles/Rasterizer.dir/main.cpp.obj
Rasterizer.exe: CMakeFiles/Rasterizer.dir/rasterizer.cpp.obj
Rasterizer.exe: CMakeFiles/Rasterizer.dir/Triangle.cpp.obj
Rasterizer.exe: CMakeFiles/Rasterizer.dir/build.make
Rasterizer.exe: D:/VSCode_Workspace/OpenCV-MinGW-Build-OpenCV-4.5.2-x64/x64/mingw/lib/libopencv_gapi452.dll.a
Rasterizer.exe: D:/VSCode_Workspace/OpenCV-MinGW-Build-OpenCV-4.5.2-x64/x64/mingw/lib/libopencv_highgui452.dll.a
Rasterizer.exe: D:/VSCode_Workspace/OpenCV-MinGW-Build-OpenCV-4.5.2-x64/x64/mingw/lib/libopencv_ml452.dll.a
Rasterizer.exe: D:/VSCode_Workspace/OpenCV-MinGW-Build-OpenCV-4.5.2-x64/x64/mingw/lib/libopencv_objdetect452.dll.a
Rasterizer.exe: D:/VSCode_Workspace/OpenCV-MinGW-Build-OpenCV-4.5.2-x64/x64/mingw/lib/libopencv_photo452.dll.a
Rasterizer.exe: D:/VSCode_Workspace/OpenCV-MinGW-Build-OpenCV-4.5.2-x64/x64/mingw/lib/libopencv_stitching452.dll.a
Rasterizer.exe: D:/VSCode_Workspace/OpenCV-MinGW-Build-OpenCV-4.5.2-x64/x64/mingw/lib/libopencv_video452.dll.a
Rasterizer.exe: D:/VSCode_Workspace/OpenCV-MinGW-Build-OpenCV-4.5.2-x64/x64/mingw/lib/libopencv_videoio452.dll.a
Rasterizer.exe: D:/VSCode_Workspace/OpenCV-MinGW-Build-OpenCV-4.5.2-x64/x64/mingw/lib/libopencv_dnn452.dll.a
Rasterizer.exe: D:/VSCode_Workspace/OpenCV-MinGW-Build-OpenCV-4.5.2-x64/x64/mingw/lib/libopencv_imgcodecs452.dll.a
Rasterizer.exe: D:/VSCode_Workspace/OpenCV-MinGW-Build-OpenCV-4.5.2-x64/x64/mingw/lib/libopencv_calib3d452.dll.a
Rasterizer.exe: D:/VSCode_Workspace/OpenCV-MinGW-Build-OpenCV-4.5.2-x64/x64/mingw/lib/libopencv_features2d452.dll.a
Rasterizer.exe: D:/VSCode_Workspace/OpenCV-MinGW-Build-OpenCV-4.5.2-x64/x64/mingw/lib/libopencv_flann452.dll.a
Rasterizer.exe: D:/VSCode_Workspace/OpenCV-MinGW-Build-OpenCV-4.5.2-x64/x64/mingw/lib/libopencv_imgproc452.dll.a
Rasterizer.exe: D:/VSCode_Workspace/OpenCV-MinGW-Build-OpenCV-4.5.2-x64/x64/mingw/lib/libopencv_core452.dll.a
Rasterizer.exe: CMakeFiles/Rasterizer.dir/linkLibs.rsp
Rasterizer.exe: CMakeFiles/Rasterizer.dir/objects1.rsp
Rasterizer.exe: CMakeFiles/Rasterizer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=D:\Desktop\GAMES101\Assignment1\code\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable Rasterizer.exe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\Rasterizer.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Rasterizer.dir/build: Rasterizer.exe
.PHONY : CMakeFiles/Rasterizer.dir/build

CMakeFiles/Rasterizer.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\Rasterizer.dir\cmake_clean.cmake
.PHONY : CMakeFiles/Rasterizer.dir/clean

CMakeFiles/Rasterizer.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" D:\Desktop\GAMES101\Assignment1\code D:\Desktop\GAMES101\Assignment1\code D:\Desktop\GAMES101\Assignment1\code\build D:\Desktop\GAMES101\Assignment1\code\build D:\Desktop\GAMES101\Assignment1\code\build\CMakeFiles\Rasterizer.dir\DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/Rasterizer.dir/depend

