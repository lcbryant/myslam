# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /opt/cmake-3.15.2/bin/cmake

# The command to remove a file.
RM = /opt/cmake-3.15.2/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/lcbryant/桌面/myslam/result

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lcbryant/桌面/myslam/result/build

# Include any dependencies generated for this target.
include CMakeFiles/slam_shared.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/slam_shared.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/slam_shared.dir/flags.make

CMakeFiles/slam_shared.dir/slamBase.cpp.o: CMakeFiles/slam_shared.dir/flags.make
CMakeFiles/slam_shared.dir/slamBase.cpp.o: ../slamBase.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lcbryant/桌面/myslam/result/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/slam_shared.dir/slamBase.cpp.o"
	g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/slam_shared.dir/slamBase.cpp.o -c /home/lcbryant/桌面/myslam/result/slamBase.cpp

CMakeFiles/slam_shared.dir/slamBase.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/slam_shared.dir/slamBase.cpp.i"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lcbryant/桌面/myslam/result/slamBase.cpp > CMakeFiles/slam_shared.dir/slamBase.cpp.i

CMakeFiles/slam_shared.dir/slamBase.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/slam_shared.dir/slamBase.cpp.s"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lcbryant/桌面/myslam/result/slamBase.cpp -o CMakeFiles/slam_shared.dir/slamBase.cpp.s

# Object files for target slam_shared
slam_shared_OBJECTS = \
"CMakeFiles/slam_shared.dir/slamBase.cpp.o"

# External object files for target slam_shared
slam_shared_EXTERNAL_OBJECTS =

libslam_shared.so: CMakeFiles/slam_shared.dir/slamBase.cpp.o
libslam_shared.so: CMakeFiles/slam_shared.dir/build.make
libslam_shared.so: CMakeFiles/slam_shared.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lcbryant/桌面/myslam/result/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libslam_shared.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/slam_shared.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/slam_shared.dir/build: libslam_shared.so

.PHONY : CMakeFiles/slam_shared.dir/build

CMakeFiles/slam_shared.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/slam_shared.dir/cmake_clean.cmake
.PHONY : CMakeFiles/slam_shared.dir/clean

CMakeFiles/slam_shared.dir/depend:
	cd /home/lcbryant/桌面/myslam/result/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lcbryant/桌面/myslam/result /home/lcbryant/桌面/myslam/result /home/lcbryant/桌面/myslam/result/build /home/lcbryant/桌面/myslam/result/build /home/lcbryant/桌面/myslam/result/build/CMakeFiles/slam_shared.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/slam_shared.dir/depend

