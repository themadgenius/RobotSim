# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/build

# Include any dependencies generated for this target.
include hector_slam/hector_geotiff/CMakeFiles/geotiff_writer.dir/depend.make

# Include the progress variables for this target.
include hector_slam/hector_geotiff/CMakeFiles/geotiff_writer.dir/progress.make

# Include the compile flags for this target's objects.
include hector_slam/hector_geotiff/CMakeFiles/geotiff_writer.dir/flags.make

hector_slam/hector_geotiff/CMakeFiles/geotiff_writer.dir/src/geotiff_writer/geotiff_writer.cpp.o: hector_slam/hector_geotiff/CMakeFiles/geotiff_writer.dir/flags.make
hector_slam/hector_geotiff/CMakeFiles/geotiff_writer.dir/src/geotiff_writer/geotiff_writer.cpp.o: /home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/src/hector_slam/hector_geotiff/src/geotiff_writer/geotiff_writer.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object hector_slam/hector_geotiff/CMakeFiles/geotiff_writer.dir/src/geotiff_writer/geotiff_writer.cpp.o"
	cd /home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/build/hector_slam/hector_geotiff && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/geotiff_writer.dir/src/geotiff_writer/geotiff_writer.cpp.o -c /home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/src/hector_slam/hector_geotiff/src/geotiff_writer/geotiff_writer.cpp

hector_slam/hector_geotiff/CMakeFiles/geotiff_writer.dir/src/geotiff_writer/geotiff_writer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/geotiff_writer.dir/src/geotiff_writer/geotiff_writer.cpp.i"
	cd /home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/build/hector_slam/hector_geotiff && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/src/hector_slam/hector_geotiff/src/geotiff_writer/geotiff_writer.cpp > CMakeFiles/geotiff_writer.dir/src/geotiff_writer/geotiff_writer.cpp.i

hector_slam/hector_geotiff/CMakeFiles/geotiff_writer.dir/src/geotiff_writer/geotiff_writer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/geotiff_writer.dir/src/geotiff_writer/geotiff_writer.cpp.s"
	cd /home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/build/hector_slam/hector_geotiff && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/src/hector_slam/hector_geotiff/src/geotiff_writer/geotiff_writer.cpp -o CMakeFiles/geotiff_writer.dir/src/geotiff_writer/geotiff_writer.cpp.s

hector_slam/hector_geotiff/CMakeFiles/geotiff_writer.dir/src/geotiff_writer/geotiff_writer.cpp.o.requires:
.PHONY : hector_slam/hector_geotiff/CMakeFiles/geotiff_writer.dir/src/geotiff_writer/geotiff_writer.cpp.o.requires

hector_slam/hector_geotiff/CMakeFiles/geotiff_writer.dir/src/geotiff_writer/geotiff_writer.cpp.o.provides: hector_slam/hector_geotiff/CMakeFiles/geotiff_writer.dir/src/geotiff_writer/geotiff_writer.cpp.o.requires
	$(MAKE) -f hector_slam/hector_geotiff/CMakeFiles/geotiff_writer.dir/build.make hector_slam/hector_geotiff/CMakeFiles/geotiff_writer.dir/src/geotiff_writer/geotiff_writer.cpp.o.provides.build
.PHONY : hector_slam/hector_geotiff/CMakeFiles/geotiff_writer.dir/src/geotiff_writer/geotiff_writer.cpp.o.provides

hector_slam/hector_geotiff/CMakeFiles/geotiff_writer.dir/src/geotiff_writer/geotiff_writer.cpp.o.provides.build: hector_slam/hector_geotiff/CMakeFiles/geotiff_writer.dir/src/geotiff_writer/geotiff_writer.cpp.o

# Object files for target geotiff_writer
geotiff_writer_OBJECTS = \
"CMakeFiles/geotiff_writer.dir/src/geotiff_writer/geotiff_writer.cpp.o"

# External object files for target geotiff_writer
geotiff_writer_EXTERNAL_OBJECTS =

/home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/devel/lib/libgeotiff_writer.so: hector_slam/hector_geotiff/CMakeFiles/geotiff_writer.dir/src/geotiff_writer/geotiff_writer.cpp.o
/home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/devel/lib/libgeotiff_writer.so: hector_slam/hector_geotiff/CMakeFiles/geotiff_writer.dir/build.make
/home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/devel/lib/libgeotiff_writer.so: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/devel/lib/libgeotiff_writer.so: /opt/ros/indigo/lib/libclass_loader.so
/home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/devel/lib/libgeotiff_writer.so: /usr/lib/libPocoFoundation.so
/home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/devel/lib/libgeotiff_writer.so: /usr/lib/x86_64-linux-gnu/libdl.so
/home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/devel/lib/libgeotiff_writer.so: /opt/ros/indigo/lib/libroslib.so
/home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/devel/lib/libgeotiff_writer.so: /opt/ros/indigo/lib/libroscpp.so
/home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/devel/lib/libgeotiff_writer.so: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/devel/lib/libgeotiff_writer.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/devel/lib/libgeotiff_writer.so: /opt/ros/indigo/lib/librosconsole.so
/home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/devel/lib/libgeotiff_writer.so: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/devel/lib/libgeotiff_writer.so: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/devel/lib/libgeotiff_writer.so: /usr/lib/liblog4cxx.so
/home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/devel/lib/libgeotiff_writer.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/devel/lib/libgeotiff_writer.so: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/devel/lib/libgeotiff_writer.so: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/devel/lib/libgeotiff_writer.so: /opt/ros/indigo/lib/librostime.so
/home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/devel/lib/libgeotiff_writer.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/devel/lib/libgeotiff_writer.so: /opt/ros/indigo/lib/libcpp_common.so
/home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/devel/lib/libgeotiff_writer.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/devel/lib/libgeotiff_writer.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/devel/lib/libgeotiff_writer.so: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/devel/lib/libgeotiff_writer.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/devel/lib/libgeotiff_writer.so: /usr/lib/x86_64-linux-gnu/libQtGui.so
/home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/devel/lib/libgeotiff_writer.so: /usr/lib/x86_64-linux-gnu/libQtCore.so
/home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/devel/lib/libgeotiff_writer.so: hector_slam/hector_geotiff/CMakeFiles/geotiff_writer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library /home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/devel/lib/libgeotiff_writer.so"
	cd /home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/build/hector_slam/hector_geotiff && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/geotiff_writer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
hector_slam/hector_geotiff/CMakeFiles/geotiff_writer.dir/build: /home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/devel/lib/libgeotiff_writer.so
.PHONY : hector_slam/hector_geotiff/CMakeFiles/geotiff_writer.dir/build

hector_slam/hector_geotiff/CMakeFiles/geotiff_writer.dir/requires: hector_slam/hector_geotiff/CMakeFiles/geotiff_writer.dir/src/geotiff_writer/geotiff_writer.cpp.o.requires
.PHONY : hector_slam/hector_geotiff/CMakeFiles/geotiff_writer.dir/requires

hector_slam/hector_geotiff/CMakeFiles/geotiff_writer.dir/clean:
	cd /home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/build/hector_slam/hector_geotiff && $(CMAKE_COMMAND) -P CMakeFiles/geotiff_writer.dir/cmake_clean.cmake
.PHONY : hector_slam/hector_geotiff/CMakeFiles/geotiff_writer.dir/clean

hector_slam/hector_geotiff/CMakeFiles/geotiff_writer.dir/depend:
	cd /home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/src /home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/src/hector_slam/hector_geotiff /home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/build /home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/build/hector_slam/hector_geotiff /home/madgenius/projects/RobotSim/ros/indigo/catkin_ws/src/hector_quadrotor_tutorial/build/hector_slam/hector_geotiff/CMakeFiles/geotiff_writer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : hector_slam/hector_geotiff/CMakeFiles/geotiff_writer.dir/depend

