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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/marceline/Desktop/BodySim/scanning

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/marceline/Desktop/BodySim/scanning/build

# Include any dependencies generated for this target.
include CMakeFiles/scanning.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/scanning.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/scanning.dir/flags.make

CMakeFiles/scanning.dir/scanning.cpp.o: CMakeFiles/scanning.dir/flags.make
CMakeFiles/scanning.dir/scanning.cpp.o: ../scanning.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/marceline/Desktop/BodySim/scanning/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/scanning.dir/scanning.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/scanning.dir/scanning.cpp.o -c /home/marceline/Desktop/BodySim/scanning/scanning.cpp

CMakeFiles/scanning.dir/scanning.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/scanning.dir/scanning.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/marceline/Desktop/BodySim/scanning/scanning.cpp > CMakeFiles/scanning.dir/scanning.cpp.i

CMakeFiles/scanning.dir/scanning.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/scanning.dir/scanning.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/marceline/Desktop/BodySim/scanning/scanning.cpp -o CMakeFiles/scanning.dir/scanning.cpp.s

CMakeFiles/scanning.dir/scanning.cpp.o.requires:
.PHONY : CMakeFiles/scanning.dir/scanning.cpp.o.requires

CMakeFiles/scanning.dir/scanning.cpp.o.provides: CMakeFiles/scanning.dir/scanning.cpp.o.requires
	$(MAKE) -f CMakeFiles/scanning.dir/build.make CMakeFiles/scanning.dir/scanning.cpp.o.provides.build
.PHONY : CMakeFiles/scanning.dir/scanning.cpp.o.provides

CMakeFiles/scanning.dir/scanning.cpp.o.provides.build: CMakeFiles/scanning.dir/scanning.cpp.o

# Object files for target scanning
scanning_OBJECTS = \
"CMakeFiles/scanning.dir/scanning.cpp.o"

# External object files for target scanning
scanning_EXTERNAL_OBJECTS =

scanning: CMakeFiles/scanning.dir/scanning.cpp.o
scanning: /usr/lib/libboost_system-mt.so
scanning: /usr/lib/libboost_filesystem-mt.so
scanning: /usr/lib/libboost_thread-mt.so
scanning: /usr/lib/libboost_date_time-mt.so
scanning: /usr/lib/libboost_iostreams-mt.so
scanning: /usr/lib/libboost_serialization-mt.so
scanning: /usr/lib/libpcl_common.so
scanning: /usr/lib/libflann_cpp_s.a
scanning: /usr/lib/libpcl_kdtree.so
scanning: /usr/lib/libpcl_octree.so
scanning: /usr/lib/libpcl_search.so
scanning: /usr/lib/libqhull.so
scanning: /usr/lib/libpcl_surface.so
scanning: /usr/lib/libpcl_sample_consensus.so
scanning: /usr/lib/libpcl_filters.so
scanning: /usr/lib/libpcl_features.so
scanning: /usr/lib/libpcl_segmentation.so
scanning: /usr/lib/libOpenNI.so
scanning: /usr/lib/libvtkCommon.so.5.8.0
scanning: /usr/lib/libvtkRendering.so.5.8.0
scanning: /usr/lib/libvtkHybrid.so.5.8.0
scanning: /usr/lib/libvtkCharts.so.5.8.0
scanning: /usr/lib/libpcl_io.so
scanning: /usr/lib/libpcl_registration.so
scanning: /usr/lib/libpcl_keypoints.so
scanning: /usr/lib/libpcl_recognition.so
scanning: /usr/lib/libpcl_visualization.so
scanning: /usr/lib/libpcl_people.so
scanning: /usr/lib/libpcl_outofcore.so
scanning: /usr/lib/libpcl_tracking.so
scanning: /usr/lib/libpcl_apps.so
scanning: /usr/lib/libboost_system-mt.so
scanning: /usr/lib/libboost_filesystem-mt.so
scanning: /usr/lib/libboost_thread-mt.so
scanning: /usr/lib/libboost_date_time-mt.so
scanning: /usr/lib/libboost_iostreams-mt.so
scanning: /usr/lib/libboost_serialization-mt.so
scanning: /usr/lib/libqhull.so
scanning: /usr/lib/libOpenNI.so
scanning: /usr/lib/libflann_cpp_s.a
scanning: /usr/lib/libvtkCommon.so.5.8.0
scanning: /usr/lib/libvtkRendering.so.5.8.0
scanning: /usr/lib/libvtkHybrid.so.5.8.0
scanning: /usr/lib/libvtkCharts.so.5.8.0
scanning: /usr/lib/libpcl_common.so
scanning: /usr/lib/libpcl_kdtree.so
scanning: /usr/lib/libpcl_octree.so
scanning: /usr/lib/libpcl_search.so
scanning: /usr/lib/libpcl_surface.so
scanning: /usr/lib/libpcl_sample_consensus.so
scanning: /usr/lib/libpcl_filters.so
scanning: /usr/lib/libpcl_features.so
scanning: /usr/lib/libpcl_segmentation.so
scanning: /usr/lib/libpcl_io.so
scanning: /usr/lib/libpcl_registration.so
scanning: /usr/lib/libpcl_keypoints.so
scanning: /usr/lib/libpcl_recognition.so
scanning: /usr/lib/libpcl_visualization.so
scanning: /usr/lib/libpcl_people.so
scanning: /usr/lib/libpcl_outofcore.so
scanning: /usr/lib/libpcl_tracking.so
scanning: /usr/lib/libpcl_apps.so
scanning: /usr/lib/libvtkViews.so.5.8.0
scanning: /usr/lib/libvtkInfovis.so.5.8.0
scanning: /usr/lib/libvtkWidgets.so.5.8.0
scanning: /usr/lib/libvtkHybrid.so.5.8.0
scanning: /usr/lib/libvtkParallel.so.5.8.0
scanning: /usr/lib/libvtkVolumeRendering.so.5.8.0
scanning: /usr/lib/libvtkRendering.so.5.8.0
scanning: /usr/lib/libvtkGraphics.so.5.8.0
scanning: /usr/lib/libvtkImaging.so.5.8.0
scanning: /usr/lib/libvtkIO.so.5.8.0
scanning: /usr/lib/libvtkFiltering.so.5.8.0
scanning: /usr/lib/libvtkCommon.so.5.8.0
scanning: /usr/lib/libvtksys.so.5.8.0
scanning: CMakeFiles/scanning.dir/build.make
scanning: CMakeFiles/scanning.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable scanning"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/scanning.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/scanning.dir/build: scanning
.PHONY : CMakeFiles/scanning.dir/build

CMakeFiles/scanning.dir/requires: CMakeFiles/scanning.dir/scanning.cpp.o.requires
.PHONY : CMakeFiles/scanning.dir/requires

CMakeFiles/scanning.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/scanning.dir/cmake_clean.cmake
.PHONY : CMakeFiles/scanning.dir/clean

CMakeFiles/scanning.dir/depend:
	cd /home/marceline/Desktop/BodySim/scanning/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/marceline/Desktop/BodySim/scanning /home/marceline/Desktop/BodySim/scanning /home/marceline/Desktop/BodySim/scanning/build /home/marceline/Desktop/BodySim/scanning/build /home/marceline/Desktop/BodySim/scanning/build/CMakeFiles/scanning.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/scanning.dir/depend
