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
CMAKE_SOURCE_DIR = /home/marceline/Desktop/BodySim/save_pcd_test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/marceline/Desktop/BodySim/save_pcd_test/build

# Include any dependencies generated for this target.
include CMakeFiles/kinectPCLviewer.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/kinectPCLviewer.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/kinectPCLviewer.dir/flags.make

CMakeFiles/kinectPCLviewer.dir/main.cpp.o: CMakeFiles/kinectPCLviewer.dir/flags.make
CMakeFiles/kinectPCLviewer.dir/main.cpp.o: ../main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/marceline/Desktop/BodySim/save_pcd_test/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/kinectPCLviewer.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/kinectPCLviewer.dir/main.cpp.o -c /home/marceline/Desktop/BodySim/save_pcd_test/main.cpp

CMakeFiles/kinectPCLviewer.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kinectPCLviewer.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/marceline/Desktop/BodySim/save_pcd_test/main.cpp > CMakeFiles/kinectPCLviewer.dir/main.cpp.i

CMakeFiles/kinectPCLviewer.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kinectPCLviewer.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/marceline/Desktop/BodySim/save_pcd_test/main.cpp -o CMakeFiles/kinectPCLviewer.dir/main.cpp.s

CMakeFiles/kinectPCLviewer.dir/main.cpp.o.requires:
.PHONY : CMakeFiles/kinectPCLviewer.dir/main.cpp.o.requires

CMakeFiles/kinectPCLviewer.dir/main.cpp.o.provides: CMakeFiles/kinectPCLviewer.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/kinectPCLviewer.dir/build.make CMakeFiles/kinectPCLviewer.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/kinectPCLviewer.dir/main.cpp.o.provides

CMakeFiles/kinectPCLviewer.dir/main.cpp.o.provides.build: CMakeFiles/kinectPCLviewer.dir/main.cpp.o

# Object files for target kinectPCLviewer
kinectPCLviewer_OBJECTS = \
"CMakeFiles/kinectPCLviewer.dir/main.cpp.o"

# External object files for target kinectPCLviewer
kinectPCLviewer_EXTERNAL_OBJECTS =

kinectPCLviewer: CMakeFiles/kinectPCLviewer.dir/main.cpp.o
kinectPCLviewer: /usr/lib/libboost_system-mt.so
kinectPCLviewer: /usr/lib/libboost_filesystem-mt.so
kinectPCLviewer: /usr/lib/libboost_thread-mt.so
kinectPCLviewer: /usr/lib/libboost_date_time-mt.so
kinectPCLviewer: /usr/lib/libboost_iostreams-mt.so
kinectPCLviewer: /usr/lib/libboost_serialization-mt.so
kinectPCLviewer: /usr/lib/libpcl_common.so
kinectPCLviewer: /usr/lib/libflann_cpp_s.a
kinectPCLviewer: /usr/lib/libpcl_kdtree.so
kinectPCLviewer: /usr/lib/libpcl_octree.so
kinectPCLviewer: /usr/lib/libpcl_search.so
kinectPCLviewer: /usr/lib/libqhull.so
kinectPCLviewer: /usr/lib/libpcl_surface.so
kinectPCLviewer: /usr/lib/libpcl_sample_consensus.so
kinectPCLviewer: /usr/lib/libpcl_filters.so
kinectPCLviewer: /usr/lib/libpcl_features.so
kinectPCLviewer: /usr/lib/libpcl_segmentation.so
kinectPCLviewer: /usr/lib/libOpenNI.so
kinectPCLviewer: /usr/lib/libvtkCommon.so.5.8.0
kinectPCLviewer: /usr/lib/libvtkRendering.so.5.8.0
kinectPCLviewer: /usr/lib/libvtkHybrid.so.5.8.0
kinectPCLviewer: /usr/lib/libvtkCharts.so.5.8.0
kinectPCLviewer: /usr/lib/libpcl_io.so
kinectPCLviewer: /usr/lib/libpcl_registration.so
kinectPCLviewer: /usr/lib/libpcl_keypoints.so
kinectPCLviewer: /usr/lib/libpcl_recognition.so
kinectPCLviewer: /usr/lib/libpcl_visualization.so
kinectPCLviewer: /usr/lib/libpcl_people.so
kinectPCLviewer: /usr/lib/libpcl_outofcore.so
kinectPCLviewer: /usr/lib/libpcl_tracking.so
kinectPCLviewer: /usr/lib/libpcl_apps.so
kinectPCLviewer: /usr/lib/libboost_system-mt.so
kinectPCLviewer: /usr/lib/libboost_filesystem-mt.so
kinectPCLviewer: /usr/lib/libboost_thread-mt.so
kinectPCLviewer: /usr/lib/libboost_date_time-mt.so
kinectPCLviewer: /usr/lib/libboost_iostreams-mt.so
kinectPCLviewer: /usr/lib/libboost_serialization-mt.so
kinectPCLviewer: /usr/lib/libqhull.so
kinectPCLviewer: /usr/lib/libOpenNI.so
kinectPCLviewer: /usr/lib/libflann_cpp_s.a
kinectPCLviewer: /usr/lib/libvtkCommon.so.5.8.0
kinectPCLviewer: /usr/lib/libvtkRendering.so.5.8.0
kinectPCLviewer: /usr/lib/libvtkHybrid.so.5.8.0
kinectPCLviewer: /usr/lib/libvtkCharts.so.5.8.0
kinectPCLviewer: /usr/lib/libpcl_common.so
kinectPCLviewer: /usr/lib/libpcl_kdtree.so
kinectPCLviewer: /usr/lib/libpcl_octree.so
kinectPCLviewer: /usr/lib/libpcl_search.so
kinectPCLviewer: /usr/lib/libpcl_surface.so
kinectPCLviewer: /usr/lib/libpcl_sample_consensus.so
kinectPCLviewer: /usr/lib/libpcl_filters.so
kinectPCLviewer: /usr/lib/libpcl_features.so
kinectPCLviewer: /usr/lib/libpcl_segmentation.so
kinectPCLviewer: /usr/lib/libpcl_io.so
kinectPCLviewer: /usr/lib/libpcl_registration.so
kinectPCLviewer: /usr/lib/libpcl_keypoints.so
kinectPCLviewer: /usr/lib/libpcl_recognition.so
kinectPCLviewer: /usr/lib/libpcl_visualization.so
kinectPCLviewer: /usr/lib/libpcl_people.so
kinectPCLviewer: /usr/lib/libpcl_outofcore.so
kinectPCLviewer: /usr/lib/libpcl_tracking.so
kinectPCLviewer: /usr/lib/libpcl_apps.so
kinectPCLviewer: /usr/lib/libvtkViews.so.5.8.0
kinectPCLviewer: /usr/lib/libvtkInfovis.so.5.8.0
kinectPCLviewer: /usr/lib/libvtkWidgets.so.5.8.0
kinectPCLviewer: /usr/lib/libvtkHybrid.so.5.8.0
kinectPCLviewer: /usr/lib/libvtkParallel.so.5.8.0
kinectPCLviewer: /usr/lib/libvtkVolumeRendering.so.5.8.0
kinectPCLviewer: /usr/lib/libvtkRendering.so.5.8.0
kinectPCLviewer: /usr/lib/libvtkGraphics.so.5.8.0
kinectPCLviewer: /usr/lib/libvtkImaging.so.5.8.0
kinectPCLviewer: /usr/lib/libvtkIO.so.5.8.0
kinectPCLviewer: /usr/lib/libvtkFiltering.so.5.8.0
kinectPCLviewer: /usr/lib/libvtkCommon.so.5.8.0
kinectPCLviewer: /usr/lib/libvtksys.so.5.8.0
kinectPCLviewer: CMakeFiles/kinectPCLviewer.dir/build.make
kinectPCLviewer: CMakeFiles/kinectPCLviewer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable kinectPCLviewer"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/kinectPCLviewer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/kinectPCLviewer.dir/build: kinectPCLviewer
.PHONY : CMakeFiles/kinectPCLviewer.dir/build

CMakeFiles/kinectPCLviewer.dir/requires: CMakeFiles/kinectPCLviewer.dir/main.cpp.o.requires
.PHONY : CMakeFiles/kinectPCLviewer.dir/requires

CMakeFiles/kinectPCLviewer.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/kinectPCLviewer.dir/cmake_clean.cmake
.PHONY : CMakeFiles/kinectPCLviewer.dir/clean

CMakeFiles/kinectPCLviewer.dir/depend:
	cd /home/marceline/Desktop/BodySim/save_pcd_test/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/marceline/Desktop/BodySim/save_pcd_test /home/marceline/Desktop/BodySim/save_pcd_test /home/marceline/Desktop/BodySim/save_pcd_test/build /home/marceline/Desktop/BodySim/save_pcd_test/build /home/marceline/Desktop/BodySim/save_pcd_test/build/CMakeFiles/kinectPCLviewer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/kinectPCLviewer.dir/depend

