# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/yukisaito/ORB-SLAM2_proposed/ORB_SLAM2-master

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yukisaito/ORB-SLAM2_proposed/ORB_SLAM2-master

# Include any dependencies generated for this target.
include CMakeFiles/webcam_monodepth.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/webcam_monodepth.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/webcam_monodepth.dir/flags.make

CMakeFiles/webcam_monodepth.dir/Examples/rgbd_monodepth/webcam.cc.o: CMakeFiles/webcam_monodepth.dir/flags.make
CMakeFiles/webcam_monodepth.dir/Examples/rgbd_monodepth/webcam.cc.o: Examples/rgbd_monodepth/webcam.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yukisaito/ORB-SLAM2_proposed/ORB_SLAM2-master/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/webcam_monodepth.dir/Examples/rgbd_monodepth/webcam.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/webcam_monodepth.dir/Examples/rgbd_monodepth/webcam.cc.o -c /home/yukisaito/ORB-SLAM2_proposed/ORB_SLAM2-master/Examples/rgbd_monodepth/webcam.cc

CMakeFiles/webcam_monodepth.dir/Examples/rgbd_monodepth/webcam.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/webcam_monodepth.dir/Examples/rgbd_monodepth/webcam.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yukisaito/ORB-SLAM2_proposed/ORB_SLAM2-master/Examples/rgbd_monodepth/webcam.cc > CMakeFiles/webcam_monodepth.dir/Examples/rgbd_monodepth/webcam.cc.i

CMakeFiles/webcam_monodepth.dir/Examples/rgbd_monodepth/webcam.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/webcam_monodepth.dir/Examples/rgbd_monodepth/webcam.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yukisaito/ORB-SLAM2_proposed/ORB_SLAM2-master/Examples/rgbd_monodepth/webcam.cc -o CMakeFiles/webcam_monodepth.dir/Examples/rgbd_monodepth/webcam.cc.s

# Object files for target webcam_monodepth
webcam_monodepth_OBJECTS = \
"CMakeFiles/webcam_monodepth.dir/Examples/rgbd_monodepth/webcam.cc.o"

# External object files for target webcam_monodepth
webcam_monodepth_EXTERNAL_OBJECTS =

Examples/rgbd_monodepth/webcam_monodepth: CMakeFiles/webcam_monodepth.dir/Examples/rgbd_monodepth/webcam.cc.o
Examples/rgbd_monodepth/webcam_monodepth: CMakeFiles/webcam_monodepth.dir/build.make
Examples/rgbd_monodepth/webcam_monodepth: libCNN_MonoFusion.a
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_stitching.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_superres.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_videostab.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_aruco.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_bgsegm.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_bioinspired.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_ccalib.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_dnn_objdetect.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_dpm.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_highgui.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_videoio.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_face.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_freetype.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_fuzzy.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_hfs.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_img_hash.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_line_descriptor.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_optflow.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_reg.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_rgbd.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_saliency.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_stereo.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_structured_light.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_phase_unwrapping.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_surface_matching.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_tracking.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_datasets.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_plot.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_text.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_dnn.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_xfeatures2d.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_ml.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_shape.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_video.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_ximgproc.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_xobjdetect.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_imgcodecs.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_objdetect.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_calib3d.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_features2d.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_flann.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_xphoto.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_photo.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_imgproc.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/opencv-3.4.9/build/lib/libopencv_core.so.3.4.9
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/ORB-SLAM2_proposed/Pangolin/build/src/libpangolin.so
Examples/rgbd_monodepth/webcam_monodepth: /usr/lib/x86_64-linux-gnu/libOpenGL.so
Examples/rgbd_monodepth/webcam_monodepth: /usr/lib/x86_64-linux-gnu/libGLX.so
Examples/rgbd_monodepth/webcam_monodepth: /usr/lib/x86_64-linux-gnu/libGLU.so
Examples/rgbd_monodepth/webcam_monodepth: /usr/lib/x86_64-linux-gnu/libGLEW.so
Examples/rgbd_monodepth/webcam_monodepth: /usr/lib/x86_64-linux-gnu/libSM.so
Examples/rgbd_monodepth/webcam_monodepth: /usr/lib/x86_64-linux-gnu/libICE.so
Examples/rgbd_monodepth/webcam_monodepth: /usr/lib/x86_64-linux-gnu/libX11.so
Examples/rgbd_monodepth/webcam_monodepth: /usr/lib/x86_64-linux-gnu/libXext.so
Examples/rgbd_monodepth/webcam_monodepth: /usr/lib/x86_64-linux-gnu/libOpenGL.so
Examples/rgbd_monodepth/webcam_monodepth: /usr/lib/x86_64-linux-gnu/libGLX.so
Examples/rgbd_monodepth/webcam_monodepth: /usr/lib/x86_64-linux-gnu/libGLU.so
Examples/rgbd_monodepth/webcam_monodepth: /usr/lib/x86_64-linux-gnu/libGLEW.so
Examples/rgbd_monodepth/webcam_monodepth: /usr/lib/x86_64-linux-gnu/libSM.so
Examples/rgbd_monodepth/webcam_monodepth: /usr/lib/x86_64-linux-gnu/libICE.so
Examples/rgbd_monodepth/webcam_monodepth: /usr/lib/x86_64-linux-gnu/libX11.so
Examples/rgbd_monodepth/webcam_monodepth: /usr/lib/x86_64-linux-gnu/libXext.so
Examples/rgbd_monodepth/webcam_monodepth: /usr/lib/x86_64-linux-gnu/libdc1394.so
Examples/rgbd_monodepth/webcam_monodepth: /usr/lib/x86_64-linux-gnu/libpng.so
Examples/rgbd_monodepth/webcam_monodepth: /usr/lib/x86_64-linux-gnu/libz.so
Examples/rgbd_monodepth/webcam_monodepth: /usr/lib/x86_64-linux-gnu/libjpeg.so
Examples/rgbd_monodepth/webcam_monodepth: /usr/lib/x86_64-linux-gnu/libtiff.so
Examples/rgbd_monodepth/webcam_monodepth: /usr/lib/x86_64-linux-gnu/libIlmImf.so
Examples/rgbd_monodepth/webcam_monodepth: Thirdparty/DBoW2/lib/libDBoW2.so
Examples/rgbd_monodepth/webcam_monodepth: Thirdparty/g2o/lib/libg2o.so
Examples/rgbd_monodepth/webcam_monodepth: /home/yukisaito/anaconda3/envs/py36/lib/libpython3.6m.so
Examples/rgbd_monodepth/webcam_monodepth: CMakeFiles/webcam_monodepth.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yukisaito/ORB-SLAM2_proposed/ORB_SLAM2-master/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable Examples/rgbd_monodepth/webcam_monodepth"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/webcam_monodepth.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/webcam_monodepth.dir/build: Examples/rgbd_monodepth/webcam_monodepth

.PHONY : CMakeFiles/webcam_monodepth.dir/build

CMakeFiles/webcam_monodepth.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/webcam_monodepth.dir/cmake_clean.cmake
.PHONY : CMakeFiles/webcam_monodepth.dir/clean

CMakeFiles/webcam_monodepth.dir/depend:
	cd /home/yukisaito/ORB-SLAM2_proposed/ORB_SLAM2-master && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yukisaito/ORB-SLAM2_proposed/ORB_SLAM2-master /home/yukisaito/ORB-SLAM2_proposed/ORB_SLAM2-master /home/yukisaito/ORB-SLAM2_proposed/ORB_SLAM2-master /home/yukisaito/ORB-SLAM2_proposed/ORB_SLAM2-master /home/yukisaito/ORB-SLAM2_proposed/ORB_SLAM2-master/CMakeFiles/webcam_monodepth.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/webcam_monodepth.dir/depend
