# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/peter/project/samples/Stereo-Webcam-Depth-Detection/source_code/openCVProject/UseFaceDetection

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/peter/project/samples/Stereo-Webcam-Depth-Detection/source_code/openCVProject/UseFaceDetection/build

# Include any dependencies generated for this target.
include CMakeFiles/facedetection_camera.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/facedetection_camera.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/facedetection_camera.dir/flags.make

CMakeFiles/facedetection_camera.dir/facedetection_camera.cpp.o: CMakeFiles/facedetection_camera.dir/flags.make
CMakeFiles/facedetection_camera.dir/facedetection_camera.cpp.o: ../facedetection_camera.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/peter/project/samples/Stereo-Webcam-Depth-Detection/source_code/openCVProject/UseFaceDetection/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/facedetection_camera.dir/facedetection_camera.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/facedetection_camera.dir/facedetection_camera.cpp.o -c /home/peter/project/samples/Stereo-Webcam-Depth-Detection/source_code/openCVProject/UseFaceDetection/facedetection_camera.cpp

CMakeFiles/facedetection_camera.dir/facedetection_camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/facedetection_camera.dir/facedetection_camera.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/peter/project/samples/Stereo-Webcam-Depth-Detection/source_code/openCVProject/UseFaceDetection/facedetection_camera.cpp > CMakeFiles/facedetection_camera.dir/facedetection_camera.cpp.i

CMakeFiles/facedetection_camera.dir/facedetection_camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/facedetection_camera.dir/facedetection_camera.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/peter/project/samples/Stereo-Webcam-Depth-Detection/source_code/openCVProject/UseFaceDetection/facedetection_camera.cpp -o CMakeFiles/facedetection_camera.dir/facedetection_camera.cpp.s

CMakeFiles/facedetection_camera.dir/facedetection_camera.cpp.o.requires:

.PHONY : CMakeFiles/facedetection_camera.dir/facedetection_camera.cpp.o.requires

CMakeFiles/facedetection_camera.dir/facedetection_camera.cpp.o.provides: CMakeFiles/facedetection_camera.dir/facedetection_camera.cpp.o.requires
	$(MAKE) -f CMakeFiles/facedetection_camera.dir/build.make CMakeFiles/facedetection_camera.dir/facedetection_camera.cpp.o.provides.build
.PHONY : CMakeFiles/facedetection_camera.dir/facedetection_camera.cpp.o.provides

CMakeFiles/facedetection_camera.dir/facedetection_camera.cpp.o.provides.build: CMakeFiles/facedetection_camera.dir/facedetection_camera.cpp.o


# Object files for target facedetection_camera
facedetection_camera_OBJECTS = \
"CMakeFiles/facedetection_camera.dir/facedetection_camera.cpp.o"

# External object files for target facedetection_camera
facedetection_camera_EXTERNAL_OBJECTS =

facedetection_camera: CMakeFiles/facedetection_camera.dir/facedetection_camera.cpp.o
facedetection_camera: CMakeFiles/facedetection_camera.dir/build.make
facedetection_camera: ../lib/libseeta_facedet_lib.so
facedetection_camera: /usr/local/lib/libopencv_shape.so.3.2.0
facedetection_camera: /usr/local/lib/libopencv_stitching.so.3.2.0
facedetection_camera: /usr/local/lib/libopencv_superres.so.3.2.0
facedetection_camera: /usr/local/lib/libopencv_videostab.so.3.2.0
facedetection_camera: /usr/local/lib/libopencv_objdetect.so.3.2.0
facedetection_camera: /usr/local/lib/libopencv_calib3d.so.3.2.0
facedetection_camera: /usr/local/lib/libopencv_features2d.so.3.2.0
facedetection_camera: /usr/local/lib/libopencv_flann.so.3.2.0
facedetection_camera: /usr/local/lib/libopencv_highgui.so.3.2.0
facedetection_camera: /usr/local/lib/libopencv_ml.so.3.2.0
facedetection_camera: /usr/local/lib/libopencv_photo.so.3.2.0
facedetection_camera: /usr/local/lib/libopencv_video.so.3.2.0
facedetection_camera: /usr/local/lib/libopencv_videoio.so.3.2.0
facedetection_camera: /usr/local/lib/libopencv_imgcodecs.so.3.2.0
facedetection_camera: /usr/local/lib/libopencv_imgproc.so.3.2.0
facedetection_camera: /usr/local/lib/libopencv_core.so.3.2.0
facedetection_camera: CMakeFiles/facedetection_camera.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/peter/project/samples/Stereo-Webcam-Depth-Detection/source_code/openCVProject/UseFaceDetection/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable facedetection_camera"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/facedetection_camera.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/facedetection_camera.dir/build: facedetection_camera

.PHONY : CMakeFiles/facedetection_camera.dir/build

CMakeFiles/facedetection_camera.dir/requires: CMakeFiles/facedetection_camera.dir/facedetection_camera.cpp.o.requires

.PHONY : CMakeFiles/facedetection_camera.dir/requires

CMakeFiles/facedetection_camera.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/facedetection_camera.dir/cmake_clean.cmake
.PHONY : CMakeFiles/facedetection_camera.dir/clean

CMakeFiles/facedetection_camera.dir/depend:
	cd /home/peter/project/samples/Stereo-Webcam-Depth-Detection/source_code/openCVProject/UseFaceDetection/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/peter/project/samples/Stereo-Webcam-Depth-Detection/source_code/openCVProject/UseFaceDetection /home/peter/project/samples/Stereo-Webcam-Depth-Detection/source_code/openCVProject/UseFaceDetection /home/peter/project/samples/Stereo-Webcam-Depth-Detection/source_code/openCVProject/UseFaceDetection/build /home/peter/project/samples/Stereo-Webcam-Depth-Detection/source_code/openCVProject/UseFaceDetection/build /home/peter/project/samples/Stereo-Webcam-Depth-Detection/source_code/openCVProject/UseFaceDetection/build/CMakeFiles/facedetection_camera.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/facedetection_camera.dir/depend

