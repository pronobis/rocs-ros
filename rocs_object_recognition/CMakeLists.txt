cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#find_package(OpenCV REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS})
rosbuild_add_executable(latentsvm_multidetect src/latentsvm_multidetect.cpp)

#define some target ...
#target_link_libraries(latentsvm_multidetect ${OpenCV_LIBS})

#rosbuild_add_executable(matcher_simple src/matcher_simple.cpp)
#rosbuild_add_executable(MatchTemplate_demo src/MatchTemplate_Demo.cpp)
#rosbuild_add_executable(cornerHarris_Demo src/cornerHarris_Demo.cpp)
#rosbuild_add_executable(surf_desciptor src/SURF_descriptor.cpp)
#rosbuild_add_executable(EqualizeHist_Demo src/EqualizeHist_Demo.cpp)
#rosbuild_add_executable(calcHist_Demo src/calcHist_Demo.cpp)
#rosbuild_add_executable(findContours_demo src/findContours_demo.cpp)
#rosbuild_add_executable(SURF_Homography src/SURF_Homography.cpp)
#rosbuild_add_executable(objectDetection1 src/objectDetection1.cpp)
#rosbuild_add_executable(drawKeyPoints src/drawKeyPoints.cpp)
#rosbuild_add_executable(main src/main.cpp)

#rosbuild_add_executable(rocs_object_recognition src/rocs_object_recognition.cpp src/SURFextractor.cpp)
#rosbuild_add_executable(DisplayImage src/DisplayImage.cpp)

