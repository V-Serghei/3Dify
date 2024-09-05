# For more information about using CMake with Android Studio, read the
# documentation: https://d.android.com/studio/projects/add-native-code.html.
# For more examples on how to use CMake, see https://github.com/android/ndk-samples.

# Sets the minimum CMake version required for this project.
cmake_minimum_required(VERSION 3.22.1)

# Declares the project name. The project name can be accessed via ${ PROJECT_NAME},
# Since this is the top level CMakeLists.txt, the project name is also accessible
# with ${CMAKE_PROJECT_NAME} (both CMake variables are in-sync within the top level
# build script scope).
project("nativelib")
set(BOOST_ROOT "pclib/boost")
set(EIGEN_ROOT "pclib/eigen")
set(FLANN_ROOT "pclib/flann")
set(LZ4_ROOT "pclib/lz4")
set(PCL_ROOT "pclib/pcl")
include_directories(${BOOST_ROOT}/include ${EIGEN_ROOT}/include/eigen3 ${FLANN_ROOT}/include ${LZ4_ROOT}/include ${PCL_ROOT}/include)
link_directories(${BOOST_ROOT}/lib ${FLANN_ROOT}/lib ${LZ4_ROOT}/lib ${PCL_ROOT}/lib)
# Creates and names a library, sets it as either STATIC
# or SHARED, and provides the relative paths to its source code.
# You can define multiple libraries, and CMake builds them for you.
# Gradle automatically packages shared libraries with your APK.
#
# In this top level CMakeLists.txt, ${CMAKE_PROJECT_NAME} is used to define
# the target library name; in the sub-module's CMakeLists.txt, ${PROJECT_NAME}
# is preferred for the same purpose.
#
# In order to load a library into your app from Java/Kotlin, you must call
# System.loadLibrary() and pass the name of the library defined here;
# for GameActivity/NativeActivity derived applications, the same library name must be
# used in the AndroidManifest.xml file.
add_library(${CMAKE_PROJECT_NAME} SHARED
        # List C/C++ source files with relative paths to this CMakeLists.txt.
        nativelib.cpp)

# Specifies libraries CMake should link to your target library. You
# can link libraries from various origins, such as libraries defined in this
# build script, prebuilt third-party libraries, or Android system libraries.
target_link_libraries(${CMAKE_PROJECT_NAME}
        -lpcl_common
        -lpcl_io
        -lpcl_io_ply
        -lpcl_kdtree
        -lpcl_features
        -lpcl_filters
        -lpcl_segmentation
        -lpcl_tracking
        -lpcl_recognition
        -lpcl_ml
        -lpcl_search
        -lpcl_registration
        -lpcl_keypoints
        -lpcl_sample_consensus
        -lpcl_surface
        -lpcl_octree
        -lpcl_stereo
        -lboost_system
        -lboost_filesystem
        -lboost_thread
        -lboost_chrono
        -lboost_date_time
        -lboost_graph
        -lboost_log
        -lboost_log_setup
        -lboost_program_options
        -lboost_random
        -lboost_math_tr1
        -lboost_exception
        -lboost_type_erasure
        -lboost_container
        -lboost_iostreams
        -lboost_stacktrace_noop
        -lboost_fiber
        -lboost_math_c99
        -lboost_stacktrace_basic
        -lboost_wserialization
        -lboost_wave
        -lboost_serialization
        -lboost_unit_test_framework
        -lboost_prg_exec_monitor
        -lboost_timer
        -lboost_context
        -lboost_log_setup
        -lboost_log
        -lboost_program_options
        -landroid
        -llog
        -latomic
        -lm
)
