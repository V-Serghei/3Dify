cmake_minimum_required(VERSION 3.22.1)

project("nativelib")

add_library(${CMAKE_PROJECT_NAME} SHARED
        nativelib.cpp
        PointCloudTo3DModel.cpp)

find_package(pclibrary REQUIRED CONFIG)

# FLANN headers in the PCL package use std::binary_function which was removed in C++17.
# This flag re-enables deprecated C++17 features in libc++ so the headers compile cleanly.
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        _LIBCPP_ENABLE_CXX17_REMOVED_FEATURES)

# Align LOAD segments to 16KB for Android 15+ compatibility (NDK 26 requires explicit flag)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -Wl,-z,max-page-size=16384)

# libpclibrary.so is linked against OpenMP; -fopenmp tells the NDK linker to
# include libomp.so from the toolchain into the APK's lib/arm64-v8a/ directory.
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -fopenmp)
target_link_libraries(${CMAKE_PROJECT_NAME}
        pclibrary::pclibrary
        -fopenmp
        android
        log)
