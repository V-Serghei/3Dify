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

target_link_libraries(${CMAKE_PROJECT_NAME}
        pclibrary::pclibrary
        android
        log)
