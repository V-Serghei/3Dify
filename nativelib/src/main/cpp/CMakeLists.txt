cmake_minimum_required(VERSION 3.22.1)

project("nativelib")

add_library(${CMAKE_PROJECT_NAME} SHARED
        nativelib.cpp
        PointCloudTo3DModel.cpp)

find_package(pclibrary REQUIRED CONFIG)

target_link_libraries(${CMAKE_PROJECT_NAME}
        pclibrary::pclibrary
        android
        log)
