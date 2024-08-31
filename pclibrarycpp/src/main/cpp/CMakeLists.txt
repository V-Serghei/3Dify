cmake_minimum_required(VERSION 3.4.1)
project(pclibrarycpp)

set(PCL_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/pcl/include")
set(PCL_LIBRARY_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/pcl/lib")

set(PCL_LIBRARIES
        "${PCL_LIBRARY_DIRS}/libpcl_common.so"
        "${PCL_LIBRARY_DIRS}/libpcl_io.so"
        "${PCL_LIBRARY_DIRS}/libpcl_filters.so"
        "${PCL_LIBRARY_DIRS}/libpcl_kdtree.so"
        "${PCL_LIBRARY_DIRS}/libpcl_search.so"
        "${PCL_LIBRARY_DIRS}/libpcl_sample_consensus.so"
        # Добавьте другие необходимые библиотеки PCL
)

include_directories(${PCL_INCLUDE_DIRS})

add_library(${CMAKE_PROJECT_NAME} SHARED pclibrarycpp.cpp)

find_library(log-lib log)

target_link_libraries(${CMAKE_PROJECT_NAME}
        android
        ${log-lib}
        ${PCL_LIBRARIES}
)
