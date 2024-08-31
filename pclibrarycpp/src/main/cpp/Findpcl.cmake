

    function(conan_message MESSAGE_OUTPUT)
        if(NOT CONAN_CMAKE_SILENT_OUTPUT)
            message(${ARGV${0}})
        endif()
    endfunction()


    macro(conan_find_apple_frameworks FRAMEWORKS_FOUND FRAMEWORKS FRAMEWORKS_DIRS)
        if(APPLE)
            foreach(_FRAMEWORK ${FRAMEWORKS})
                # https://cmake.org/pipermail/cmake-developers/2017-August/030199.html
                find_library(CONAN_FRAMEWORK_${_FRAMEWORK}_FOUND NAMES ${_FRAMEWORK} PATHS ${FRAMEWORKS_DIRS} CMAKE_FIND_ROOT_PATH_BOTH)
                if(CONAN_FRAMEWORK_${_FRAMEWORK}_FOUND)
                    list(APPEND ${FRAMEWORKS_FOUND} ${CONAN_FRAMEWORK_${_FRAMEWORK}_FOUND})
                else()
                    message(FATAL_ERROR "Framework library ${_FRAMEWORK} not found in paths: ${FRAMEWORKS_DIRS}")
                endif()
            endforeach()
        endif()
    endmacro()


    function(conan_package_library_targets libraries package_libdir deps out_libraries out_libraries_target build_type package_name)
        unset(_CONAN_ACTUAL_TARGETS CACHE)
        unset(_CONAN_FOUND_SYSTEM_LIBS CACHE)
        foreach(_LIBRARY_NAME ${libraries})
            find_library(CONAN_FOUND_LIBRARY NAMES ${_LIBRARY_NAME} PATHS ${package_libdir}
                         NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
            if(CONAN_FOUND_LIBRARY)
                conan_message(STATUS "Library ${_LIBRARY_NAME} found ${CONAN_FOUND_LIBRARY}")
                list(APPEND _out_libraries ${CONAN_FOUND_LIBRARY})
                if(NOT ${CMAKE_VERSION} VERSION_LESS "3.0")
                    # Create a micro-target for each lib/a found
                    string(REGEX REPLACE "[^A-Za-z0-9.+_-]" "_" _LIBRARY_NAME ${_LIBRARY_NAME})
                    set(_LIB_NAME CONAN_LIB::${package_name}_${_LIBRARY_NAME}${build_type})
                    if(NOT TARGET ${_LIB_NAME})
                        # Create a micro-target for each lib/a found
                        add_library(${_LIB_NAME} UNKNOWN IMPORTED)
                        set_target_properties(${_LIB_NAME} PROPERTIES IMPORTED_LOCATION ${CONAN_FOUND_LIBRARY})
                        set(_CONAN_ACTUAL_TARGETS ${_CONAN_ACTUAL_TARGETS} ${_LIB_NAME})
                    else()
                        conan_message(STATUS "Skipping already existing target: ${_LIB_NAME}")
                    endif()
                    list(APPEND _out_libraries_target ${_LIB_NAME})
                endif()
                conan_message(STATUS "Found: ${CONAN_FOUND_LIBRARY}")
            else()
                conan_message(STATUS "Library ${_LIBRARY_NAME} not found in package, might be system one")
                list(APPEND _out_libraries_target ${_LIBRARY_NAME})
                list(APPEND _out_libraries ${_LIBRARY_NAME})
                set(_CONAN_FOUND_SYSTEM_LIBS "${_CONAN_FOUND_SYSTEM_LIBS};${_LIBRARY_NAME}")
            endif()
            unset(CONAN_FOUND_LIBRARY CACHE)
        endforeach()

        if(NOT ${CMAKE_VERSION} VERSION_LESS "3.0")
            # Add all dependencies to all targets
            string(REPLACE " " ";" deps_list "${deps}")
            foreach(_CONAN_ACTUAL_TARGET ${_CONAN_ACTUAL_TARGETS})
                set_property(TARGET ${_CONAN_ACTUAL_TARGET} PROPERTY INTERFACE_LINK_LIBRARIES "${_CONAN_FOUND_SYSTEM_LIBS};${deps_list}")
            endforeach()
        endif()

        set(${out_libraries} ${_out_libraries} PARENT_SCOPE)
        set(${out_libraries_target} ${_out_libraries_target} PARENT_SCOPE)
    endfunction()


    include(FindPackageHandleStandardArgs)

    conan_message(STATUS "Conan: Using autogenerated Findpcl.cmake")
    # Global approach
    set(pcl_FOUND 1)
    set(pcl_VERSION "1.9.1")

    find_package_handle_standard_args(pcl REQUIRED_VARS
                                      pcl_VERSION VERSION_VAR pcl_VERSION)
    mark_as_advanced(pcl_FOUND pcl_VERSION)


    set(pcl_INCLUDE_DIRS "/home/sitavlas/.conan/data/pcl/1.9.1/bashbug/stable/package/1dbc8a6a865a8fad699417711b0b0b490d4d2d18/include")
    set(pcl_INCLUDE_DIR "/home/sitavlas/.conan/data/pcl/1.9.1/bashbug/stable/package/1dbc8a6a865a8fad699417711b0b0b490d4d2d18/include")
    set(pcl_INCLUDES "/home/sitavlas/.conan/data/pcl/1.9.1/bashbug/stable/package/1dbc8a6a865a8fad699417711b0b0b490d4d2d18/include")
    set(pcl_RES_DIRS )
    set(pcl_DEFINITIONS )
    set(pcl_LINKER_FLAGS_LIST
            "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:>"
            "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:>"
            "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:>"
    )
    set(pcl_COMPILE_DEFINITIONS )
    set(pcl_COMPILE_OPTIONS_LIST "" "")
    set(pcl_COMPILE_OPTIONS_C "")
    set(pcl_COMPILE_OPTIONS_CXX "")
    set(pcl_LIBRARIES_TARGETS "") # Will be filled later, if CMake 3
    set(pcl_LIBRARIES "") # Will be filled later
    set(pcl_LIBS "") # Same as pcl_LIBRARIES
    set(pcl_SYSTEM_LIBS )
    set(pcl_FRAMEWORK_DIRS )
    set(pcl_FRAMEWORKS )
    set(pcl_FRAMEWORKS_FOUND "") # Will be filled later
    set(pcl_BUILD_MODULES_PATHS )

    conan_find_apple_frameworks(pcl_FRAMEWORKS_FOUND "${pcl_FRAMEWORKS}" "${pcl_FRAMEWORK_DIRS}")

    mark_as_advanced(pcl_INCLUDE_DIRS
                     pcl_INCLUDE_DIR
                     pcl_INCLUDES
                     pcl_DEFINITIONS
                     pcl_LINKER_FLAGS_LIST
                     pcl_COMPILE_DEFINITIONS
                     pcl_COMPILE_OPTIONS_LIST
                     pcl_LIBRARIES
                     pcl_LIBS
                     pcl_LIBRARIES_TARGETS)

    # Find the real .lib/.a and add them to pcl_LIBS and pcl_LIBRARY_LIST
    set(pcl_LIBRARY_LIST pcl_common pcl_features pcl_filters pcl_io pcl_io_ply pcl_kdtree pcl_keypoints pcl_ml pcl_octree pcl_recognition pcl_registration pcl_sample_consensus pcl_search pcl_segmentation pcl_stereo pcl_surface pcl_tracking)
    set(pcl_LIB_DIRS "/home/sitavlas/.conan/data/pcl/1.9.1/bashbug/stable/package/1dbc8a6a865a8fad699417711b0b0b490d4d2d18/lib")

    # Gather all the libraries that should be linked to the targets (do not touch existing variables):
    set(_pcl_DEPENDENCIES "${pcl_FRAMEWORKS_FOUND} ${pcl_SYSTEM_LIBS} boost::boost;flann::flann;Eigen3::Eigen3")

    conan_package_library_targets("${pcl_LIBRARY_LIST}"  # libraries
                                  "${pcl_LIB_DIRS}"      # package_libdir
                                  "${_pcl_DEPENDENCIES}"  # deps
                                  pcl_LIBRARIES            # out_libraries
                                  pcl_LIBRARIES_TARGETS    # out_libraries_targets
                                  ""                          # build_type
                                  "pcl")                                      # package_name

    set(pcl_LIBS ${pcl_LIBRARIES})

    foreach(_FRAMEWORK ${pcl_FRAMEWORKS_FOUND})
        list(APPEND pcl_LIBRARIES_TARGETS ${_FRAMEWORK})
        list(APPEND pcl_LIBRARIES ${_FRAMEWORK})
    endforeach()

    foreach(_SYSTEM_LIB ${pcl_SYSTEM_LIBS})
        list(APPEND pcl_LIBRARIES_TARGETS ${_SYSTEM_LIB})
        list(APPEND pcl_LIBRARIES ${_SYSTEM_LIB})
    endforeach()

    # We need to add our requirements too
    set(pcl_LIBRARIES_TARGETS "${pcl_LIBRARIES_TARGETS};boost::boost;flann::flann;Eigen3::Eigen3")
    set(pcl_LIBRARIES "${pcl_LIBRARIES};boost::boost;flann::flann;Eigen3::Eigen3")

    set(CMAKE_MODULE_PATH "/home/sitavlas/.conan/data/pcl/1.9.1/bashbug/stable/package/1dbc8a6a865a8fad699417711b0b0b490d4d2d18/" ${CMAKE_MODULE_PATH})
    set(CMAKE_PREFIX_PATH "/home/sitavlas/.conan/data/pcl/1.9.1/bashbug/stable/package/1dbc8a6a865a8fad699417711b0b0b490d4d2d18/" ${CMAKE_PREFIX_PATH})

    if(NOT ${CMAKE_VERSION} VERSION_LESS "3.0")
        # Target approach
        if(NOT TARGET pcl::pcl)
            add_library(pcl::pcl INTERFACE IMPORTED)
            if(pcl_INCLUDE_DIRS)
                set_target_properties(pcl::pcl PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                      "${pcl_INCLUDE_DIRS}")
            endif()
            set_property(TARGET pcl::pcl PROPERTY INTERFACE_LINK_LIBRARIES
                         "${pcl_LIBRARIES_TARGETS};${pcl_LINKER_FLAGS_LIST}")
            set_property(TARGET pcl::pcl PROPERTY INTERFACE_COMPILE_DEFINITIONS
                         ${pcl_COMPILE_DEFINITIONS})
            set_property(TARGET pcl::pcl PROPERTY INTERFACE_COMPILE_OPTIONS
                         "${pcl_COMPILE_OPTIONS_LIST}")

            # Library dependencies
            include(CMakeFindDependencyMacro)

            if(NOT boost_FOUND)
                find_dependency(boost REQUIRED)
            else()
                message(STATUS "Dependency boost already found")
            endif()


            if(NOT flann_FOUND)
                find_dependency(flann REQUIRED)
            else()
                message(STATUS "Dependency flann already found")
            endif()


            if(NOT Eigen3_FOUND)
                find_dependency(Eigen3 REQUIRED)
            else()
                message(STATUS "Dependency Eigen3 already found")
            endif()

        endif()
    endif()

    foreach(_BUILD_MODULE_PATH ${pcl_BUILD_MODULES_PATHS})
        include(${_BUILD_MODULE_PATH})
    endforeach()
