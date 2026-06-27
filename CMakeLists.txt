# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.16)

project(iDynTree VERSION 15.1.0
                 LANGUAGES C CXX)

# Disable in source build, unless Eclipse is used
if( ("${PROJECT_SOURCE_DIR}" STREQUAL "${PROJECT_BINARY_DIR}") AND
    (NOT ("${CMAKE_EXTRA_GENERATOR}" STREQUAL "Eclipse CDT4")) )
   message(FATAL_ERROR "In-source builds of iDynTree are not allowed. "
                       "Please remove CMakeCache.txt and the CMakeFiles/ directory, then build out-of-source. "
                       "If you really want to do an in-source build, feel free to comment out this error"
                       " or use Eclipse CDT4 Extra Generator, that is tipically used for in-source builds."
                       "Note that to use re-generate the project usign the Eclipse CDT4 Extra Generator"
                       "you need to remove CMakeCache.txt and CMakeFiles/ as well.")
endif()

set(VARS_PREFIX "${PROJECT_NAME}")

# Pick up our CMake scripts - they are all in the cmake subdirectory.
set(IDYNTREE_MODULE_DIR ${PROJECT_SOURCE_DIR}/cmake)
set(IDYNTREE_MODULE_PATH ${IDYNTREE_MODULE_DIR})
list(APPEND CMAKE_MODULE_PATH ${IDYNTREE_MODULE_PATH})

include(GNUInstallDirs)

#Use options specified in iDynTreeOptions.cmake file
include(iDynTreeOptions)

if(NOT IDYNTREE_ONLY_DOCS)
    # Find dependecies
    include(iDynTreeDependencies)

    # Add external libraries that are embedded in iDynTree
    # source tree, if necessary (by default does not adds
    # anything) feel free to check the logic inside
    add_subdirectory(extern)

    # Create the absolute paths for the BIN DIRS.
    foreach(item ${IDYNTREE_BINARY_DIRS})
        list(APPEND ABSOLUTE_BIN_DIRS "${CMAKE_INSTALL_PREFIX}/${item}")
    endforeach()

    include(AddInstallRPATHSupport)
    add_install_rpath_support(BIN_DIRS ${ABSOLUTE_BIN_DIRS}
                              LIB_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
                              DEPENDS IDYNTREE_ENABLE_RPATH
                              USE_LINK_PATH)

    # add the actual components of the library
    add_subdirectory(src)

    # List exported CMake package dependencies when the library is compiled as static
    set(_IDYNTREE_EXPORTED_DEPENDENCIES_ONLY_STATIC "")
    list(APPEND _IDYNTREE_EXPORTED_DEPENDENCIES_ONLY_STATIC LibXml2)
    if(IDYNTREE_USES_OSQPEIGEN)
        list(APPEND _IDYNTREE_EXPORTED_DEPENDENCIES_ONLY_STATIC OsqpEigen)
    endif()
    if(IDYNTREE_USES_ASSIMP)
        list(APPEND _IDYNTREE_EXPORTED_DEPENDENCIES_ONLY_STATIC assimp)
    endif()

    include(InstallBasicPackageFiles)
    install_basic_package_files(iDynTree VARS_PREFIX ${VARS_PREFIX}
                                         VERSION ${${VARS_PREFIX}_VERSION}
                                         COMPATIBILITY AnyNewerVersion
                                         TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                         NO_CHECK_REQUIRED_COMPONENTS_MACRO
                                         ENABLE_COMPATIBILITY_VARS
                                         DEPENDENCIES ${_IDYNTREE_EXPORTED_DEPENDENCIES}
                                         PRIVATE_DEPENDENCIES ${_IDYNTREE_EXPORTED_DEPENDENCIES_ONLY_STATIC})

    include(AddUninstallTarget)

    add_subdirectory(bindings)
endif()

# add a dox target to generate doxygen documentation
add_subdirectory(doc)
