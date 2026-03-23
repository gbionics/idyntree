# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause


#########################################################################
# Enable/disable dependencies
# Positional argument: CMake package name
# Options:
# MINIMUM_VERSION <version> : Minimum version required for the package
# COMPONENTS <comp1> ...    : Components that needs to be find for the dependency
# DO_NOT_SILENTLY_SEARCH    : Do not search for the package to set the default
#                             value of IDYNTREE_USES_<dep> option, but just set
#                             it to OFF
# MAIN_TARGET               : If the specified target exists, then the package is
#                             is not searched via find_package because it means that
#                             iDynTree has been added to a bigger project via add_subdirectory,
#                             and the required dependency has already been defined via add_subdirectory
macro(idyntree_handle_dependency package)
  set(options DO_NOT_SILENTLY_SEARCH NO_MODULE)
  set(singleValueArgs MINIMUM_VERSION MAIN_TARGET)
  set(multiValueArgs COMPONENTS)
  cmake_parse_arguments(IHD "${options}" "${singleValueArgs}" "${multiValueArgs}" ${ARGN})
  string(TOUPPER ${package} PKG)
  # Handle MAIN_TARGET option
  set(IHD_DEP_TARGET_IS_ALREADY_DEFINED FALSE)
  if(IHD_MAIN_TARGET)
    if(TARGET ${IHD_MAIN_TARGET})
      set(IHD_DEP_TARGET_IS_ALREADY_DEFINED TRUE)
    endif()
  endif()
  set(IHD_FIND_PACKAGE_ADDITIONAL_OPTIONS "")
  if(IHD_NO_MODULE)
    list(APPEND IHD_FIND_PACKAGE_ADDITIONAL_OPTIONS "NO_MODULE")
  endif()
  if (NOT IHD_DO_NOT_SILENTLY_SEARCH AND NOT IHD_DEP_TARGET_IS_ALREADY_DEFINED)
    if (IHD_COMPONENTS)
      find_package(${package} ${IHD_MINIMUM_VERSION} QUIET COMPONENTS ${IHD_COMPONENTS} ${IHD_FIND_PACKAGE_ADDITIONAL_OPTIONS})
    else ()
      find_package(${package} ${IHD_MINIMUM_VERSION} QUIET ${IHD_FIND_PACKAGE_ADDITIONAL_OPTIONS})
    endif ()
    set(IDYNTREE_USES_${PKG}_DEFAULT ${${package}_FOUND})
  else ()
    # If the target is defined, the dependency is enabled unless IDYNTREE_USES_${PKG} is explicitly set to FALSE
    if (IHD_DEP_TARGET_IS_ALREADY_DEFINED)
      set(IDYNTREE_USES_${PKG}_DEFAULT TRUE)
    else ()
      set(IDYNTREE_USES_${PKG}_DEFAULT FALSE)
    endif()
  endif ()
  option(IDYNTREE_USES_${PKG} "Build the part of iDynTree that depends on package ${package}" ${IDYNTREE_USES_${PKG}_DEFAULT})
  if (IDYNTREE_USES_${PKG} AND NOT IHD_DEP_TARGET_IS_ALREADY_DEFINED)
    if (IHD_COMPONENTS)
      find_package(${package} ${IHD_MINIMUM_VERSION} COMPONENTS ${IHD_COMPONENTS} REQUIRED ${IHD_FIND_PACKAGE_ADDITIONAL_OPTIONS})
    else ()
      find_package(${package} ${IHD_MINIMUM_VERSION} REQUIRED ${IHD_FIND_PACKAGE_ADDITIONAL_OPTIONS})
    endif ()
  endif ()
endmacro ()

# Eigen is required
if(NOT TARGET Eigen3::Eigen)
  find_package(Eigen3 REQUIRED CONFIG)
endif()

if(NOT TARGET LibXml2::LibXml2)
  find_package(LibXml2 REQUIRED)
endif()

idyntree_handle_dependency(YARP COMPONENTS os dev math MAIN_TARGET YARP::YARP_os)
set(YARP_REQUIRED_VERSION 3.3)
if(IDYNTREE_USES_YARP AND YARP_FOUND)
  if(${YARP_VERSION} VERSION_LESS ${YARP_REQUIRED_VERSION})
    message(FATAL_ERROR "YARP version ${YARP_VERSION} not sufficient, at least version ${YARP_REQUIRED_VERSION} is required.")
  endif()
endif()

find_package(ICUB QUIET)
option(IDYNTREE_USES_ICUB_MAIN "Build the part of iDynTree that depends on package ICUB" ${ICUB_FOUND})
if (IDYNTREE_USES_ICUB_MAIN)
  find_package(ICUB REQUIRED)
endif ()

idyntree_handle_dependency(IPOPT)
idyntree_handle_dependency(Irrlicht DO_NOT_SILENTLY_SEARCH)
idyntree_handle_dependency(MeshcatCpp MAIN_TARGET MeshcatCpp::MeshcatCpp DO_NOT_SILENTLY_SEARCH)
idyntree_handle_dependency(OsqpEigen MAIN_TARGET OsqpEigen::OsqpEigen)
idyntree_handle_dependency(ALGLIB DO_NOT_SILENTLY_SEARCH)
set(ALGLIB_REQUIRED_VERSION 3.14.0)
if(IDYNTREE_USES_ALGLIB AND ALGLIB_FOUND)
  if(NOT ${ALGLIB_VERSION} VERSION_EQUAL ${ALGLIB_REQUIRED_VERSION})
    message(FATAL_ERROR "Exactly ALGLIB version ${ALGLIB_REQUIRED_VERSION} is required, but ${ALGLIB_VERSION} is available.")
  endif()
endif()
idyntree_handle_dependency(WORHP DO_NOT_SILENTLY_SEARCH)
# Workaround for https://github.com/robotology/idyntree/issues/599
# NO_MODULE passed to avoid that the Findassimp of YCM is used instead, https://github.com/robotology/idyntree/pull/832
idyntree_handle_dependency(assimp DO_NOT_SILENTLY_SEARCH NO_MODULE MAIN_TARGET assimp::assimp)
idyntree_handle_dependency(sdformat DO_NOT_SILENTLY_SEARCH MAIN_TARGET sdformat::sdformat)
idyntree_handle_dependency(nlohmann_json)
# Workaround for https://github.com/robotology/idyntree/issues/693
if(TARGET assimp::assimp)
  get_property(assimp_INTERFACE_INCLUDE_DIRECTORIES
               TARGET assimp::assimp
               PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
  if(assimp_INTERFACE_INCLUDE_DIRECTORIES MATCHES "/usr/lib/include")
    string(REPLACE "/usr/lib/include" "/usr/include" assimp_INTERFACE_INCLUDE_DIRECTORIES "${assimp_INTERFACE_INCLUDE_DIRECTORIES}")
    set_property(TARGET assimp::assimp
                 PROPERTY INTERFACE_INCLUDE_DIRECTORIES
                 "${assimp_INTERFACE_INCLUDE_DIRECTORIES}")
    get_property(assimp_LOCATION_RELEASE
                 TARGET assimp::assimp
                 PROPERTY LOCATION_RELEASE)
    set_property(TARGET assimp::assimp
                 PROPERTY IMPORTED_LOCATION
                 "${assimp_LOCATION_RELEASE}")
  endif()
endif()
