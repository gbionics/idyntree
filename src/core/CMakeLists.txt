# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

set(IDYNTREE_CORE_EXP_HEADERS include/iDynTree/Axis.h
                              include/iDynTree/ArticulatedBodyInertia.h
                              include/iDynTree/ClassicalAcc.h
                              include/iDynTree/Direction.h
                              include/iDynTree/EigenSparseHelpers.h
                              include/iDynTree/EigenMathHelpers.h
                              include/iDynTree/EigenHelpers.h
                              include/iDynTree/InertiaNonLinearParametrization.h
                              include/iDynTree/MatrixDynSize.h
                              include/iDynTree/MatrixFixSize.h
                              include/iDynTree/Position.h
                              include/iDynTree/PositionRaw.h
                              include/iDynTree/Rotation.h
                              include/iDynTree/RotationRaw.h
                              include/iDynTree/RotationalInertia.h
                              include/iDynTree/RotationalInertiaRaw.h
                              include/iDynTree/SpatialAcc.h
                              include/iDynTree/SpatialForceVector.h
                              include/iDynTree/SpatialInertiaRaw.h
                              include/iDynTree/SpatialInertia.h
                              include/iDynTree/SpatialMomentum.h
                              include/iDynTree/SpatialMotionVector.h
                              include/iDynTree/TestUtils.h
                              include/iDynTree/Transform.h
                              include/iDynTree/TransformDerivative.h
                              include/iDynTree/Twist.h
                              include/iDynTree/Utils.h
                              include/iDynTree/VectorFixSize.h
                              include/iDynTree/VectorDynSize.h
                              include/iDynTree/Wrench.h
                              include/iDynTree/PrivateUtils.h
                              include/iDynTree/PrivatePreProcessorUtils.h
                              include/iDynTree/GeomVector3.h
                              include/iDynTree/SpatialVector.h
                              include/iDynTree/SparseMatrix.h
                              include/iDynTree/Triplets.h
                              include/iDynTree/CubicSpline.h
                              include/iDynTree/Span.h
                              include/iDynTree/SO3Utils.h
                              include/iDynTree/MatrixView.h
                              include/iDynTree/URIUtils.h)


set(IDYNTREE_CORE_EXP_SOURCES src/Axis.cpp
                              src/ArticulatedBodyInertia.cpp
                              src/ClassicalAcc.cpp
                              src/Direction.cpp
                              src/InertiaNonLinearParametrization.cpp
                              src/MatrixDynSize.cpp
                              src/GeomVector3.cpp
                              src/Position.cpp
                              src/Rotation.cpp
                              src/RotationalInertia.cpp
                              src/SpatialAcc.cpp
                              src/SpatialForceVector.cpp
                              src/SpatialMomentum.cpp
                              src/SpatialMotionVector.cpp
                              src/SpatialInertia.cpp
                              src/TestUtils.cpp
                              src/Transform.cpp
                              src/TransformDerivative.cpp
                              src/Twist.cpp
                              src/Utils.cpp
                              src/VectorDynSize.cpp
                              src/Wrench.cpp
                              src/PrivateUtils.cpp
                              src/SparseMatrix.cpp
                              src/Triplets.cpp
                              src/CubicSpline.cpp
                              src/SO3Utils.cpp
                              src/URIUtils.cpp)

SOURCE_GROUP("Source Files" FILES ${IDYNTREE_CORE_EXP_SOURCES})
SOURCE_GROUP("Header Files" FILES ${IDYNTREE_CORE_EXP_HEADERS})

# Check if this does not break existing build
# reason: avoid including with <iDynTree/**> inside .cpp but using directly
# "**" which clearly states the difference between in-library files and external files
include_directories(include/iDynTree)

set(libraryname idyntree-core)

add_library(${libraryname} ${IDYNTREE_CORE_EXP_SOURCES} ${IDYNTREE_CORE_EXP_HEADERS})
add_library(iDynTree::${libraryname} ALIAS ${libraryname})

if (DEFINED CMAKE_COMPILER_IS_GNUCXX)
  if(${CMAKE_COMPILER_IS_GNUCXX} AND ${CMAKE_CXX_COMPILER_VERSION} VERSION_LESS 5)
    target_compile_options(idyntree-core INTERFACE ${CXX11_FLAGS})
  endif()
endif()

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")
target_link_libraries(${libraryname} PRIVATE Eigen3::Eigen)

# Workaround for header-only ResolveRoboticsURICpp: extract include directories
# to avoid export set issues with old CMake versions
get_target_property(RRUC_INCLUDE_DIRS ResolveRoboticsURICpp::ResolveRoboticsURICpp INTERFACE_INCLUDE_DIRECTORIES)
if(RRUC_INCLUDE_DIRS)
    target_include_directories(${libraryname} PRIVATE ${RRUC_INCLUDE_DIRS})
endif()

# On Windows we need to correctly export global constants that are not inlined with the use of GenerateExportHeader
# vtk 6.3 installs a GenerateExportHeader CMake module that shadows the official CMake module if find_package(VTK)
# is invoked, the issue has been solved in vtk >= 7.1, see https://gitlab.kitware.com/vtk/vtk/-/merge_requests/1593 .
# To ensure that the build works even if find_package(VTK) is invoked and VTK 6 is found, we manually include exactly the
# GenerateExportHeader that is shipped with CMake, extracting the CMake install prefix from the CMAKE_COMMAND variable
# This workaround can be removed once we will not support anymore platforms that ship with vtk 6.3 as main vtk version (Ubuntu 18.04).
get_filename_component(CMAKE_UPSTREAM_MODULE_LOCATION ${CMAKE_COMMAND} PATH)
get_filename_component(CMAKE_UPSTREAM_MODULE_LOCATION ${CMAKE_UPSTREAM_MODULE_LOCATION} PATH)
set(CMAKE_UPSTREAM_MODULE_LOCATION ${CMAKE_UPSTREAM_MODULE_LOCATION}/share/cmake-${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}/Modules)
if(EXISTS ${CMAKE_UPSTREAM_MODULE_LOCATION}/GenerateExportHeader.cmake)
    include(${CMAKE_UPSTREAM_MODULE_LOCATION}/GenerateExportHeader.cmake)
else()
    include(GenerateExportHeader)
endif()
generate_export_header(${libraryname} EXPORT_FILE_NAME CoreExport.h)
list(APPEND IDYNTREE_CORE_EXP_HEADERS ${CMAKE_CURRENT_BINARY_DIR}/CoreExport.h)
target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>")

set_property(TARGET ${libraryname} PROPERTY PUBLIC_HEADER ${IDYNTREE_CORE_EXP_HEADERS})

target_compile_options(${libraryname} PRIVATE ${IDYNTREE_WARNING_FLAGS})

if(MSVC)
   add_definitions(-D_USE_MATH_DEFINES)
endif()

target_compile_features(${libraryname} PUBLIC cxx_attribute_deprecated)

install(TARGETS ${libraryname}
        EXPORT iDynTree
        COMPONENT runtime
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree
        PRIVATE_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/impl)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${libraryname})

# Install deprecated headers
install(DIRECTORY include/iDynTree/Core
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif(BUILD_TESTING)
