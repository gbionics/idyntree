# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

set(IDYNTREE_MODELIO_HEADERS include/iDynTree/URDFDofsImport.h
                                  include/iDynTree/ModelLoader.h
                                  include/iDynTree/ModelExporter.h
                                  include/iDynTree/ModelCalibrationHelper.h)

set(IDYNTREE_MODELIO_PRIVATE_HEADERS include/private/URDFDocument.h
                                          include/private/SDFormatDocument.h
                                          include/private/ModelIOFormatUtils.h
                                          include/private/InertialElement.h
                                          include/private/JointElement.h
                                          include/private/LinkElement.h
                                          include/private/RobotElement.h
                                          include/private/OriginElement.h
                                          include/private/SensorElement.h
                                          include/private/ForceTorqueSensorElement.h
                                          include/private/MaterialElement.h
                                          include/private/VisualElement.h
                                          include/private/GeometryElement.h
                                          include/private/URDFParsingUtils.h
                                          include/private/URDFModelExport.h)

set(IDYNTREE_MODELIO_URDF_XMLELEMENTS_SOURCES src/URDFDocument.cpp
                                              src/SDFormatDocument.cpp
                                              src/InertialElement.cpp
                                              src/JointElement.cpp
                                              src/LinkElement.cpp
                                              src/RobotElement.cpp
                                              src/OriginElement.cpp
                                              src/SensorElement.cpp
                                              src/ForceTorqueSensorElement.cpp
                                              src/MaterialElement.cpp
                                              src/VisualElement.cpp
                                              src/GeometryElement.cpp)

set(IDYNTREE_MODELIO_SOURCES src/URDFDofsImport.cpp
                                  src/ModelLoader.cpp
                                  src/ModelExporter.cpp
                                  src/ModelCalibrationHelper.cpp
                                  src/URDFModelExport.cpp)

if(IDYNTREE_USES_NLOHMANN_JSON)
    list(APPEND IDYNTREE_MODELIO_SOURCES src/ModelJSONImportExport.cpp)
    list(APPEND IDYNTREE_MODELIO_PRIVATE_HEADERS include/private/ModelJSONImportExport.h)
endif()

list(APPEND IDYNTREE_MODELIO_SOURCES ${IDYNTREE_MODELIO_URDF_XMLELEMENTS_SOURCES})

set(libraryname idyntree-modelio)

add_library(${libraryname} ${IDYNTREE_MODELIO_SOURCES} ${IDYNTREE_MODELIO_HEADERS} ${IDYNTREE_MODELIO_PRIVATE_HEADERS} $<TARGET_OBJECTS:idyntree-private-fpconv>)
add_library(iDynTree::${libraryname} ALIAS ${libraryname})

target_compile_features(${libraryname} PRIVATE cxx_auto_type cxx_delegating_constructors cxx_final cxx_lambdas cxx_lambda_init_captures)

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                 "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/private>"
                                                 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")

target_link_libraries(${libraryname} PUBLIC idyntree-core idyntree-model idyntree-modelio-xml
                                     PRIVATE Eigen3::Eigen LibXml2::LibXml2)

# Add SDFormat support if available
if(IDYNTREE_USES_SDFORMAT)
    target_link_libraries(${libraryname} PRIVATE sdformat::sdformat)
    target_compile_definitions(${libraryname} PRIVATE IDYNTREE_USES_SDFORMAT)
endif()

# Add nlohmann_json support if available
if(IDYNTREE_USES_NLOHMANN_JSON)
    target_link_libraries(${libraryname} PRIVATE nlohmann_json::nlohmann_json)
    target_compile_definitions(${libraryname} PRIVATE IDYNTREE_USES_NLOHMANN_JSON)
endif()

# See https://stackoverflow.com/questions/38832528/transitive-target-include-directories-on-object-libraries
# Can be removed with CMake 3.12
target_include_directories(${libraryname} PRIVATE $<TARGET_PROPERTY:idyntree-private-fpconv,INTERFACE_INCLUDE_DIRECTORIES>)

target_compile_options(${libraryname} PRIVATE ${IDYNTREE_WARNING_FLAGS})

set_property(TARGET ${libraryname} PROPERTY PUBLIC_HEADER ${IDYNTREE_MODELIO_HEADERS})

install(TARGETS ${libraryname}
        EXPORT iDynTree
        COMPONENT runtime
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${libraryname})

if(BUILD_TESTING)
    add_subdirectory(tests)
endif(BUILD_TESTING)


# Install deprecated headers
install(DIRECTORY include/iDynTree/ModelIO
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree)