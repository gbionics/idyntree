# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause


macro(add_modelio_urdf_unit_test classname)
    set(testsrc ${classname}UnitTest.cpp)
    set(testbinary ${classname}UnitTest)
    set(testname   UnitTest${classname})
    add_executable(${testbinary} ${testsrc})
    target_link_libraries(${testbinary} PRIVATE idyntree-modelio idyntree-testmodels Eigen3::Eigen)
    add_test(NAME ${testname} COMMAND ${testbinary})

    if(IDYNTREE_RUN_VALGRIND_TESTS)
        add_test(NAME memcheck_${testname} COMMAND ${MEMCHECK_COMMAND_COMPLETE} $<TARGET_FILE:${testbinary}>)

    endif()

endmacro()

add_modelio_urdf_unit_test(URDFModelImport)
add_modelio_urdf_unit_test(ModelCalibrationHelper)
add_modelio_urdf_unit_test(ModelExporter)
add_modelio_urdf_unit_test(URDFGenericSensorImport)
add_modelio_urdf_unit_test(PredictSensorsMeasurement)
add_modelio_urdf_unit_test(icubSensorURDF)

# Add idyntree-model-json round-trip test (only if nlohmann_json support is enabled)
if(IDYNTREE_USES_NLOHMANN_JSON)
    set(testsrc JSONModelRoundTripTest.cpp)
    set(testbinary JSONModelRoundTripTest)
    set(testname UnitTestJSONModelRoundTrip)
    add_executable(${testbinary} ${testsrc})
    target_link_libraries(${testbinary} PRIVATE idyntree-modelio idyntree-testmodels Eigen3::Eigen)
    target_compile_definitions(${testbinary} PRIVATE IDYNTREE_USES_NLOHMANN_JSON)
    add_test(NAME ${testname} COMMAND ${testbinary})

    if(IDYNTREE_RUN_VALGRIND_TESTS)
        add_test(NAME memcheck_${testname} COMMAND ${MEMCHECK_COMMAND_COMPLETE} $<TARGET_FILE:${testbinary}>)
    endif()
endif()

# Add SDFormat test (only if SDFormat support is enabled)
if(IDYNTREE_USES_SDFORMAT)
    set(testsrc SDFormatModelImportUnitTest.cpp)
    set(testbinary SDFormatModelImportUnitTest)
    set(testname UnitTestSDFormatModelImport)
    add_executable(${testbinary} ${testsrc})
    target_link_libraries(${testbinary} PRIVATE idyntree-modelio idyntree-testmodels Eigen3::Eigen)
    target_compile_definitions(${testbinary} PRIVATE IDYNTREE_USES_SDFORMAT)
    add_test(NAME ${testname} COMMAND ${testbinary})

    if(IDYNTREE_RUN_VALGRIND_TESTS)
        add_test(NAME memcheck_${testname} COMMAND ${MEMCHECK_COMMAND_COMPLETE} $<TARGET_FILE:${testbinary}>)
    endif()

    # Add SDFormat sensor and geometry test
    set(testsrc SDFormatGenericSensorImportUnitTest.cpp)
    set(testbinary SDFormatGenericSensorImportUnitTest)
    set(testname UnitTestSDFormatGenericSensorImport)
    add_executable(${testbinary} ${testsrc})
    target_link_libraries(${testbinary} PRIVATE idyntree-modelio idyntree-testmodels Eigen3::Eigen)
    target_compile_definitions(${testbinary} PRIVATE IDYNTREE_USES_SDFORMAT)
    add_test(NAME ${testname} COMMAND ${testbinary})

    if(IDYNTREE_RUN_VALGRIND_TESTS)
        add_test(NAME memcheck_${testname} COMMAND ${MEMCHECK_COMMAND_COMPLETE} $<TARGET_FILE:${testbinary}>)
    endif()

    # Add URDF to SDF roundtrip test
    set(testsrc URDFToSDFormatRoundtripTest.cpp)
    set(testbinary URDFToSDFormatRoundtripTest)
    set(testname UnitTestURDFToSDFormatRoundtrip)
    add_executable(${testbinary} ${testsrc})
    target_link_libraries(${testbinary} PRIVATE idyntree-modelio idyntree-testmodels idyntree-high-level Eigen3::Eigen ${sdformat_LIBRARIES})
    target_compile_definitions(${testbinary} PRIVATE IDYNTREE_USES_SDFORMAT)
    target_include_directories(${testbinary} PRIVATE ${sdformat_INCLUDE_DIRS})
    add_test(NAME ${testname} COMMAND ${testbinary})

    if(IDYNTREE_RUN_VALGRIND_TESTS)
        add_test(NAME memcheck_${testname} COMMAND ${MEMCHECK_COMMAND_COMPLETE} $<TARGET_FILE:${testbinary}>)
    endif()
endif()
