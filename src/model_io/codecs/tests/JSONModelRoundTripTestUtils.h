// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_JSON_MODEL_ROUND_TRIP_TEST_UTILS_H
#define IDYNTREE_JSON_MODEL_ROUND_TRIP_TEST_UTILS_H

#include <iDynTree/AccelerometerSensor.h>
#include <iDynTree/GyroscopeSensor.h>
#include <iDynTree/Model.h>
#include <iDynTree/ModelTestUtils.h>
#include <iDynTree/SixAxisForceTorqueSensor.h>
#include <iDynTree/SolidShapes.h>
#include <iDynTree/TestUtils.h>
#include <iDynTree/ThreeAxisAngularAccelerometerSensor.h>
#include <iDynTree/ThreeAxisForceTorqueContactSensor.h>

#include <vector>

namespace iDynTree
{

inline void assertJSONRoundTripTransformsAreEqual(const Transform& lhs,
                                                  const Transform& rhs,
                                                  double tol = 1e-10)
{
    ASSERT_EQUAL_MATRIX_TOL(lhs.asHomogeneousTransform(), rhs.asHomogeneousTransform(), tol);
}

inline void
assertJSONRoundTripBaseModelAreEqual(const Model& expected, const Model& actual, double tol = 1e-10)
{
    ASSERT_EQUAL_DOUBLE(expected.getNrOfLinks(), actual.getNrOfLinks());
    ASSERT_EQUAL_DOUBLE(expected.getNrOfJoints(), actual.getNrOfJoints());
    ASSERT_EQUAL_DOUBLE(expected.getNrOfDOFs(), actual.getNrOfDOFs());
    ASSERT_EQUAL_DOUBLE(expected.getNrOfPosCoords(), actual.getNrOfPosCoords());
    ASSERT_EQUAL_DOUBLE(expected.getNrOfFrames(), actual.getNrOfFrames());

    ASSERT_EQUAL_STRING(expected.getLinkName(expected.getDefaultBaseLink()),
                        actual.getLinkName(actual.getDefaultBaseLink()));

    for (LinkIndex expectedLinkIdx = 0; expectedLinkIdx < expected.getNrOfLinks();
         ++expectedLinkIdx)
    {
        const std::string& linkName = expected.getLinkName(expectedLinkIdx);
        const LinkIndex actualLinkIdx = actual.getLinkIndex(linkName);

        ASSERT_IS_TRUE(actualLinkIdx != LINK_INVALID_INDEX);
        ASSERT_EQUAL_STRING(linkName, actual.getLinkName(actualLinkIdx));
        ASSERT_EQUAL_MATRIX_TOL(expected.getLink(expectedLinkIdx)->getInertia().asMatrix(),
                                actual.getLink(actualLinkIdx)->getInertia().asMatrix(),
                                tol);
    }

    for (JointIndex expectedJointIdx = 0; expectedJointIdx < expected.getNrOfJoints();
         ++expectedJointIdx)
    {
        const std::string& jointName = expected.getJointName(expectedJointIdx);
        const JointIndex actualJointIdx = actual.getJointIndex(jointName);

        ASSERT_IS_TRUE(actualJointIdx != JOINT_INVALID_INDEX);
        ASSERT_EQUAL_STRING(jointName, actual.getJointName(actualJointIdx));

        const IJointConstPtr expectedJoint = expected.getJoint(expectedJointIdx);
        const IJointConstPtr actualJoint = actual.getJoint(actualJointIdx);

        ASSERT_EQUAL_STRING(expected.getLinkName(expectedJoint->getFirstAttachedLink()),
                            actual.getLinkName(actualJoint->getFirstAttachedLink()));
        ASSERT_EQUAL_STRING(expected.getLinkName(expectedJoint->getSecondAttachedLink()),
                            actual.getLinkName(actualJoint->getSecondAttachedLink()));
        ASSERT_EQUAL_DOUBLE(expectedJoint->getNrOfDOFs(), actualJoint->getNrOfDOFs());
        ASSERT_EQUAL_DOUBLE(expectedJoint->getNrOfPosCoords(), actualJoint->getNrOfPosCoords());

        const LinkIndex expectedLink1 = expectedJoint->getFirstAttachedLink();
        const LinkIndex expectedLink2 = expectedJoint->getSecondAttachedLink();
        const LinkIndex actualLink1 = actualJoint->getFirstAttachedLink();
        const LinkIndex actualLink2 = actualJoint->getSecondAttachedLink();

        assertJSONRoundTripTransformsAreEqual(expectedJoint->getRestTransform(expectedLink1,
                                                                              expectedLink2),
                                              actualJoint->getRestTransform(actualLink1,
                                                                            actualLink2),
                                              tol);

        ASSERT_EQUAL_DOUBLE(expectedJoint->hasPosLimits(), actualJoint->hasPosLimits());
        if (expectedJoint->hasPosLimits())
        {
            for (size_t i = 0; i < expectedJoint->getNrOfPosCoords(); ++i)
            {
                ASSERT_EQUAL_DOUBLE_TOL(expectedJoint->getMinPosLimit(i),
                                        actualJoint->getMinPosLimit(i),
                                        tol);
                ASSERT_EQUAL_DOUBLE_TOL(expectedJoint->getMaxPosLimit(i),
                                        actualJoint->getMaxPosLimit(i),
                                        tol);
            }
        }

        ASSERT_EQUAL_DOUBLE(expectedJoint->hasEffortLimits(), actualJoint->hasEffortLimits());
        if (expectedJoint->hasEffortLimits())
        {
            for (size_t i = 0; i < expectedJoint->getNrOfDOFs(); ++i)
            {
                ASSERT_EQUAL_DOUBLE_TOL(expectedJoint->getEffortLimit(i),
                                        actualJoint->getEffortLimit(i),
                                        tol);
            }
        }

        ASSERT_EQUAL_DOUBLE(expectedJoint->hasVelocityLimits(), actualJoint->hasVelocityLimits());
        if (expectedJoint->hasVelocityLimits())
        {
            for (size_t i = 0; i < expectedJoint->getNrOfDOFs(); ++i)
            {
                ASSERT_EQUAL_DOUBLE_TOL(expectedJoint->getVelocityLimit(i),
                                        actualJoint->getVelocityLimit(i),
                                        tol);
            }
        }

        ASSERT_EQUAL_DOUBLE(expectedJoint->getJointDynamicsType(),
                            actualJoint->getJointDynamicsType());
        for (size_t i = 0; i < expectedJoint->getNrOfDOFs(); ++i)
        {
            ASSERT_EQUAL_DOUBLE_TOL(expectedJoint->getDamping(i), actualJoint->getDamping(i), tol);
            ASSERT_EQUAL_DOUBLE_TOL(expectedJoint->getStaticFriction(i),
                                    actualJoint->getStaticFriction(i),
                                    tol);
        }

        const auto* expectedFixed = dynamic_cast<const FixedJoint*>(expectedJoint);
        const auto* expectedRevolute = dynamic_cast<const RevoluteJoint*>(expectedJoint);
        const auto* expectedPrismatic = dynamic_cast<const PrismaticJoint*>(expectedJoint);
        const auto* expectedRevoluteSO2 = dynamic_cast<const RevoluteSO2Joint*>(expectedJoint);
        const auto* expectedSpherical = dynamic_cast<const SphericalJoint*>(expectedJoint);

        if (expectedFixed)
        {
            ASSERT_IS_TRUE(dynamic_cast<const FixedJoint*>(actualJoint) != nullptr);
        } else if (expectedRevolute)
        {
            const auto* actualRevolute = dynamic_cast<const RevoluteJoint*>(actualJoint);
            ASSERT_IS_TRUE(actualRevolute != nullptr);

            const Axis expectedAxis = expectedRevolute->getAxis(expectedLink2, expectedLink1);
            const Axis actualAxis = actualRevolute->getAxis(actualLink2, actualLink1);

            for (int i = 0; i < 3; ++i)
            {
                ASSERT_EQUAL_DOUBLE_TOL(expectedAxis.getDirection()(i),
                                        actualAxis.getDirection()(i),
                                        tol);
                ASSERT_EQUAL_DOUBLE_TOL(expectedAxis.getOrigin()(i),
                                        actualAxis.getOrigin()(i),
                                        tol);
            }
        } else if (expectedPrismatic)
        {
            const auto* actualPrismatic = dynamic_cast<const PrismaticJoint*>(actualJoint);
            ASSERT_IS_TRUE(actualPrismatic != nullptr);

            const Axis expectedAxis = expectedPrismatic->getAxis(expectedLink2, expectedLink1);
            const Axis actualAxis = actualPrismatic->getAxis(actualLink2, actualLink1);

            for (int i = 0; i < 3; ++i)
            {
                ASSERT_EQUAL_DOUBLE_TOL(expectedAxis.getDirection()(i),
                                        actualAxis.getDirection()(i),
                                        tol);
                ASSERT_EQUAL_DOUBLE_TOL(expectedAxis.getOrigin()(i),
                                        actualAxis.getOrigin()(i),
                                        tol);
            }
        } else if (expectedRevoluteSO2)
        {
            const auto* actualRevoluteSO2 = dynamic_cast<const RevoluteSO2Joint*>(actualJoint);
            ASSERT_IS_TRUE(actualRevoluteSO2 != nullptr);

            const Axis expectedAxis = expectedRevoluteSO2->getAxis(expectedLink2, expectedLink1);
            const Axis actualAxis = actualRevoluteSO2->getAxis(actualLink2, actualLink1);

            for (int i = 0; i < 3; ++i)
            {
                ASSERT_EQUAL_DOUBLE_TOL(expectedAxis.getDirection()(i),
                                        actualAxis.getDirection()(i),
                                        tol);
                ASSERT_EQUAL_DOUBLE_TOL(expectedAxis.getOrigin()(i),
                                        actualAxis.getOrigin()(i),
                                        tol);
            }
        } else if (expectedSpherical)
        {
            const auto* actualSpherical = dynamic_cast<const SphericalJoint*>(actualJoint);
            ASSERT_IS_TRUE(actualSpherical != nullptr);

            for (int i = 0; i < 3; ++i)
            {
                ASSERT_EQUAL_DOUBLE_TOL(expectedSpherical->getJointCenter(expectedLink1)(i),
                                        actualSpherical->getJointCenter(actualLink1)(i),
                                        tol);
            }
        } else
        {
            ASSERT_IS_TRUE(false);
        }
    }

    for (FrameIndex expectedFrameIdx = expected.getNrOfLinks();
         expectedFrameIdx < expected.getNrOfFrames();
         ++expectedFrameIdx)
    {
        const std::string& frameName = expected.getFrameName(expectedFrameIdx);
        const FrameIndex actualFrameIdx = actual.getFrameIndex(frameName);

        ASSERT_IS_TRUE(actualFrameIdx != FRAME_INVALID_INDEX);
        ASSERT_EQUAL_STRING(frameName, actual.getFrameName(actualFrameIdx));
        ASSERT_EQUAL_STRING(expected.getLinkName(expected.getFrameLink(expectedFrameIdx)),
                            actual.getLinkName(actual.getFrameLink(actualFrameIdx)));
        assertJSONRoundTripTransformsAreEqual(expected.getFrameTransform(expectedFrameIdx),
                                              actual.getFrameTransform(actualFrameIdx),
                                              tol);
    }
}

inline void
assertJSONRoundTripVector3AreEqual(const Vector3& lhs, const Vector3& rhs, double tol = 1e-10)
{
    for (int i = 0; i < 3; ++i)
    {
        ASSERT_EQUAL_DOUBLE_TOL(lhs(i), rhs(i), tol);
    }
}

inline void
assertJSONRoundTripVector4AreEqual(const Vector4& lhs, const Vector4& rhs, double tol = 1e-10)
{
    for (int i = 0; i < 4; ++i)
    {
        ASSERT_EQUAL_DOUBLE_TOL(lhs(i), rhs(i), tol);
    }
}

inline void assertJSONRoundTripMaterialsAreEqual(const Material& expected,
                                                 const Material& actual,
                                                 double tol = 1e-10)
{
    ASSERT_EQUAL_STRING(expected.name(), actual.name());
    ASSERT_IS_TRUE(expected.hasColor() == actual.hasColor());
    if (expected.hasColor())
    {
        assertJSONRoundTripVector4AreEqual(expected.color(), actual.color(), tol);
    }

    ASSERT_IS_TRUE(expected.hasTexture() == actual.hasTexture());
    if (expected.hasTexture())
    {
        ASSERT_EQUAL_STRING(expected.texture(), actual.texture());
    }
}

inline void assertJSONRoundTripSolidShapesAreEqual(const SolidShape& expected,
                                                   const SolidShape& actual,
                                                   double tol = 1e-10)
{
    ASSERT_IS_TRUE(expected.isSphere() == actual.isSphere());
    ASSERT_IS_TRUE(expected.isBox() == actual.isBox());
    ASSERT_IS_TRUE(expected.isCylinder() == actual.isCylinder());
    ASSERT_IS_TRUE(expected.isExternalMesh() == actual.isExternalMesh());
    ASSERT_IS_TRUE(expected.isNameValid() == actual.isNameValid());
    if (expected.isNameValid())
    {
        ASSERT_EQUAL_STRING(expected.getName(), actual.getName());
    }

    assertJSONRoundTripTransformsAreEqual(expected.getLink_H_geometry(),
                                          actual.getLink_H_geometry(),
                                          tol);

    ASSERT_IS_TRUE(expected.isMaterialSet() == actual.isMaterialSet());
    if (expected.isMaterialSet())
    {
        assertJSONRoundTripMaterialsAreEqual(expected.getMaterial(), actual.getMaterial(), tol);
    }

    if (expected.isSphere())
    {
        ASSERT_EQUAL_DOUBLE_TOL(expected.asSphere()->getRadius(),
                                actual.asSphere()->getRadius(),
                                tol);
    } else if (expected.isBox())
    {
        ASSERT_EQUAL_DOUBLE_TOL(expected.asBox()->getX(), actual.asBox()->getX(), tol);
        ASSERT_EQUAL_DOUBLE_TOL(expected.asBox()->getY(), actual.asBox()->getY(), tol);
        ASSERT_EQUAL_DOUBLE_TOL(expected.asBox()->getZ(), actual.asBox()->getZ(), tol);
    } else if (expected.isCylinder())
    {
        ASSERT_EQUAL_DOUBLE_TOL(expected.asCylinder()->getLength(),
                                actual.asCylinder()->getLength(),
                                tol);
        ASSERT_EQUAL_DOUBLE_TOL(expected.asCylinder()->getRadius(),
                                actual.asCylinder()->getRadius(),
                                tol);
    } else if (expected.isExternalMesh())
    {
        ASSERT_EQUAL_STRING(expected.asExternalMesh()->getFilename(),
                            actual.asExternalMesh()->getFilename());
        assertJSONRoundTripVector3AreEqual(expected.asExternalMesh()->getScale(),
                                           actual.asExternalMesh()->getScale(),
                                           tol);
    }
}

inline void assertJSONRoundTripModelSolidShapesAreEqual(const Model& expectedModel,
                                                        const Model& actualModel,
                                                        const ModelSolidShapes& expectedShapes,
                                                        const ModelSolidShapes& actualShapes,
                                                        double tol = 1e-10)
{
    const auto& expectedLinkShapes = expectedShapes.getLinkSolidShapes();
    const auto& actualLinkShapes = actualShapes.getLinkSolidShapes();

    ASSERT_EQUAL_DOUBLE(expectedLinkShapes.size(), actualLinkShapes.size());

    for (LinkIndex expectedLinkIdx = 0; expectedLinkIdx < expectedModel.getNrOfLinks();
         ++expectedLinkIdx)
    {
        const std::string& linkName = expectedModel.getLinkName(expectedLinkIdx);
        const LinkIndex actualLinkIdx = actualModel.getLinkIndex(linkName);

        ASSERT_IS_TRUE(actualLinkIdx != LINK_INVALID_INDEX);
        ASSERT_EQUAL_DOUBLE(expectedLinkShapes[expectedLinkIdx].size(),
                            actualLinkShapes[actualLinkIdx].size());

        for (size_t shapeIndex = 0; shapeIndex < expectedLinkShapes[expectedLinkIdx].size();
             ++shapeIndex)
        {
            ASSERT_IS_TRUE(expectedLinkShapes[expectedLinkIdx][shapeIndex] != nullptr);
            ASSERT_IS_TRUE(actualLinkShapes[actualLinkIdx][shapeIndex] != nullptr);
            assertJSONRoundTripSolidShapesAreEqual(*expectedLinkShapes[expectedLinkIdx][shapeIndex],
                                                   *actualLinkShapes[actualLinkIdx][shapeIndex],
                                                   tol);
        }
    }
}

inline void assertJSONRoundTripSensorsAreEqual(const Model& expectedModel,
                                               const Model& actualModel,
                                               double tol = 1e-10)
{
    const SensorsList& expectedSensors = expectedModel.sensors();
    const SensorsList& actualSensors = actualModel.sensors();

    for (int sensorTypeIndex = 0; sensorTypeIndex < NR_OF_SENSOR_TYPES; ++sensorTypeIndex)
    {
        const SensorType sensorType = static_cast<SensorType>(sensorTypeIndex);
        ASSERT_EQUAL_DOUBLE(expectedSensors.getNrOfSensors(sensorType),
                            actualSensors.getNrOfSensors(sensorType));

        for (std::ptrdiff_t sensorIndex = 0;
             sensorIndex < static_cast<std::ptrdiff_t>(expectedSensors.getNrOfSensors(sensorType));
             ++sensorIndex)
        {
            const Sensor* expectedSensor = expectedSensors.getSensor(sensorType, sensorIndex);
            const Sensor* actualSensor = actualSensors.getSensor(sensorType, sensorIndex);

            ASSERT_IS_TRUE(expectedSensor != nullptr);
            ASSERT_IS_TRUE(actualSensor != nullptr);
            ASSERT_EQUAL_STRING(expectedSensor->getName(), actualSensor->getName());
            ASSERT_EQUAL_DOUBLE(expectedSensor->getSensorType(), actualSensor->getSensorType());

            if (sensorType == SIX_AXIS_FORCE_TORQUE)
            {
                const auto* expectedFT
                    = dynamic_cast<const SixAxisForceTorqueSensor*>(expectedSensor);
                const auto* actualFT = dynamic_cast<const SixAxisForceTorqueSensor*>(actualSensor);
                ASSERT_IS_TRUE(expectedFT != nullptr);
                ASSERT_IS_TRUE(actualFT != nullptr);

                ASSERT_EQUAL_STRING(expectedFT->getParentJoint(), actualFT->getParentJoint());
                ASSERT_EQUAL_STRING(expectedFT->getFirstLinkName(), actualFT->getFirstLinkName());
                ASSERT_EQUAL_STRING(expectedFT->getSecondLinkName(), actualFT->getSecondLinkName());
                ASSERT_EQUAL_STRING(expectedModel.getLinkName(expectedFT->getAppliedWrenchLink()),
                                    actualModel.getLinkName(actualFT->getAppliedWrenchLink()));

                Transform expectedFirstLink_H_sensor;
                Transform actualFirstLink_H_sensor;
                ASSERT_IS_TRUE(expectedFT->getLinkSensorTransform(expectedFT->getFirstLinkIndex(),
                                                                  expectedFirstLink_H_sensor));
                ASSERT_IS_TRUE(actualFT->getLinkSensorTransform(actualFT->getFirstLinkIndex(),
                                                                actualFirstLink_H_sensor));
                assertJSONRoundTripTransformsAreEqual(expectedFirstLink_H_sensor,
                                                      actualFirstLink_H_sensor,
                                                      tol);

                Transform expectedSecondLink_H_sensor;
                Transform actualSecondLink_H_sensor;
                ASSERT_IS_TRUE(expectedFT->getLinkSensorTransform(expectedFT->getSecondLinkIndex(),
                                                                  expectedSecondLink_H_sensor));
                ASSERT_IS_TRUE(actualFT->getLinkSensorTransform(actualFT->getSecondLinkIndex(),
                                                                actualSecondLink_H_sensor));
                assertJSONRoundTripTransformsAreEqual(expectedSecondLink_H_sensor,
                                                      actualSecondLink_H_sensor,
                                                      tol);
            } else if (sensorType == ACCELEROMETER)
            {
                const auto* expectedAccelerometer
                    = dynamic_cast<const AccelerometerSensor*>(expectedSensor);
                const auto* actualAccelerometer
                    = dynamic_cast<const AccelerometerSensor*>(actualSensor);
                ASSERT_IS_TRUE(expectedAccelerometer != nullptr);
                ASSERT_IS_TRUE(actualAccelerometer != nullptr);
                ASSERT_EQUAL_STRING(expectedAccelerometer->getParentLink(),
                                    actualAccelerometer->getParentLink());
                assertJSONRoundTripTransformsAreEqual(expectedAccelerometer
                                                          ->getLinkSensorTransform(),
                                                      actualAccelerometer->getLinkSensorTransform(),
                                                      tol);
            } else if (sensorType == GYROSCOPE)
            {
                const auto* expectedGyroscope
                    = dynamic_cast<const GyroscopeSensor*>(expectedSensor);
                const auto* actualGyroscope = dynamic_cast<const GyroscopeSensor*>(actualSensor);
                ASSERT_IS_TRUE(expectedGyroscope != nullptr);
                ASSERT_IS_TRUE(actualGyroscope != nullptr);
                ASSERT_EQUAL_STRING(expectedGyroscope->getParentLink(),
                                    actualGyroscope->getParentLink());
                assertJSONRoundTripTransformsAreEqual(expectedGyroscope->getLinkSensorTransform(),
                                                      actualGyroscope->getLinkSensorTransform(),
                                                      tol);
            } else if (sensorType == THREE_AXIS_ANGULAR_ACCELEROMETER)
            {
                const auto* expectedAngularAccelerometer
                    = dynamic_cast<const ThreeAxisAngularAccelerometerSensor*>(expectedSensor);
                const auto* actualAngularAccelerometer
                    = dynamic_cast<const ThreeAxisAngularAccelerometerSensor*>(actualSensor);
                ASSERT_IS_TRUE(expectedAngularAccelerometer != nullptr);
                ASSERT_IS_TRUE(actualAngularAccelerometer != nullptr);
                ASSERT_EQUAL_STRING(expectedAngularAccelerometer->getParentLink(),
                                    actualAngularAccelerometer->getParentLink());
                assertJSONRoundTripTransformsAreEqual(expectedAngularAccelerometer
                                                          ->getLinkSensorTransform(),
                                                      actualAngularAccelerometer
                                                          ->getLinkSensorTransform(),
                                                      tol);
            } else if (sensorType == THREE_AXIS_FORCE_TORQUE_CONTACT)
            {
                const auto* expectedContact
                    = dynamic_cast<const ThreeAxisForceTorqueContactSensor*>(expectedSensor);
                const auto* actualContact
                    = dynamic_cast<const ThreeAxisForceTorqueContactSensor*>(actualSensor);
                ASSERT_IS_TRUE(expectedContact != nullptr);
                ASSERT_IS_TRUE(actualContact != nullptr);
                ASSERT_EQUAL_STRING(expectedContact->getParentLink(),
                                    actualContact->getParentLink());
                assertJSONRoundTripTransformsAreEqual(expectedContact->getLinkSensorTransform(),
                                                      actualContact->getLinkSensorTransform(),
                                                      tol);

                const std::vector<Position> expectedLoadCellLocations
                    = expectedContact->getLoadCellLocations();
                const std::vector<Position> actualLoadCellLocations
                    = actualContact->getLoadCellLocations();
                ASSERT_EQUAL_DOUBLE(expectedLoadCellLocations.size(),
                                    actualLoadCellLocations.size());
                for (size_t loadCellIndex = 0; loadCellIndex < expectedLoadCellLocations.size();
                     ++loadCellIndex)
                {
                    for (int coordinateIndex = 0; coordinateIndex < 3; ++coordinateIndex)
                    {
                        ASSERT_EQUAL_DOUBLE_TOL(expectedLoadCellLocations[loadCellIndex](
                                                    coordinateIndex),
                                                actualLoadCellLocations[loadCellIndex](
                                                    coordinateIndex),
                                                tol);
                    }
                }
            }
        }
    }
}

inline void assertModelsAreEqualIncludingShapesAndSensors(const Model& expected,
                                                          const Model& actual,
                                                          double tol = 1e-10)
{
    assertJSONRoundTripBaseModelAreEqual(expected, actual, tol);
    assertJSONRoundTripModelSolidShapesAreEqual(expected,
                                                actual,
                                                expected.visualSolidShapes(),
                                                actual.visualSolidShapes(),
                                                tol);
    assertJSONRoundTripModelSolidShapesAreEqual(expected,
                                                actual,
                                                expected.collisionSolidShapes(),
                                                actual.collisionSolidShapes(),
                                                tol);
    assertJSONRoundTripSensorsAreEqual(expected, actual, tol);
}

} // namespace iDynTree

#endif
