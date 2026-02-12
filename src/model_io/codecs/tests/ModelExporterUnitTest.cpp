// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "testModels.h"

#include <iDynTree/TestUtils.h>

#include <iDynTree/Model.h>
#include <iDynTree/ModelExporter.h>
#include <iDynTree/ModelLoader.h>
#include <iDynTree/ModelTransformers.h>
#include <iDynTree/RevoluteJoint.h>
#include <iDynTree/SphericalJoint.h>

#include <algorithm>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>

using namespace iDynTree;

unsigned int getNrOfVisuals(const iDynTree::Model& model)
{
    unsigned int nrOfVisuals = 0;
    for (LinkIndex index = 0; index < model.getNrOfLinks(); ++index)
    {
        nrOfVisuals += model.visualSolidShapes().getLinkSolidShapes()[index].size();
    }
    return nrOfVisuals;
}

unsigned int getNrOfCollisions(const iDynTree::Model& model)
{
    unsigned int nrOfCollisions = 0;
    for (LinkIndex index = 0; index < model.getNrOfLinks(); ++index)
    {
        nrOfCollisions += model.collisionSolidShapes().getLinkSolidShapes()[index].size();
    }
    return nrOfCollisions;
}

void checkSolidAreEqual(SolidShape* solid, SolidShape* solidCheck)
{
    ASSERT_IS_TRUE(solid->isBox() == solidCheck->isBox());
    ASSERT_IS_TRUE(solid->isCylinder() == solidCheck->isCylinder());
    ASSERT_IS_TRUE(solid->isSphere() == solidCheck->isSphere());
    ASSERT_IS_TRUE(solid->isExternalMesh() == solidCheck->isExternalMesh());

    ASSERT_IS_TRUE(solid->getName() == solidCheck->getName());
    ASSERT_IS_TRUE(solid->isNameValid() == solidCheck->isNameValid());

    ASSERT_EQUAL_TRANSFORM(solid->getLink_H_geometry(), solidCheck->getLink_H_geometry());

    if (solid->isBox())
    {
        const Box* box = solid->asBox();
        const Box* boxCheck = solidCheck->asBox();
        ASSERT_EQUAL_DOUBLE(box->getX(), boxCheck->getX());
        ASSERT_EQUAL_DOUBLE(box->getY(), boxCheck->getY());
        ASSERT_EQUAL_DOUBLE(box->getZ(), boxCheck->getZ());

    } else if (solid->isCylinder())
    {
        const Cylinder* cylinder = solid->asCylinder();
        const Cylinder* cylinderCheck = solidCheck->asCylinder();
        ASSERT_EQUAL_DOUBLE(cylinder->getRadius(), cylinderCheck->getRadius());
        ASSERT_EQUAL_DOUBLE(cylinder->getLength(), cylinderCheck->getLength());

    } else if (solid->isSphere())
    {
        const Sphere* sphere = solid->asSphere();
        const Sphere* sphereCheck = solidCheck->asSphere();
        ASSERT_EQUAL_DOUBLE(sphere->getRadius(), sphereCheck->getRadius());

    } else if (solid->isExternalMesh())
    {
        const ExternalMesh* mesh = solid->asExternalMesh();
        const ExternalMesh* meshCheck = solidCheck->asExternalMesh();
        ASSERT_EQUAL_VECTOR(mesh->getScale(), meshCheck->getScale());
        ASSERT_IS_TRUE(mesh->getFilename() == meshCheck->getFilename());
    } else
    {
        ASSERT_IS_TRUE(false);
    }
}

void checkImportExportURDF(std::string fileName)
{
    // Import, export and re-import a URDF file, and check that the key properties of the model are
    // mantained
    ModelLoader mdlLoader;
    bool ok = mdlLoader.loadModelFromFile(fileName, "urdf");
    Model model = mdlLoader.model();
    ASSERT_IS_TRUE(ok);

    std::cerr << "Model loaded from " << fileName << std::endl;
    std::string urdfString;
    ModelExporter mdlExporter;
    ok = mdlExporter.init(model, SensorsList());
    ASSERT_IS_TRUE(ok);
    ok = mdlExporter.exportModelToString(urdfString);
    ASSERT_IS_TRUE(ok);

    std::cerr << "Model serialized back to xml " << std::endl << urdfString << std::endl;

    ModelLoader mdlLoaderReloaded;
    ok = mdlLoaderReloaded.loadModelFromString(urdfString);
    ASSERT_IS_TRUE(ok);
    Model modelReloaded = mdlLoaderReloaded.model();

    ASSERT_EQUAL_DOUBLE(model.getNrOfLinks(), modelReloaded.getNrOfLinks());
    ASSERT_EQUAL_DOUBLE(model.getNrOfJoints(), modelReloaded.getNrOfJoints());
    ASSERT_EQUAL_DOUBLE(model.getNrOfDOFs(), modelReloaded.getNrOfDOFs());
    ASSERT_EQUAL_DOUBLE(model.getNrOfFrames(), modelReloaded.getNrOfFrames());
    ASSERT_EQUAL_DOUBLE(getNrOfVisuals(model), getNrOfVisuals(modelReloaded));
    ASSERT_EQUAL_DOUBLE(getNrOfCollisions(model), getNrOfCollisions(modelReloaded));

    // Verify that the link correspond (note that the serialization could have changed)
    for (int lnkIndex = 0; lnkIndex < model.getNrOfLinks(); lnkIndex++)
    {
        LinkIndex lnkIndexInReloaded = modelReloaded.getLinkIndex(model.getLinkName(lnkIndex));
        ASSERT_IS_TRUE(lnkIndexInReloaded != LINK_INVALID_INDEX);
        ASSERT_IS_TRUE(model.getLinkName(lnkIndex)
                       == modelReloaded.getLinkName(lnkIndexInReloaded));
        SpatialInertia inertia = model.getLink(lnkIndex)->getInertia();
        SpatialInertia inertiaReloaded = modelReloaded.getLink(lnkIndexInReloaded)->getInertia();
        std::cerr << "Testing inertia of link " << model.getLinkName(lnkIndex) << std::endl;
        ASSERT_EQUAL_MATRIX(inertia.asMatrix(), inertiaReloaded.asMatrix());

        // For each link, verify that the visual and collision shape correspond

        // First verify that the number of visual elements are the same
        ASSERT_EQUAL_DOUBLE(model.visualSolidShapes().getLinkSolidShapes()[lnkIndex].size(),
                            modelReloaded.visualSolidShapes()
                                .getLinkSolidShapes()[lnkIndexInReloaded]
                                .size());

        // Then, if there is only one element, verify that it matches
        if (model.visualSolidShapes().getLinkSolidShapes()[lnkIndex].size() == 1)
        {
            SolidShape* solidInModel = model.visualSolidShapes().getLinkSolidShapes()[lnkIndex][0];
            SolidShape* solidInModelReloaded
                = modelReloaded.visualSolidShapes().getLinkSolidShapes()[lnkIndexInReloaded][0];
            std::cerr << "original: " << solidInModel->getLink_H_geometry().toString() << std::endl
                      << "reloaded: " << solidInModelReloaded->getLink_H_geometry().toString()
                      << std::endl;
            checkSolidAreEqual(solidInModel, solidInModelReloaded);
        }

        // First verify that the number of visual elements are the same
        ASSERT_EQUAL_DOUBLE(model.collisionSolidShapes().getLinkSolidShapes()[lnkIndex].size(),
                            modelReloaded.collisionSolidShapes()
                                .getLinkSolidShapes()[lnkIndexInReloaded]
                                .size());

        // Then, if there is only one element, verify that it matches
        if (model.collisionSolidShapes().getLinkSolidShapes()[lnkIndex].size() == 1)
        {
            SolidShape* solidInModel
                = model.collisionSolidShapes().getLinkSolidShapes()[lnkIndex][0];
            SolidShape* solidInModelReloaded
                = modelReloaded.collisionSolidShapes().getLinkSolidShapes()[lnkIndexInReloaded][0];
            checkSolidAreEqual(solidInModel, solidInModelReloaded);
        }
    }

    // Verify that the frame correspond (note that the serialization could have changed)
    for (FrameIndex frameIndex = model.getNrOfLinks(); frameIndex < model.getNrOfFrames();
         frameIndex++)
    {
        FrameIndex frameIndexInReloaded
            = modelReloaded.getFrameIndex(model.getFrameName(frameIndex));
        ASSERT_IS_TRUE(frameIndexInReloaded != FRAME_INVALID_INDEX);
        ASSERT_IS_TRUE(model.getFrameName(frameIndex)
                       == modelReloaded.getFrameName(frameIndexInReloaded));
        Transform link_H_frame = model.getFrameTransform(frameIndex);
        Transform link_H_frame_reloaded = modelReloaded.getFrameTransform(frameIndexInReloaded);
        ASSERT_EQUAL_MATRIX(link_H_frame.asHomogeneousTransform(),
                            link_H_frame_reloaded.asHomogeneousTransform());
    }
}

void testFramesNotInTraversal()
{
    // Create a model with two different roots and export only one.
    Model model;
    SpatialInertia zeroInertia;
    zeroInertia.zero();

    Link link;
    link.setInertia(zeroInertia);

    // Create a 2 links - one joint model.
    model.addLink("link_1_1", link);
    model.addLink("link_1_2", link);

    auto joint = std::make_unique<RevoluteJoint>();
    model.addJoint("link_1_1", "link_1_2", "j1", joint.get());

    // Add a frame to the body.
    model.addAdditionalFrameToLink("link_1_2", "f1", Transform::Identity());

    // Add also another link not connected to the other body.
    model.addLink("link_2_1", link);

    // Add a frame to this body too.
    model.addAdditionalFrameToLink("link_2_1", "f2", Transform::Identity());

    // Export the model. We want to export only the first body.
    ModelExporter exporter;
    // Set the root link to link_1_1.
    ModelExporterOptions options;
    options.baseLink = "link_1_1";
    exporter.setExportingOptions(options);

    bool ok = exporter.init(model, SensorsList());
    ASSERT_IS_TRUE(ok);
    std::string urdfString;
    ok = exporter.exportModelToString(urdfString);
    ASSERT_IS_TRUE(ok);

    // Reload the model.
    ModelLoader loader;

    ok = loader.loadModelFromString(urdfString);
    ASSERT_IS_TRUE(ok);
    Model modelReloaded = loader.model();

    // Check that the reloaded model contains only the 2 links - 1 joint model.
    ASSERT_EQUAL_DOUBLE(modelReloaded.getNrOfLinks(), 2);
    ASSERT_EQUAL_DOUBLE(modelReloaded.getNrOfJoints(), 1);
    ASSERT_EQUAL_DOUBLE(modelReloaded.getNrOfDOFs(), 1);
    ASSERT_EQUAL_DOUBLE(modelReloaded.getNrOfFrames(), 2 + 1);
}

void testJointAxisWithNonZeroOriginButPassingThroughChildLinkFrameOrigin()
{
    // iDynTree has no constraint of where the joint axis passes w.r.t. to
    // the link frames of the link it connects, while URDF constraints the
    // joint axis to pass through the origin of the child link frame

    // This test checks that a model with a non-zero offset origin but that
    // passes through the child link origin is correctly exported
    Model model;
    SpatialInertia zeroInertia;
    zeroInertia.zero();

    Link link;
    link.setInertia(zeroInertia);

    // Create a 2 links - one joint model.
    std::string link1Name = "link_1";
    std::string link2Name = "link_2";
    std::string jointName = "j1";

    model.addLink(link1Name, link);
    model.addLink(link2Name, link);

    auto joint = std::make_unique<RevoluteJoint>();

    // Add a offset in the x direction between link_1 and link_2
    iDynTree::Transform link1_X_link2 = iDynTree::Transform::Identity();
    link1_X_link2.setPosition(iDynTree::Position(1.0, 0.0, 0.0));
    joint->setRestTransform(link1_X_link2);

    // Set the attached links first before setting the axis
    joint->setAttachedLinks(model.getLinkIndex(link1Name), model.getLinkIndex(link2Name));

    // Mark the rotation of the joint along z, and add a offset of the axis along z
    iDynTree::Axis axis;
    axis.setDirection(iDynTree::Direction(0.0, 0.0, 1.0));
    axis.setOrigin(iDynTree::Position(0.0, 0.0, 1.0));
    joint->setAxis(axis, model.getLinkIndex(link2Name));

    model.addJoint(link1Name, link2Name, jointName, joint.get());

    ModelExporter exporter;
    bool ok = exporter.init(model);
    ASSERT_IS_TRUE(ok);
    std::string urdfString;
    ok = exporter.exportModelToString(urdfString);
    ASSERT_IS_TRUE(ok);
    // Reload the model.
    ModelLoader loader;

    ok = loader.loadModelFromString(urdfString);
    ASSERT_IS_TRUE(ok);

    // Reload model
    Model modelReloaded = loader.model();

    // Check that the reloaded have the same transform of the original model
    iDynTree::VectorDynSize jointPos;
    jointPos.resize(model.getNrOfPosCoords());
    for (size_t i = 0; i++; i < 10)
    {
        // Arbitrary joint angles
        jointPos(0) = 0.1 * i - 0.5;
        iDynTree::Transform link1_H_link2_orig = model.getJoint(model.getJointIndex(jointName))
                                                     ->getTransform(jointPos,
                                                                    model.getLinkIndex(link1Name),
                                                                    model.getLinkIndex(link2Name));
        iDynTree::Transform link1_H_link2_reloaded
            = modelReloaded.getJoint(modelReloaded.getJointIndex(jointName))
                  ->getTransform(jointPos,
                                 modelReloaded.getLinkIndex(link1Name),
                                 modelReloaded.getLinkIndex(link2Name));

        ASSERT_EQUAL_TRANSFORM(link1_H_link2_orig, link1_H_link2_reloaded);
    }

    // Create a new model with an axis that does not pass through the origin of the child link and
    // verify that it can't be loaded
    Model modelThatCantBeExportedToUrdf;
    modelThatCantBeExportedToUrdf.addLink(link1Name, link);
    modelThatCantBeExportedToUrdf.addLink(link2Name, link);
    auto jointNew = std::make_unique<RevoluteJoint>();

    // Add a offset in the x direction between link_1 and link_2
    iDynTree::Transform link1_X_link2New = iDynTree::Transform::Identity();
    link1_X_link2New.setPosition(iDynTree::Position(1.0, 0.0, 0.0));
    jointNew->setRestTransform(link1_X_link2New);

    // Set the attached links first before setting the axis
    jointNew->setAttachedLinks(modelThatCantBeExportedToUrdf.getLinkIndex(link1Name),
                               modelThatCantBeExportedToUrdf.getLinkIndex(link2Name));

    // Mark the rotation of the joint along z, and add a offset of the axis along y
    iDynTree::Axis axisNew;
    axisNew.setDirection(iDynTree::Direction(0.0, 0.0, 1.0));
    axisNew.setOrigin(iDynTree::Position(0.0, 1.0, 0.0));
    jointNew->setAxis(axisNew, modelThatCantBeExportedToUrdf.getLinkIndex(link2Name));

    modelThatCantBeExportedToUrdf.addJoint(link1Name, link2Name, jointName, jointNew.get());

    ModelExporter exporterNew;
    ok = exporterNew.init(modelThatCantBeExportedToUrdf);
    ASSERT_IS_TRUE(ok);
    ok = exporterNew.exportModelToString(urdfString);
    ASSERT_IS_FALSE(ok);
}

void testSphericalJointExportToURDF()
{
    // Create a model with spherical joints and verify URDF export converts them to three revolute
    // joints
    Model originalModel;

    // Create links with proper inertia
    auto baseLinkInertia = getRandomInertia();
    auto link1Inertia = getRandomInertia();
    auto link2Inertia = getRandomInertia();
    auto endEffectorLinkInertia = getRandomInertia();

    Link baseLink, link1, link2, endEffectorLink;
    baseLink.setInertia(baseLinkInertia);
    link1.setInertia(link1Inertia);
    link2.setInertia(link2Inertia);
    endEffectorLink.setInertia(endEffectorLinkInertia);

    // Add links to model
    auto baseLinkIndex = originalModel.addLink("base", baseLink);
    originalModel.setDefaultBaseLink(baseLinkIndex);
    auto link1Index = originalModel.addLink("link1", link1);
    auto link2Index = originalModel.addLink("link2", link2);
    auto endEffectorLinkIndex = originalModel.addLink("end_effector", endEffectorLink);

    // Create first spherical joint (base -> link1)
    Transform baseToLink1 = getRandomTransform();
    SphericalJoint sphericalJoint1;
    sphericalJoint1.setAttachedLinks(baseLinkIndex, link1Index);
    sphericalJoint1.setRestTransform(baseToLink1);
    Position jointCenter1(baseToLink1.getPosition()); // Joint center at the origin of link1 frame
    sphericalJoint1.setJointCenter(baseLinkIndex, jointCenter1);

    originalModel.addJoint("spherical_joint_1", &sphericalJoint1);

    // Add a revolute joint between link1 and link2 to test mixed joint types
    Transform link1ToLink2 = getRandomTransform();
    Axis revoluteAxis(getRandomRotation() * Direction(1, 0, 0), link1ToLink2.getPosition());
    RevoluteJoint revoluteJoint;
    revoluteJoint.setAttachedLinks(link1Index, link2Index);
    revoluteJoint.setRestTransform(link1ToLink2);
    revoluteJoint.setAxis(revoluteAxis);

    originalModel.addJoint("revolute_joint", &revoluteJoint);

    // Create second spherical joint (link2 -> end_effector)
    Transform link2ToEndEffector = getRandomTransform();
    SphericalJoint sphericalJoint2;
    sphericalJoint2.setAttachedLinks(link2Index, endEffectorLinkIndex);
    sphericalJoint2.setRestTransform(link2ToEndEffector);
    Position jointCenter2(link2ToEndEffector.getPosition()); // Joint center at the origin of
                                                             // end_effector frame
    sphericalJoint2.setJointCenter(link2Index, jointCenter2);

    originalModel.addJoint("spherical_joint_2", &sphericalJoint2);

    // Export to URDF with spherical joint conversion enabled (default)
    ModelExporter exporter;
    bool ok = exporter.init(originalModel, SensorsList());
    ModelExporterOptions options;
    options.exportSphericalJointsAsThreeRevoluteJoints = true;
    options.sphericalJointFakeLinkPrefix = "fake_";
    options.sphericalJointRevoluteJointPrefix = "rev_";
    exporter.setExportingOptions(options);

    ASSERT_IS_TRUE(ok);
    std::cerr << "Original model:" << std::endl;
    std::cerr << originalModel.toString() << std::endl;

    std::string urdfString;
    ok = exporter.exportModelToString(urdfString);
    ASSERT_IS_TRUE(ok);

    std::cerr << "URDF with converted spherical joints:" << std::endl << urdfString << std::endl;

    // Load the exported URDF back
    ModelLoader loader;
    ModelParserOptions parserOptions;
    parserOptions.convertThreeRevoluteJointsToSphericalJoint = false; // Disable conversion back to
                                                                      // spherical joints
    loader.setParsingOptions(parserOptions);
    ok = loader.loadModelFromString(urdfString);
    ASSERT_IS_TRUE(ok);

    Model exportedModel = loader.model();

    // Verify the exported model structure
    // Original: 4 links, 3 joints (2 spherical + 1 revolute)
    // Expected after conversion: 8 links (4 original + 4 fake), 7 joints (6 revolute from spherical
    // conversion + 1 original revolute)
    ASSERT_EQUAL_DOUBLE(exportedModel.getNrOfLinks(), 8);
    ASSERT_EQUAL_DOUBLE(exportedModel.getNrOfJoints(), 7);
    ASSERT_EQUAL_DOUBLE(exportedModel.getNrOfDOFs(), 7); // 6 DOFs from 2 spherical joints + 1 DOF
                                                         // from revolute joint

    // Check that original links still exist
    ASSERT_IS_TRUE(exportedModel.isLinkNameUsed("base"));
    ASSERT_IS_TRUE(exportedModel.isLinkNameUsed("link1"));
    ASSERT_IS_TRUE(exportedModel.isLinkNameUsed("link2"));
    ASSERT_IS_TRUE(exportedModel.isLinkNameUsed("end_effector"));

    std::cerr << "Exported model:" << std::endl;
    std::cerr << exportedModel.toString() << std::endl;

    // Check that fake links were created for each spherical joint
    ASSERT_IS_TRUE(exportedModel.isLinkNameUsed("fake_spherical_joint_1_link1"));
    ASSERT_IS_TRUE(exportedModel.isLinkNameUsed("fake_spherical_joint_1_link2"));
    ASSERT_IS_TRUE(exportedModel.isLinkNameUsed("fake_spherical_joint_2_link1"));
    ASSERT_IS_TRUE(exportedModel.isLinkNameUsed("fake_spherical_joint_2_link2"));

    // Verify fake links have zero mass
    LinkIndex fakeLinkIdx = exportedModel.getLinkIndex("fake_spherical_joint_1_link1");
    ASSERT_IS_TRUE(exportedModel.getLink(fakeLinkIdx)->getInertia().getMass() < 1e-10);

    fakeLinkIdx = exportedModel.getLinkIndex("fake_spherical_joint_1_link2");
    ASSERT_IS_TRUE(exportedModel.getLink(fakeLinkIdx)->getInertia().getMass() < 1e-10);

    fakeLinkIdx = exportedModel.getLinkIndex("fake_spherical_joint_2_link1");
    ASSERT_IS_TRUE(exportedModel.getLink(fakeLinkIdx)->getInertia().getMass() < 1e-10);

    fakeLinkIdx = exportedModel.getLinkIndex("fake_spherical_joint_2_link2");
    ASSERT_IS_TRUE(exportedModel.getLink(fakeLinkIdx)->getInertia().getMass() < 1e-10);

    // Check that revolute joints were created for each spherical joint (3 per spherical joint)
    // First spherical joint conversion
    ASSERT_IS_TRUE(exportedModel.isJointNameUsed("rev_spherical_joint_1_x"));
    ASSERT_IS_TRUE(exportedModel.isJointNameUsed("rev_spherical_joint_1_y"));
    ASSERT_IS_TRUE(exportedModel.isJointNameUsed("rev_spherical_joint_1_z"));

    // Second spherical joint conversion
    ASSERT_IS_TRUE(exportedModel.isJointNameUsed("rev_spherical_joint_2_x"));
    ASSERT_IS_TRUE(exportedModel.isJointNameUsed("rev_spherical_joint_2_y"));
    ASSERT_IS_TRUE(exportedModel.isJointNameUsed("rev_spherical_joint_2_z"));

    // Original revolute joint should still exist
    ASSERT_IS_TRUE(exportedModel.isJointNameUsed("revolute_joint"));

    // Verify that all joints are revolute (URDF doesn't support spherical joints)
    for (JointIndex jointIdx = 0; jointIdx < exportedModel.getNrOfJoints(); jointIdx++)
    {
        const IJoint* joint = exportedModel.getJoint(jointIdx);
        ASSERT_IS_TRUE(dynamic_cast<const RevoluteJoint*>(joint) != nullptr);
    }

    // Check original spherical joints no longer exist
    ASSERT_IS_TRUE(!exportedModel.isJointNameUsed("spherical_joint_1"));
    ASSERT_IS_TRUE(!exportedModel.isJointNameUsed("spherical_joint_2"));

    // Verify joint connectivity for first spherical joint conversion
    // Chain should be: base -> fake_spherical_joint_1_link1 -> fake_spherical_joint_1_link2 ->
    // link1
    JointIndex xJoint1Idx = exportedModel.getJointIndex("rev_spherical_joint_1_x");
    JointIndex yJoint1Idx = exportedModel.getJointIndex("rev_spherical_joint_1_y");
    JointIndex zJoint1Idx = exportedModel.getJointIndex("rev_spherical_joint_1_z");

    const IJoint* xJoint1 = exportedModel.getJoint(xJoint1Idx);
    const IJoint* yJoint1 = exportedModel.getJoint(yJoint1Idx);
    const IJoint* zJoint1 = exportedModel.getJoint(zJoint1Idx);

    ASSERT_IS_TRUE(xJoint1->getFirstAttachedLink() == exportedModel.getLinkIndex("base"));
    ASSERT_IS_TRUE(xJoint1->getSecondAttachedLink()
                   == exportedModel.getLinkIndex("fake_spherical_joint_1_link1"));

    ASSERT_IS_TRUE(yJoint1->getFirstAttachedLink()
                   == exportedModel.getLinkIndex("fake_spherical_joint_1_link1"));
    ASSERT_IS_TRUE(yJoint1->getSecondAttachedLink()
                   == exportedModel.getLinkIndex("fake_spherical_joint_1_link2"));

    ASSERT_IS_TRUE(zJoint1->getFirstAttachedLink()
                   == exportedModel.getLinkIndex("fake_spherical_joint_1_link2"));
    ASSERT_IS_TRUE(zJoint1->getSecondAttachedLink() == exportedModel.getLinkIndex("link1"));

    std::cerr << "[TEST] testSphericalJointExportToURDF passed" << std::endl;
}

void testSphericalJointExportDisabled()
{
    // Test that spherical joint export can be disabled
    Model originalModel;

    // Create a simple model with one spherical joint
    SpatialInertia linkInertia;

    auto baseLinkInertia = getRandomInertia();
    auto childLinkInertia = getRandomInertia();

    Link baseLink, childLink;
    baseLink.setInertia(baseLinkInertia);
    childLink.setInertia(childLinkInertia);

    originalModel.addLink("base", baseLink);
    originalModel.addLink("child", childLink);

    Transform baseToChild = getRandomTransform();
    SphericalJoint sphericalJoint(baseToChild);
    Position jointCenter = getRandomPosition();
    LinkIndex baseLinkIdx = originalModel.getLinkIndex("base");
    sphericalJoint.setJointCenter(baseLinkIdx, jointCenter);

    originalModel.addJoint("base", "child", "spherical_joint", &sphericalJoint);
    originalModel.setDefaultBaseLink(baseLinkIdx);

    // Export with spherical joint conversion disabled

    ModelExporter exporter;
    bool ok = exporter.init(originalModel, SensorsList());
    ModelExporterOptions options;
    options.exportSphericalJointsAsThreeRevoluteJoints = false;
    exporter.setExportingOptions(options);

    ASSERT_IS_TRUE(ok);

    std::cerr << "Original model with spherical joint:" << std::endl;
    std::cerr << originalModel.toString() << std::endl;

    std::string urdfString;
    ok = exporter.exportModelToString(urdfString);

    // Export should fail because URDF doesn't natively support spherical joints
    // and conversion is disabled
    ASSERT_IS_FALSE(ok);

    std::cerr << "[TEST] testSphericalJointExportDisabled passed" << std::endl;
}

void testNumericalPrecision()
{
    // Test numerical precision options in model export

    Model model;

    // Create a model with precise inertia values
    double mass = 1.23456789012345;
    Position com(1.23456789012345, 2.34567890123456, 3.45678901234567);
    RotationalInertia rotInertia;
    rotInertia.zero();
    rotInertia(0, 0) = 0.123456789;
    rotInertia(1, 1) = 0.234567890;
    rotInertia(2, 2) = 0.345678901;
    SpatialInertia inertia1(mass, com, rotInertia);
    SpatialInertia inertia2 = getRandomInertia();

    Link link1, link2;
    link1.setInertia(inertia1);
    link2.setInertia(inertia2);

    model.addLink("link1", link1);
    model.addLink("link2", link2);

    // Create a joint with specific transform values to test precision on transforms
    Transform link1_H_link2 = Transform::Identity();
    link1_H_link2.setPosition(Position(1.23456789012345, 2.34567890123456, 3.45678901234567));

    auto joint = std::make_unique<RevoluteJoint>();
    joint->setAttachedLinks(model.getLinkIndex("link1"), model.getLinkIndex("link2"));
    joint->setRestTransform(link1_H_link2);

    Axis axis;
    axis.setDirection(Direction(0.0, 0.0, 1.0));
    axis.setOrigin(Position(0.0, 0.0, 0.0));
    joint->setAxis(axis, model.getLinkIndex("link2"));

    model.addJoint("link1", "link2", "joint1", joint.get());

    // Add an additional frame to test precision on frame transforms
    Transform link1_H_frame = Transform::Identity();
    link1_H_frame.setPosition(Position(4.56789012345678, 5.67890123456789, 6.78901234567890));
    model.addAdditionalFrameToLink("link1", "test_frame", link1_H_frame);

    // Check that the default option (0) uses maximum precision
    {
        ModelExporter exporter;
        ModelExporterOptions options;
        ASSERT_EQUAL_DOUBLE(options.numericalPrecision, 0);

        bool ok = exporter.init(model, options);
        ASSERT_IS_TRUE(ok);

        std::string urdfString;
        ok = exporter.exportModelToString(urdfString);
        ASSERT_IS_TRUE(ok);

        std::cerr << "URDF with default (maximum) precision:" << std::endl
                  << urdfString << std::endl;

        // Verify that the output contains high precision values
        ASSERT_IS_TRUE(urdfString.find("1.234567890") != std::string::npos);
    }

    // Check that a custom option (6) affects all numerical values
    {
        ModelExporter exporter;
        ModelExporterOptions options;
        options.numericalPrecision = 6;

        bool ok = exporter.init(model, options);
        ASSERT_IS_TRUE(ok);

        std::string urdfString;
        ok = exporter.exportModelToString(urdfString);
        ASSERT_IS_TRUE(ok);

        std::cerr << "URDF with precision=6:" << std::endl << urdfString << std::endl;

        // Reload and verify values are reasonably close
        ModelLoader loader;
        ok = loader.loadModelFromString(urdfString);
        ASSERT_IS_TRUE(ok);

        Model reloadedModel = loader.model();
        LinkIndex linkIdx = reloadedModel.getLinkIndex("link1");
        ASSERT_IS_TRUE(linkIdx != LINK_INVALID_INDEX);

        SpatialInertia reloadedInertia = reloadedModel.getLink(linkIdx)->getInertia();

        // With precision=6 values should be close but not exact
        double massTolerance = 1e-5;
        ASSERT_EQUAL_DOUBLE_TOL(inertia1.getMass(), reloadedInertia.getMass(), massTolerance);

        std::cerr << "Original mass: " << inertia1.getMass()
                  << "Reloaded mass: " << reloadedInertia.getMass() << std::endl;
    }

    std::cerr << "[TEST] testNumericalPrecision passed" << std::endl;
}

// Update the main function to include the new tests:
int main()
{
    for (unsigned int mdl = 0; mdl < IDYNTREE_TESTS_URDFS_NR; mdl++)
    {
        if (std::string(IDYNTREE_TESTS_URDFS[mdl]) == "bigman.urdf")
        {
            // walkman model fails this test due to
            // https://github.com/robotology/idyntree/issues/247
            continue;
        }

        std::string urdfFileName = getAbsModelPath(std::string(IDYNTREE_TESTS_URDFS[mdl]));
        std::cout << "Checking model import/export test on " << urdfFileName << std::endl;
        checkImportExportURDF(urdfFileName);
    }

    testFramesNotInTraversal();
    testJointAxisWithNonZeroOriginButPassingThroughChildLinkFrameOrigin();

    // Add the new spherical joint export tests
    testSphericalJointExportToURDF();
    testSphericalJointExportDisabled();

    // Add the numerical precision test
    testNumericalPrecision();

    return EXIT_SUCCESS;
}
