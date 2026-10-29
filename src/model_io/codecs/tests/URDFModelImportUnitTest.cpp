// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "testModels.h"

#include <iDynTree/TestUtils.h>

#include <iDynTree/Model.h>
#include <iDynTree/ModelExporter.h>
#include <iDynTree/ModelLoader.h>
#include <iDynTree/SphericalJoint.h>
#include <iDynTree/URDFDofsImport.h>

#include <algorithm>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <random>

using namespace iDynTree;

void checkParsingOfDofsFromURDF(std::string fileName, size_t expectedNrOfDOFs)
{
    std::vector<std::string> dofsNameList;
    bool ok = dofsListFromURDF(fileName, dofsNameList);
    ASSERT_IS_TRUE(ok);

    ASSERT_EQUAL_DOUBLE(dofsNameList.size(), expectedNrOfDOFs);
}

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

void checkURDF(std::string fileName,
               unsigned int expectedNrOfLinks,
               unsigned int expectedNrOfJoints,
               unsigned int expectedNrOfDOFs,
               unsigned int expectedNrOfFrames,
               unsigned int expectedNrOfVisuals,
               unsigned int expectedNrOfCollisions,
               std::string expectedDefaultBase)
{
    ModelLoader loader;
    bool ok = loader.loadModelFromFile(fileName);
    Model model = loader.model();
    assert(ok);

    std::cerr << "Model loaded from " << fileName << std::endl;
    std::cerr << model.toString() << std::endl;

    ASSERT_EQUAL_DOUBLE(model.getNrOfLinks(), expectedNrOfLinks);
    ASSERT_EQUAL_DOUBLE(model.getNrOfJoints(), expectedNrOfJoints);
    ASSERT_EQUAL_DOUBLE(model.getNrOfDOFs(), expectedNrOfDOFs);
    ASSERT_EQUAL_DOUBLE(model.getNrOfFrames(), expectedNrOfFrames);
    ASSERT_EQUAL_DOUBLE(getNrOfVisuals(model), expectedNrOfVisuals);
    ASSERT_EQUAL_DOUBLE(getNrOfCollisions(model), expectedNrOfCollisions);
    ASSERT_EQUAL_STRING(model.getLinkName(model.getDefaultBaseLink()), expectedDefaultBase);

    checkParsingOfDofsFromURDF(fileName, expectedNrOfDOFs);

    // Check that the copy constructor works fine
    Model modelCopyConstruced = model;

    std::cerr << "Model copy constructed from " << fileName << std::endl;
    std::cerr << modelCopyConstruced.toString() << std::endl;

    ASSERT_EQUAL_DOUBLE(modelCopyConstruced.getNrOfLinks(), expectedNrOfLinks);
    ASSERT_EQUAL_DOUBLE(modelCopyConstruced.getNrOfJoints(), expectedNrOfJoints);
    ASSERT_EQUAL_DOUBLE(modelCopyConstruced.getNrOfDOFs(), expectedNrOfDOFs);
    ASSERT_EQUAL_DOUBLE(modelCopyConstruced.getNrOfFrames(), expectedNrOfFrames);
    ASSERT_EQUAL_DOUBLE(getNrOfVisuals(modelCopyConstruced), expectedNrOfVisuals);
    ASSERT_EQUAL_DOUBLE(getNrOfCollisions(modelCopyConstruced), expectedNrOfCollisions);
    ASSERT_EQUAL_STRING(modelCopyConstruced.getLinkName(modelCopyConstruced.getDefaultBaseLink()),
                        expectedDefaultBase);

    // Check that the copy assignent works fine
    Model modelCopyAssigned;

    modelCopyAssigned = model;

    std::cerr << "Model copy assigned from " << fileName << std::endl;
    std::cerr << modelCopyAssigned.toString() << std::endl;

    ASSERT_EQUAL_DOUBLE(modelCopyAssigned.getNrOfLinks(), expectedNrOfLinks);
    ASSERT_EQUAL_DOUBLE(modelCopyAssigned.getNrOfJoints(), expectedNrOfJoints);
    ASSERT_EQUAL_DOUBLE(modelCopyAssigned.getNrOfDOFs(), expectedNrOfDOFs);
    ASSERT_EQUAL_DOUBLE(modelCopyAssigned.getNrOfFrames(), expectedNrOfFrames);
    ASSERT_EQUAL_DOUBLE(getNrOfVisuals(modelCopyAssigned), expectedNrOfVisuals);
    ASSERT_EQUAL_DOUBLE(getNrOfCollisions(modelCopyAssigned), expectedNrOfCollisions);
    ASSERT_EQUAL_STRING(modelCopyAssigned.getLinkName(modelCopyAssigned.getDefaultBaseLink()),
                        expectedDefaultBase);
}

void checkModelLoderForURDFFile(std::string urdfFile)
{
    ModelLoader loader;
    bool result = loader.loadModelFromFile(urdfFile);
    ASSERT_IS_TRUE(result && loader.isValid());
}

void checkModelLoaderFromURDFString(std::string urdfString, bool shouldBeCorrect = true)
{
    ModelLoader loader;
    bool result = loader.loadModelFromString(urdfString);
    if (shouldBeCorrect)
    {
        ASSERT_IS_TRUE(result && loader.isValid());
    } else
    {
        ASSERT_IS_TRUE(!result && !loader.isValid());
    }
}

void checkLimitsForJointsAreDefined(Model& model)
{

    for (JointIndex jnt = 0; jnt < model.getNrOfJoints(); jnt++)
    {
        IJointPtr jntPtr = model.getJoint(jnt);

        ASSERT_IS_TRUE(jntPtr != 0);

        if (jntPtr->getNrOfDOFs() == 1)
        {
            ASSERT_IS_TRUE(jntPtr->hasPosLimits());

            double max = jntPtr->getMaxPosLimit(0);
            double min = jntPtr->getMinPosLimit(0);

            ASSERT_IS_TRUE(min <= max);
            ASSERT_IS_TRUE(min > -10e7);
            ASSERT_IS_TRUE(max < 10e7);
        }
    }
}

void checkLimitsForJointsAreDefinedFromFileName(std::string urdfFileName)
{
    ModelLoader loader;
    bool ok = loader.loadModelFromFile(urdfFileName);
    Model model = loader.model();

    ASSERT_IS_TRUE(ok);

    checkLimitsForJointsAreDefined(model);

    Model copyConstructedModel = model;

    checkLimitsForJointsAreDefined(copyConstructedModel);

    Model assignedModel;
    assignedModel = model;

    checkLimitsForJointsAreDefined(assignedModel);

    // Check the reduced model loader
    std::vector<std::string> dofsOfModel;
    ok = dofsListFromURDF(urdfFileName, dofsOfModel);
    ASSERT_IS_TRUE(ok);

    ok = loader.loadReducedModelFromFile(urdfFileName, dofsOfModel);

    ASSERT_IS_TRUE(ok);

    Model reducedModel = loader.model();

    checkLimitsForJointsAreDefined(reducedModel);
}

void checkLoadReducedModelOrderIsKept(std::string urdfFileName)
{
    ModelLoader loader;
    ASSERT_IS_TRUE(loader.loadModelFromFile(urdfFileName) && loader.isValid());

    Model loadedModel = loader.model();
    // get all joints in order
    std::vector<std::string> dofsName;
    dofsName.resize(loadedModel.getNrOfDOFs());

    for (JointIndex index = 0; index < loadedModel.getNrOfJoints(); ++index)
    {
        IJointPtr joint = loadedModel.getJoint(index);
        std::string jointName = loadedModel.getJointName(index);

        for (unsigned dof = 0; dof < joint->getNrOfDOFs(); ++dof)
        {
            dofsName[joint->getDOFsOffset() + dof] = jointName;
        }
    }

    std::mt19937 random_gen(0);
    std::shuffle(dofsName.begin(), dofsName.end(), random_gen);

    // now load the new model and check they are the same
    ASSERT_IS_TRUE(loader.loadReducedModelFromFullModel(loadedModel, dofsName) && loader.isValid());

    Model shuffledModel = loader.model();
    for (JointIndex index = 0; index < shuffledModel.getNrOfJoints(); ++index)
    {
        IJointPtr joint = shuffledModel.getJoint(index);
        std::string jointName = shuffledModel.getJointName(index);
        for (unsigned dof = 0; dof < joint->getNrOfDOFs(); ++dof)
        {
            ASSERT_IS_TRUE(dofsName[joint->getDOFsOffset() + dof] == jointName);
        }
    }
}

void checkDuplicateJointsReturnsError()
{
    std::string urdf = R"(
<robot name="robot">
  <link name="link_1">
    <inertial>
      <mass value="1"/>
    </inertial>
  </link>
  <link name="link_2">
    <inertial>
      <mass value="2"/>
    </inertial>
  </link>
  <link name="link_3">
    <inertial>
      <mass value="3"/>
    </inertial>
  </link>
  <joint name="joint_1" type="revolute">
    <origin xyz="0.0 0.0 0.0"/>
    <axis xyz="0 0 1"/>
    <parent link="link_1"/>
    <child link="link_2"/>
    <limit lower="-1.0" upper="1.0"/>
  </joint>
  <joint name="joint_1" type="revolute">
    <origin xyz="0.0 0.0 0.0"/>
    <axis xyz="0 0 1"/>
    <parent link="link_1"/>
    <child link="link_3"/>
    <limit lower="-2.0" upper="2.0"/>
  </joint>
</robot>
)";

    ModelLoader loader;
    ASSERT_IS_FALSE(loader.loadModelFromString(urdf));
}

void checkaddSensorFramesAsAdditionalFramesOption()
{
    std::string urdf = R"(
  <robot name="robot">
  <link name="link_1">
    <inertial>
      <mass value="1"/>
    </inertial>
  </link>
  <link name="link_2">
    <inertial>
      <mass value="2"/>
    </inertial>
  </link>
  <joint name="joint_1" type="fixed">
    <origin xyz="0.0 0.0 0.0"/>
    <axis xyz="0 0 1"/>
    <parent link="link_1"/>
    <child link="link_2"/>
    <limit lower="-1.0" upper="1.0"/>
  </joint>
  <sensor name="l_leg_ft" type="force_torque">
    <parent joint="joint_1"/>
    <force_torque>
      <frame>sensor</frame>
      <measure_direction>child_to_parent</measure_direction>
    </force_torque>
    <origin rpy="2.220446049250313e-16 -2.220446049250314e-16 2.220446049250313e-16" xyz="0.0 -1.3877787807814457e-17 0.0"/>
  </sensor>
</robot>
)";

    // Let's first load with the option disabled
    {
        iDynTree::ModelLoader mdlLoader;
        iDynTree::ModelParserOptions parserOptions;
        parserOptions.addSensorFramesAsAdditionalFrames = false;
        mdlLoader.setParsingOptions(parserOptions);
        ASSERT_IS_TRUE(mdlLoader.loadModelFromString(urdf));
        ASSERT_IS_FALSE(mdlLoader.model().isFrameNameUsed("l_leg_ft"));
    }

    // Then with the option enabled
    {
        iDynTree::ModelLoader mdlLoader;
        iDynTree::ModelParserOptions parserOptions;
        parserOptions.addSensorFramesAsAdditionalFrames = true;
        mdlLoader.setParsingOptions(parserOptions);
        ASSERT_IS_TRUE(mdlLoader.loadModelFromString(urdf));
        ASSERT_IS_TRUE(mdlLoader.model().isFrameNameUsed("l_leg_ft"));
    }
}

void checkSphericalJointImport()
{
    auto loadAndValidate = [](const std::string& jointType) {
        std::string urdf = std::string("<robot name=\"spherical_robot\">\n"
                                       "  <link name=\"base\">\n"
                                       "    <inertial>\n"
                                       "      <mass value=\"1\"/>\n"
                                       "    </inertial>\n"
                                       "  </link>\n"
                                       "  <link name=\"tip\">\n"
                                       "    <inertial>\n"
                                       "      <mass value=\"1\"/>\n"
                                       "    </inertial>\n"
                                       "  </link>\n"
                                       "  <joint name=\"ball_joint\" type=\"")
                           + jointType
                           + std::string("\">\n"
                                         "    <origin xyz=\"0.1 0.2 0.3\" rpy=\"0 0 0\"/>\n"
                                         "    <parent link=\"base\"/>\n"
                                         "    <child link=\"tip\"/>\n"
                                         "  </joint>\n"
                                         "</robot>\n");

        ModelLoader loader;
        ASSERT_IS_TRUE(loader.loadModelFromString(urdf));

        const Model& model = loader.model();
        ASSERT_EQUAL_DOUBLE(model.getNrOfJoints(), 1);
        ASSERT_EQUAL_DOUBLE(model.getNrOfDOFs(), 3);
        ASSERT_EQUAL_STRING(model.getLinkName(model.getDefaultBaseLink()), "base");

        JointIndex jointIdx = model.getJointIndex("ball_joint");
        const IJoint* joint = model.getJoint(jointIdx);
        const auto* spherical = dynamic_cast<const SphericalJoint*>(joint);
        ASSERT_IS_TRUE(spherical != nullptr);

        LinkIndex parentIdx = model.getLinkIndex("base");
        LinkIndex childIdx = model.getLinkIndex("tip");

        Position centerInParent = spherical->getJointCenter(parentIdx);
        Position centerInChild = spherical->getJointCenter(childIdx);

        ASSERT_EQUAL_DOUBLE(centerInParent(0), 0.1);
        ASSERT_EQUAL_DOUBLE(centerInParent(1), 0.2);
        ASSERT_EQUAL_DOUBLE(centerInParent(2), 0.3);
        ASSERT_EQUAL_DOUBLE(centerInChild(0), 0.0);
        ASSERT_EQUAL_DOUBLE(centerInChild(1), 0.0);
        ASSERT_EQUAL_DOUBLE(centerInChild(2), 0.0);
    };

    loadAndValidate("spherical");
    loadAndValidate("ball");
}

void checkEffortAndVelocityLimits()
{
    // Test URDF with effort and velocity limits
    std::string urdfString = R"(
<robot name="test_limits">
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <link name="link1">
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="revolute_joint" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100.0" velocity="10.0"/>
  </joint>

  <link name="link2">
    <inertial>
      <mass value="0.3"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="prismatic_joint" type="prismatic">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0.5 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0.0" upper="0.5" effort="200.0" velocity="5.0"/>
  </joint>
</robot>
)";

    ModelLoader loader;
    bool ok = loader.loadModelFromString(urdfString);
    ASSERT_IS_TRUE(ok);

    Model model = loader.model();

    // Check revolute joint
    JointIndex revoluteIdx = model.getJointIndex("revolute_joint");
    ASSERT_IS_TRUE(revoluteIdx != JOINT_INVALID_INDEX);

    IJointConstPtr revoluteJoint = model.getJoint(revoluteIdx);
    ASSERT_IS_TRUE(revoluteJoint != nullptr);
    ASSERT_EQUAL_DOUBLE(revoluteJoint->getNrOfDOFs(), 1);

    // Check effort limits
    ASSERT_IS_TRUE(revoluteJoint->hasEffortLimits());
    ASSERT_EQUAL_DOUBLE(revoluteJoint->getEffortLimit(0), 100.0);

    // Check velocity limits
    ASSERT_IS_TRUE(revoluteJoint->hasVelocityLimits());
    ASSERT_EQUAL_DOUBLE(revoluteJoint->getVelocityLimit(0), 10.0);

    // Check prismatic joint
    JointIndex prismaticIdx = model.getJointIndex("prismatic_joint");
    ASSERT_IS_TRUE(prismaticIdx != JOINT_INVALID_INDEX);

    IJointConstPtr prismaticJoint = model.getJoint(prismaticIdx);
    ASSERT_IS_TRUE(prismaticJoint != nullptr);
    ASSERT_EQUAL_DOUBLE(prismaticJoint->getNrOfDOFs(), 1);

    // Check effort limits
    ASSERT_IS_TRUE(prismaticJoint->hasEffortLimits());
    ASSERT_EQUAL_DOUBLE(prismaticJoint->getEffortLimit(0), 200.0);

    // Check velocity limits
    ASSERT_IS_TRUE(prismaticJoint->hasVelocityLimits());
    ASSERT_EQUAL_DOUBLE(prismaticJoint->getVelocityLimit(0), 5.0);

    std::cerr << "URDF effort and velocity limits test passed" << std::endl;
}

void checkEffortAndVelocityLimitsRoundtrip()
{
    // Test that limits survive URDF export/import roundtrip
    std::string urdfString = R"(<?xml version="1.0"?>
<robot name="test_roundtrip">
  <link name="base">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <link name="link1">
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="joint1" type="revolute">
    <parent link="base"/>
    <child link="link1"/>
    <origin xyz="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="50.0" velocity="2.5"/>
  </joint>
</robot>
)";

    // Load original model
    ModelLoader loader1;
    bool ok = loader1.loadModelFromString(urdfString);
    ASSERT_IS_TRUE(ok);
    Model originalModel = loader1.model();

    // Export to URDF string
    std::string exportedUrdf;
    ModelExporter exporter;
    ok = exporter.init(originalModel);
    ASSERT_IS_TRUE(ok);
    ok = exporter.exportModelToString(exportedUrdf);
    ASSERT_IS_TRUE(ok);

    std::cerr << "Exported URDF:\n" << exportedUrdf << std::endl;

    // Re-import from exported URDF
    ModelLoader loader2;
    ok = loader2.loadModelFromString(exportedUrdf);
    ASSERT_IS_TRUE(ok);
    Model reimportedModel = loader2.model();

    // Check that limits are preserved
    JointIndex jointIdx = reimportedModel.getJointIndex("joint1");
    ASSERT_IS_TRUE(jointIdx != JOINT_INVALID_INDEX);

    IJointConstPtr joint = reimportedModel.getJoint(jointIdx);
    ASSERT_IS_TRUE(joint != nullptr);

    ASSERT_IS_TRUE(joint->hasEffortLimits());
    ASSERT_EQUAL_DOUBLE(joint->getEffortLimit(0), 50.0);

    ASSERT_IS_TRUE(joint->hasVelocityLimits());
    ASSERT_EQUAL_DOUBLE(joint->getVelocityLimit(0), 2.5);

    std::cerr << "URDF effort and velocity limits roundtrip test passed" << std::endl;
}


void checkTextureOnlyMaterialImport()
{
    // Regression test for https://github.com/gbionics/idyntree/issues/1301 :
    // a visual referencing a global material that has a texture but no color
    // used to segfault instead of being loaded with the default color.
    std::string urdf = R"(<?xml version="1.0"?>
<robot name="texture_only_material">
  <material name="TexMat">
    <texture filename="package://foo/bar.png"/>
  </material>
  <link name="base_link">
    <visual>
      <geometry><box size="1 1 1"/></geometry>
      <material name="TexMat"/>
    </visual>
  </link>
</robot>)";

    ModelLoader loader;
    bool result = loader.loadModelFromString(urdf);
    ASSERT_IS_TRUE(result && loader.isValid());
    ASSERT_IS_TRUE(getNrOfVisuals(loader.model()) == 1);
}

int main()
{
    checkTextureOnlyMaterialImport();
    checkURDF(getAbsModelPath("/simple_model.urdf"), 1, 0, 0, 1, 1, 0, "link1");
    checkURDF(getAbsModelPath("/oneLink.urdf"), 1, 0, 0, 7, 1, 1, "link1");
    checkURDF(getAbsModelPath("twoLinks.urdf"), 2, 1, 1, 6, 0, 0, "link1");
    checkURDF(getAbsModelPath("icub_skin_frames.urdf"), 39, 38, 32, 62, 28, 28, "root_link");
    checkURDF(getAbsModelPath("iCubGenova02.urdf"), 33, 32, 26, 111, 33, 33, "root_link");
    checkURDF(getAbsModelPath("icalibrate.urdf"), 6, 5, 3, 7, 6, 6, "base");

    checkModelLoderForURDFFile(getAbsModelPath("/oneLink.urdf"));
    checkModelLoaderFromURDFString("this is not an xml", false);

    checkLimitsForJointsAreDefinedFromFileName(getAbsModelPath("iCubGenova02.urdf"));

    checkLoadReducedModelOrderIsKept(getAbsModelPath("iCubGenova02.urdf"));
    checkaddSensorFramesAsAdditionalFramesOption();
    checkSphericalJointImport();

    checkEffortAndVelocityLimits();
    checkEffortAndVelocityLimitsRoundtrip();

    return EXIT_SUCCESS;
}
