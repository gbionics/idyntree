// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "URDFDocument.h"
#include "RobotElement.h"

#include <iDynTree/FixedJoint.h>
#include <iDynTree/ModelTransformers.h>
#include <iDynTree/PrismaticJoint.h>
#include <iDynTree/RevoluteJoint.h>
#include <iDynTree/SixAxisForceTorqueSensor.h>
#include <iDynTree/SphericalJoint.h>
#include <iDynTree/Traversal.h>

#include <algorithm>
#include <unordered_set>
#include <vector>

namespace iDynTree
{

// Functions local to the current compilation unit
static std::unordered_set<std::string>
processJoints(iDynTree::Model& model,
              std::unordered_map<std::string, JointElement::JointInfo>& joints,
              std::unordered_map<std::string, JointElement::JointInfo>& fixed_joints);
static bool processSensors(Model& model, const std::vector<std::shared_ptr<SensorHelper>>& helpers);
static bool addSensorFramesAsAdditionalFramesToModel(Model& model);
static bool addVisualPropertiesToModel(
    const Model& model,
    const std::unordered_map<std::string, std::vector<VisualElement::VisualInfo>>& visuals,
    const std::unordered_map<std::string, MaterialElement::MaterialInfo>& materialDatabase,
    ModelSolidShapes& modelGeometries);

URDFDocument::URDFDocument(XMLParserState& parserState, const iDynTree::ModelParserOptions& options)
    : XMLDocument(parserState)
    , m_options(options)
{
}

iDynTree::ModelParserOptions& URDFDocument::options()
{
    return m_options;
}

const iDynTree::Model& URDFDocument::model() const
{
    return m_model;
}

const iDynTree::SensorsList& URDFDocument::sensors() const
{
    return m_model.sensors();
}

URDFDocument::~URDFDocument()
{
}
std::shared_ptr<XMLElement>
URDFDocument::rootElementForName(const std::string& name,
                                 const std::vector<std::string>& packageDirs)
{
    // allowed tag is <robot>
    if (name == "robot")
    {
        m_model = iDynTree::Model();
        m_model.setPackageDirs(packageDirs);
        m_buffers.sensorHelpers.clear();
        m_buffers.joints.clear();
        m_buffers.fixedJoints.clear();
        m_buffers.materials.clear();

        return std::make_shared<RobotElement>(getParserState(),
                                              m_model,
                                              m_buffers.sensorHelpers,
                                              m_buffers.joints,
                                              m_buffers.fixedJoints,
                                              m_buffers.materials,
                                              m_buffers.visuals,
                                              m_buffers.collisions);
    }
    // TODO: raise an error here
    return std::shared_ptr<XMLElement>(new XMLElement(getParserState()));
}

bool URDFDocument::documentHasBeenParsed()
{
    // Maybe it is not the cleanest solution to process the model here,
    // but the after the scope is exited, the Model should be fully formed.
    // Add the joints to the model
    std::unordered_set<std::string> childLinks
        = processJoints(m_model, m_buffers.joints, m_buffers.fixedJoints);

    // Get root
    std::vector<std::string> rootCandidates;
    for (unsigned link = 0; link < m_model.getNrOfLinks(); ++link)
    {
        std::string linkName = m_model.getLinkName(link);
        if (childLinks.find(linkName) == childLinks.end())
        {
            rootCandidates.push_back(linkName);
        }
    }

    if (rootCandidates.empty())
    {
        reportError("RobotElement", "exitElementScope", "No root link found in URDF string");
        m_model = Model();
        return false;
    }

    if (rootCandidates.size() >= 2)
    {
        std::stringstream ss;
        ss << "Multiple (" << rootCandidates.size() << ") root links: (";
        ss << rootCandidates[0];
        for (size_t root = 1; root < rootCandidates.size(); ++root)
        {
            ss << ", " << rootCandidates[root];
        }
        ss << ") found in URDF string";

        reportError("RobotElement", "exitElementScope", ss.str().c_str());
        m_model = Model();
        return false;
    }

    // set the default root in the model
    m_model.setDefaultBaseLink(m_model.getLinkIndex(rootCandidates[0]));
    iDynTree::Model newModel, normalizedModel;
    newModel.setPackageDirs(m_model.getPackageDirs());
    normalizedModel.setPackageDirs(m_model.getPackageDirs());

    if (!removeFakeLinks(m_model, newModel))
    {
        reportError("URDFDocument",
                    "documentHasBeenParsed",
                    "Failed to remove fake links from the model");
        return false;
    }

    std::string baseLinkName = newModel.getLinkName(newModel.getDefaultBaseLink());
    if (!createModelWithNormalizedJointNumbering(newModel, baseLinkName, normalizedModel))
    {
        reportError("URDFDocument",
                    "documentHasBeenParsed",
                    "Failed to remove fake links from the model");
        return false;
    }

    m_model = normalizedModel;

    if (!processSensors(m_model, m_buffers.sensorHelpers))
    {
        // TODO: error
        return false;
    }

    if (m_options.addSensorFramesAsAdditionalFrames)
    {
        if (!addSensorFramesAsAdditionalFramesToModel(m_model))
        {
            // TODO: error
            return false;
        }
    }

    // Assign visual properties to objects
    if (!addVisualPropertiesToModel(m_model,
                                    m_buffers.visuals,
                                    m_buffers.materials,
                                    m_model.visualSolidShapes()))
    {
        reportError("URDFDocument",
                    "documentHasBeenParsed",
                    "Failed to add visual elements to model");
    }
    if (!addVisualPropertiesToModel(m_model,
                                    m_buffers.collisions,
                                    m_buffers.materials,
                                    m_model.collisionSolidShapes()))
    {
        reportError("URDFDocument",
                    "documentHasBeenParsed",
                    "Failed to add collision elements to model");
    }

    // Convert three consecutive revolute joints to spherical joints if requested
    if (m_options.convertThreeRevoluteJointsToSphericalJoint)
    {
        Model convertedModel;
        if (!convertThreeRevoluteJointsToSphericalJoint(m_model,
                                                        convertedModel,
                                                        m_options.sphericalJointZeroMassTolerance,
                                                        m_options
                                                            .sphericalJointOrthogonalityTolerance,
                                                        m_options
                                                            .sphericalJointIntersectionTolerance))
        {
            reportError("URDFDocument",
                        "documentHasBeenParsed",
                        "Failed to convert three revolute joints to spherical joints");
            m_model = Model();
            return false;
        }
        m_model = convertedModel;
    }

    return true;
}

// MARK: - Static function definitions

std::unordered_set<std::string>
processJoints(iDynTree::Model& model,
              std::unordered_map<std::string, JointElement::JointInfo>& joints,
              std::unordered_map<std::string, JointElement::JointInfo>& fixed_joints)
{
    std::unordered_set<std::string> childLinks;

    for (auto& pair : joints)
    {
        std::string jointName = pair.first;
        std::shared_ptr<IJoint> joint = pair.second.joint;
        std::string& parentLinkName = pair.second.parentLinkName;
        std::string& childLinkName = pair.second.childLinkName;
        LinkIndex parentLinkIndex = model.getLinkIndex(parentLinkName);
        LinkIndex childLinkIndex = model.getLinkIndex(childLinkName);

        joint->setAttachedLinks(parentLinkIndex, childLinkIndex);

        // Set the axis
        // I need to check the type of Joint. Unfortunately Joint does not possesses a generic
        // setAxis method
        if (std::shared_ptr<RevoluteJoint> revolute
            = std::dynamic_pointer_cast<RevoluteJoint>(joint))
        {
            revolute->setAxis(pair.second.axis, childLinkIndex, parentLinkIndex);
        } else if (std::shared_ptr<PrismaticJoint> prismatic
                   = std::dynamic_pointer_cast<PrismaticJoint>(joint))
        {
            prismatic->setAxis(pair.second.axis, childLinkIndex, parentLinkIndex);
        } else if (std::shared_ptr<SphericalJoint> spherical
                   = std::dynamic_pointer_cast<SphericalJoint>(joint))
        {
            if (pair.second.hasJointCenter)
            {
                spherical->setJointCenter(parentLinkIndex, pair.second.jointCenterInParentFrame);
            }
        } else
        {
            reportError("URDFDocument", "processJoints", "Unrecognized joint type");
        }

        // TODO: check error
        //  It will get clone internally
        model.addJoint(jointName, joint.get());
        // save the child link name
        childLinks.insert(childLinkName);
    }

    // Adding all the fixed joint in the end
    for (auto& pair : fixed_joints)
    {
        std::string jointName = pair.first;
        std::shared_ptr<IJoint> joint = pair.second.joint;
        std::string& parentLinkName = pair.second.parentLinkName;
        std::string& childLinkName = pair.second.childLinkName;
        LinkIndex parentLinkIndex = model.getLinkIndex(parentLinkName);
        LinkIndex childLinkIndex = model.getLinkIndex(childLinkName);
        // TODO: check error
        joint->setAttachedLinks(parentLinkIndex, childLinkIndex);
        // It will get clone internally
        model.addJoint(jointName, joint.get());
        // save the child link name
        childLinks.insert(childLinkName);
    }
    return childLinks;
}

bool processSensors(Model& model, const std::vector<std::shared_ptr<SensorHelper>>& helpers)
{
    model.sensors() = iDynTree::SensorsList();
    for (auto& sensorHelper : helpers)
    {
        Sensor* sensor = sensorHelper->generateSensor(model);
        if (!sensor)
        {
            // TODO: write error
            // Clean sensor list
            model.sensors() = iDynTree::SensorsList();
            return false;
        }
        model.sensors().addSensor(*sensor);
        delete sensor;
    }
    return true;
}

bool addSensorFramesAsAdditionalFramesToModel(Model& model)
{
    bool ret = true;

    // First, we cycle on all the sensor that are attached to a link, because their frame is easy to
    // add
    // TODO : not super happy about this cycle, but is doing is work well
    for (SensorType type = SIX_AXIS_FORCE_TORQUE; type < NR_OF_SENSOR_TYPES;
         type = (SensorType)(type + 1))
    {
        // TODO : link sensor are extremly widespared and they have all approximatly the same API,
        //        so we need a better way to iterate over them
        if (isLinkSensor(type))
        {
            for (size_t sensIdx = 0; sensIdx < model.sensors().getNrOfSensors(type); ++sensIdx)
            {
                LinkSensor* linkSensor
                    = dynamic_cast<LinkSensor*>(model.sensors().getSensor(type, sensIdx));
                if (!linkSensor)
                {
                    // TODO: error
                    return false;
                }

                LinkIndex linkToWhichTheSensorIsAttached = linkSensor->getParentLinkIndex();
                std::string linkToWhichTheSensorIsAttachedName
                    = model.getLinkName(linkToWhichTheSensorIsAttached);

                if (!model.isFrameNameUsed(linkSensor->getName()))
                {
                    // std::cerr << "Adding sensor " << linkSensor->getName() << " to link " <<
                    // linkToWhichTheSensorIsAttachedName << " as additional frame"<< std::endl;
                    bool ok = model.addAdditionalFrameToLink(linkToWhichTheSensorIsAttachedName,
                                                             linkSensor->getName(),
                                                             linkSensor->getLinkSensorTransform());

                    if (!ok)
                    {
                        std::string err = "addSensorFramesAsAdditionalFrames is specified as an "
                                          "option, but it is impossible to add the frame of sensor "
                                          + linkSensor->getName() + " for unknown reasons";
                        reportError("", "addSensorFramesAsAdditionalFramesToModel", err.c_str());
                        ret = false;
                    }
                }
            }
        }

        // Explictly address the case of F/T sensors
        if (type == SIX_AXIS_FORCE_TORQUE)
        {
            // We add the sensor frame as an additional frame of the **child** link
            // (as tipically for URDF sensors the child link frame is coincident with the F/T sensor
            // frame
            for (size_t sensIdx = 0; sensIdx < model.sensors().getNrOfSensors(type); ++sensIdx)
            {
                SixAxisForceTorqueSensor* ftSensor = dynamic_cast<SixAxisForceTorqueSensor*>(
                    model.sensors().getSensor(type, sensIdx));

                std::string linkToWhichTheSensorIsAttachedName = ftSensor->getSecondLinkName();

                if (!model.isFrameNameUsed(ftSensor->getName()))
                {
                    Transform link_H_sensor;
                    bool ok = ftSensor->getLinkSensorTransform(ftSensor->getSecondLinkIndex(),
                                                               link_H_sensor);
                    ok = ok
                         && model.addAdditionalFrameToLink(linkToWhichTheSensorIsAttachedName,
                                                           ftSensor->getName(),
                                                           link_H_sensor);

                    if (!ok)
                    {
                        std::string err = "addSensorFramesAsAdditionalFrames is specified as an "
                                          "option, but it is impossible to add the frame of sensor "
                                          + ftSensor->getName() + " for unknown reasons";
                        reportError("", "addSensorFramesAsAdditionalFramesToModel", err.c_str());
                        ret = false;
                    }
                }
            }
        }
    }

    return ret;
}

bool addVisualPropertiesToModel(
    const Model& model,
    const std::unordered_map<std::string, std::vector<VisualElement::VisualInfo>>& visuals,
    const std::unordered_map<std::string, MaterialElement::MaterialInfo>& materialDatabase,
    ModelSolidShapes& modelGeometries)
{
    for (const auto& linkVisual : visuals)
    {
        // A urdf link can be either a iDynTree link or an iDynTree frame, solve
        // this ambiguity and get idyntreeLinkName and idynTreeLink_H_geometry
        bool isLink = model.isLinkNameUsed(linkVisual.first);

        for (const auto& visual : linkVisual.second)
        {
            Transform link_H_geometry;
            std::string linkName;

            // Getting associated link and tranform from link frame to the geometry frame
            if (isLink)
            {
                link_H_geometry = visual.m_origin;
                linkName = linkVisual.first;
            } else
            {
                FrameIndex urdfLinkFrameIndex = model.getFrameIndex(linkVisual.first);

                if (urdfLinkFrameIndex == FRAME_INVALID_INDEX)
                {
                    std::string message = std::string("Expecting ") + linkVisual.first
                                          + " to be a frame, but it was not found in the frame "
                                            "list";
                    reportError("URDFDocument", "addVisualPropertiesToModel", message.c_str());
                    return false;
                }

                link_H_geometry = model.getFrameTransform(urdfLinkFrameIndex) * visual.m_origin;
                linkName = model.getLinkName(model.getFrameLink(urdfLinkFrameIndex));
            }

            // Retrieving the geometry
            std::shared_ptr<SolidShape> shape = visual.m_solidShape;
            if (visual.m_material)
            {
                Material material;
                // Now check material (if exists) if it has all the information
                if (visual.m_material->m_rgba)
                {
                    material.setColor(*visual.m_material->m_rgba);
                } else
                {
                    // Look in the DB for the material with the specified name
                    auto found = materialDatabase.find(visual.m_material->m_name);
                    if (found == materialDatabase.end())
                    {
                        std::string message = std::string("Material for link ") + linkName
                                              + " has not rgba and it is not in the global "
                                                "material database.";
                        reportError("URDFDocument", "addVisualPropertiesToModel", message.c_str());
                        return false;
                    }
                    // A material can have no rgba color, for example if it
                    // only specifies a texture (not supported by iDynTree):
                    // keep the default color instead of dereferencing a
                    // null rgba. See https://github.com/gbionics/idyntree/issues/1301
                    if (found->second.m_rgba)
                    {
                        material.setColor(*found->second.m_rgba);
                    }
                }
                shape->setMaterial(material);
            }

            if (visual.m_nameAttributeFound)
            {
                shape->setName(visual.m_name);
            }
            shape->setLink_H_geometry(link_H_geometry);

            LinkIndex idyntreeLinkIndex = model.getLinkIndex(linkName);
            // ModelSolidShapes contains plain pointers, but they own the memory.
            SolidShape* solidShape = shape->clone();
            modelGeometries.getLinkSolidShapes()[idyntreeLinkIndex].push_back(solidShape);
        }
    }
    return true;
}
} // namespace iDynTree
