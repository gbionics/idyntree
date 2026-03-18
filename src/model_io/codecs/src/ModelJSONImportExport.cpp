// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "ModelJSONImportExport.h"

#include <iDynTree/AccelerometerSensor.h>
#include <iDynTree/Direction.h>
#include <iDynTree/FixedJoint.h>
#include <iDynTree/GyroscopeSensor.h>
#include <iDynTree/Model.h>
#include <iDynTree/PrismaticJoint.h>
#include <iDynTree/RevoluteJoint.h>
#include <iDynTree/RevoluteSO2Joint.h>
#include <iDynTree/SixAxisForceTorqueSensor.h>
#include <iDynTree/SolidShapes.h>
#include <iDynTree/SpatialInertia.h>
#include <iDynTree/SphericalJoint.h>
#include <iDynTree/ThreeAxisAngularAccelerometerSensor.h>
#include <iDynTree/ThreeAxisForceTorqueContactSensor.h>
#include <iDynTree/Transform.h>
#include <iDynTree/Utils.h>

#include <nlohmann/json.hpp>

#include <fstream>
#include <sstream>
#include <string>

namespace iDynTree
{

// ─── serialization helpers ───────────────────────────────────────────────────

static nlohmann::json positionToJSON(const Position& pos)
{
    return nlohmann::json::array({pos(0), pos(1), pos(2)});
}

static Position positionFromJSON(const nlohmann::json& j)
{
    return Position(j.at(0).get<double>(), j.at(1).get<double>(), j.at(2).get<double>());
}

static nlohmann::json directionToJSON(const Direction& dir)
{
    return nlohmann::json::array({dir(0), dir(1), dir(2)});
}

static Direction directionFromJSON(const nlohmann::json& j)
{
    return Direction(j.at(0).get<double>(), j.at(1).get<double>(), j.at(2).get<double>());
}

static nlohmann::json rotation3x3ToJSON(const Rotation& rot)
{
    nlohmann::json m = nlohmann::json::array();
    for (int r = 0; r < 3; r++)
    {
        nlohmann::json row = nlohmann::json::array();
        for (int c = 0; c < 3; c++)
            row.push_back(rot(r, c));
        m.push_back(row);
    }
    return m;
}

static Rotation rotation3x3FromJSON(const nlohmann::json& j)
{
    Rotation rot;
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            rot(r, c) = j.at(r).at(c).get<double>();
    return rot;
}

static nlohmann::json transformToJSON(const Transform& T)
{
    return {{"position", positionToJSON(T.getPosition())},
            {"rotation_matrix", rotation3x3ToJSON(T.getRotation())}};
}

static Transform transformFromJSON(const nlohmann::json& j)
{
    return Transform(rotation3x3FromJSON(j.at("rotation_matrix")),
                     positionFromJSON(j.at("position")));
}

static nlohmann::json axisToJSON(const Axis& ax)
{
    return {{"direction", directionToJSON(ax.getDirection())},
            {"origin", positionToJSON(ax.getOrigin())}};
}

static Axis axisFromJSON(const nlohmann::json& j)
{
    return Axis(directionFromJSON(j.at("direction")), positionFromJSON(j.at("origin")));
}

static nlohmann::json vector3ToJSON(const Vector3& vec)
{
    return nlohmann::json::array({vec(0), vec(1), vec(2)});
}

static Vector3 vector3FromJSON(const nlohmann::json& j)
{
    Vector3 vec;
    for (int i = 0; i < 3; ++i)
    {
        vec(i) = j.at(i).get<double>();
    }
    return vec;
}

static nlohmann::json vector4ToJSON(const Vector4& vec)
{
    return nlohmann::json::array({vec(0), vec(1), vec(2), vec(3)});
}

static Vector4 vector4FromJSON(const nlohmann::json& j)
{
    Vector4 vec;
    for (int i = 0; i < 4; ++i)
    {
        vec(i) = j.at(i).get<double>();
    }
    return vec;
}

static nlohmann::json materialToJSON(const Material& material)
{
    nlohmann::json materialJSON;
    materialJSON["name"] = material.name();
    materialJSON["has_color"] = material.hasColor();
    if (material.hasColor())
    {
        materialJSON["color"] = vector4ToJSON(material.color());
    }
    materialJSON["has_texture"] = material.hasTexture();
    if (material.hasTexture())
    {
        materialJSON["texture"] = material.texture();
    }
    return materialJSON;
}

static Material materialFromJSON(const nlohmann::json& j)
{
    Material material(j.at("name").get<std::string>());
    if (j.at("has_color").get<bool>())
    {
        material.setColor(vector4FromJSON(j.at("color")));
    }
    if (j.at("has_texture").get<bool>())
    {
        material.setTexture(j.at("texture").get<std::string>());
    }
    return material;
}

static nlohmann::json solidShapeToJSON(const SolidShape& shape)
{
    nlohmann::json shapeJSON;
    shapeJSON["has_name"] = shape.isNameValid();
    if (shape.isNameValid())
    {
        shapeJSON["name"] = shape.getName();
    }
    shapeJSON["link_H_geometry"] = transformToJSON(shape.getLink_H_geometry());
    shapeJSON["has_material"] = shape.isMaterialSet();
    if (shape.isMaterialSet())
    {
        shapeJSON["material"] = materialToJSON(shape.getMaterial());
    }

    if (shape.isSphere())
    {
        shapeJSON["type"] = "sphere";
        shapeJSON["radius"] = shape.asSphere()->getRadius();
    }
    else if (shape.isBox())
    {
        shapeJSON["type"] = "box";
        shapeJSON["size"]
            = nlohmann::json::array({shape.asBox()->getX(), shape.asBox()->getY(), shape.asBox()->getZ()});
    }
    else if (shape.isCylinder())
    {
        shapeJSON["type"] = "cylinder";
        shapeJSON["length"] = shape.asCylinder()->getLength();
        shapeJSON["radius"] = shape.asCylinder()->getRadius();
    }
    else if (shape.isExternalMesh())
    {
        shapeJSON["type"] = "external_mesh";
        shapeJSON["filename"] = shape.asExternalMesh()->getFilename();
        shapeJSON["scale"] = vector3ToJSON(shape.asExternalMesh()->getScale());
    }

    return shapeJSON;
}

static bool addSolidShapeToModel(const nlohmann::json& shapeJSON,
                                 ModelSolidShapes& modelShapes,
                                 LinkIndex linkIndex)
{
    const std::string type = shapeJSON.at("type").get<std::string>();

    SolidShape* shape = nullptr;
    if (type == "sphere")
    {
        Sphere sphere;
        sphere.setRadius(shapeJSON.at("radius").get<double>());
        shape = sphere.clone();
    }
    else if (type == "box")
    {
        Box box;
        box.setX(shapeJSON.at("size").at(0).get<double>());
        box.setY(shapeJSON.at("size").at(1).get<double>());
        box.setZ(shapeJSON.at("size").at(2).get<double>());
        shape = box.clone();
    }
    else if (type == "cylinder")
    {
        Cylinder cylinder;
        cylinder.setLength(shapeJSON.at("length").get<double>());
        cylinder.setRadius(shapeJSON.at("radius").get<double>());
        shape = cylinder.clone();
    }
    else if (type == "external_mesh")
    {
        ExternalMesh mesh;
        mesh.setFilename(shapeJSON.at("filename").get<std::string>());
        mesh.setScale(vector3FromJSON(shapeJSON.at("scale")));
        shape = mesh.clone();
    }
    else
    {
        reportError("ModelJSONExport", "addSolidShapeToModel", "Unknown solid shape type.");
        return false;
    }

    shape->setLink_H_geometry(transformFromJSON(shapeJSON.at("link_H_geometry")));
    if (shapeJSON.at("has_name").get<bool>())
    {
        shape->setName(shapeJSON.at("name").get<std::string>());
    }
    if (shapeJSON.at("has_material").get<bool>())
    {
        shape->setMaterial(materialFromJSON(shapeJSON.at("material")));
    }

    modelShapes.addSingleLinkSolidShape(linkIndex, *shape);
    delete shape;
    return true;
}

static nlohmann::json modelSolidShapesToJSON(const Model& model, const ModelSolidShapes& modelShapes)
{
    nlohmann::json linkShapesJSON = nlohmann::json::array();
    const auto& allLinkShapes = modelShapes.getLinkSolidShapes();

    for (LinkIndex linkIndex = 0; linkIndex < model.getNrOfLinks(); ++linkIndex)
    {
        nlohmann::json linkJSON;
        linkJSON["link_name"] = model.getLinkName(linkIndex);
        linkJSON["shapes"] = nlohmann::json::array();

        for (const SolidShape* shape : allLinkShapes[linkIndex])
        {
            if (shape == nullptr)
            {
                reportError("ModelJSONExport",
                            "modelSolidShapesToJSON",
                            "Encountered null solid shape pointer.");
                return nlohmann::json();
            }
            linkJSON["shapes"].push_back(solidShapeToJSON(*shape));
        }

        linkShapesJSON.push_back(linkJSON);
    }

    return linkShapesJSON;
}

static bool modelSolidShapesFromJSON(const nlohmann::json& linkShapesJSON,
                                     Model& model,
                                     ModelSolidShapes& modelShapes)
{
    for (const auto& linkJSON : linkShapesJSON)
    {
        const std::string linkName = linkJSON.at("link_name").get<std::string>();
        const LinkIndex linkIndex = model.getLinkIndex(linkName);
        if (linkIndex == LINK_INVALID_INDEX)
        {
            reportError("ModelJSONExport",
                        "modelSolidShapesFromJSON",
                        ("Unknown link for solid shapes: " + linkName).c_str());
            return false;
        }

        for (const auto& shapeJSON : linkJSON.at("shapes"))
        {
            if (!addSolidShapeToModel(shapeJSON, modelShapes, linkIndex))
            {
                return false;
            }
        }
    }

    return true;
}

static nlohmann::json sensorToJSON(const Sensor& sensor, const Model& model)
{
    nlohmann::json sensorJSON;
    sensorJSON["name"] = sensor.getName();

    switch (sensor.getSensorType())
    {
    case SIX_AXIS_FORCE_TORQUE:
    {
        const auto* ftSensor = dynamic_cast<const SixAxisForceTorqueSensor*>(&sensor);
        sensorJSON["type"] = "six_axis_force_torque";
        sensorJSON["parent_joint"] = ftSensor->getParentJoint();
        sensorJSON["first_link"] = ftSensor->getFirstLinkName();
        sensorJSON["second_link"] = ftSensor->getSecondLinkName();
        sensorJSON["applied_wrench_link"] = model.getLinkName(ftSensor->getAppliedWrenchLink());

        Transform firstLink_H_sensor;
        Transform secondLink_H_sensor;
        ftSensor->getLinkSensorTransform(ftSensor->getFirstLinkIndex(), firstLink_H_sensor);
        ftSensor->getLinkSensorTransform(ftSensor->getSecondLinkIndex(), secondLink_H_sensor);
        sensorJSON["first_link_H_sensor"] = transformToJSON(firstLink_H_sensor);
        sensorJSON["second_link_H_sensor"] = transformToJSON(secondLink_H_sensor);
        break;
    }
    case ACCELEROMETER:
    {
        const auto* accelerometer = dynamic_cast<const AccelerometerSensor*>(&sensor);
        sensorJSON["type"] = "accelerometer";
        sensorJSON["parent_link"] = accelerometer->getParentLink();
        sensorJSON["link_H_sensor"] = transformToJSON(accelerometer->getLinkSensorTransform());
        break;
    }
    case GYROSCOPE:
    {
        const auto* gyroscope = dynamic_cast<const GyroscopeSensor*>(&sensor);
        sensorJSON["type"] = "gyroscope";
        sensorJSON["parent_link"] = gyroscope->getParentLink();
        sensorJSON["link_H_sensor"] = transformToJSON(gyroscope->getLinkSensorTransform());
        break;
    }
    case THREE_AXIS_ANGULAR_ACCELEROMETER:
    {
        const auto* angularAccelerometer
            = dynamic_cast<const ThreeAxisAngularAccelerometerSensor*>(&sensor);
        sensorJSON["type"] = "three_axis_angular_accelerometer";
        sensorJSON["parent_link"] = angularAccelerometer->getParentLink();
        sensorJSON["link_H_sensor"]
            = transformToJSON(angularAccelerometer->getLinkSensorTransform());
        break;
    }
    case THREE_AXIS_FORCE_TORQUE_CONTACT:
    {
        const auto* contactSensor
            = dynamic_cast<const ThreeAxisForceTorqueContactSensor*>(&sensor);
        sensorJSON["type"] = "three_axis_force_torque_contact";
        sensorJSON["parent_link"] = contactSensor->getParentLink();
        sensorJSON["link_H_sensor"] = transformToJSON(contactSensor->getLinkSensorTransform());
        sensorJSON["load_cell_locations"] = nlohmann::json::array();
        for (const Position& loadCellLocation : contactSensor->getLoadCellLocations())
        {
            sensorJSON["load_cell_locations"].push_back(positionToJSON(loadCellLocation));
        }
        break;
    }
    default:
        reportError("ModelJSONExport", "sensorToJSON", "Unknown sensor type.");
        return nlohmann::json();
    }

    return sensorJSON;
}

static bool addSensorToModel(const nlohmann::json& sensorJSON, Model& model)
{
    const std::string sensorType = sensorJSON.at("type").get<std::string>();

    if (sensorType == "six_axis_force_torque")
    {
        const std::string parentJointName = sensorJSON.at("parent_joint").get<std::string>();
        const std::string firstLinkName = sensorJSON.at("first_link").get<std::string>();
        const std::string secondLinkName = sensorJSON.at("second_link").get<std::string>();
        const std::string appliedWrenchLinkName
            = sensorJSON.at("applied_wrench_link").get<std::string>();

        const JointIndex parentJointIndex = model.getJointIndex(parentJointName);
        const LinkIndex firstLinkIndex = model.getLinkIndex(firstLinkName);
        const LinkIndex secondLinkIndex = model.getLinkIndex(secondLinkName);
        const LinkIndex appliedWrenchLinkIndex = model.getLinkIndex(appliedWrenchLinkName);
        if (parentJointIndex == JOINT_INVALID_INDEX || firstLinkIndex == LINK_INVALID_INDEX
            || secondLinkIndex == LINK_INVALID_INDEX
            || appliedWrenchLinkIndex == LINK_INVALID_INDEX)
        {
            reportError("ModelJSONExport",
                        "addSensorToModel",
                        "Failed to resolve joint or link referenced by six axis force torque sensor.");
            return false;
        }

        SixAxisForceTorqueSensor sensor;
        sensor.setName(sensorJSON.at("name").get<std::string>());
        sensor.setParentJoint(parentJointName);
        sensor.setParentJointIndex(parentJointIndex);
        sensor.setFirstLinkName(firstLinkName);
        sensor.setSecondLinkName(secondLinkName);
        sensor.setFirstLinkSensorTransform(firstLinkIndex,
                                           transformFromJSON(sensorJSON.at("first_link_H_sensor")));
        sensor.setSecondLinkSensorTransform(secondLinkIndex,
                                            transformFromJSON(sensorJSON.at("second_link_H_sensor")));
        sensor.setAppliedWrenchLink(appliedWrenchLinkIndex);
        return model.sensors().addSensor(sensor) >= 0;
    }

    const std::string parentLinkName = sensorJSON.at("parent_link").get<std::string>();
    const LinkIndex parentLinkIndex = model.getLinkIndex(parentLinkName);
    if (parentLinkIndex == LINK_INVALID_INDEX)
    {
        reportError("ModelJSONExport",
                    "addSensorToModel",
                    ("Unknown parent link referenced by sensor: " + parentLinkName).c_str());
        return false;
    }

    if (sensorType == "accelerometer")
    {
        AccelerometerSensor sensor;
        sensor.setName(sensorJSON.at("name").get<std::string>());
        sensor.setParentLink(parentLinkName);
        sensor.setParentLinkIndex(parentLinkIndex);
        sensor.setLinkSensorTransform(transformFromJSON(sensorJSON.at("link_H_sensor")));
        return model.sensors().addSensor(sensor) >= 0;
    }
    if (sensorType == "gyroscope")
    {
        GyroscopeSensor sensor;
        sensor.setName(sensorJSON.at("name").get<std::string>());
        sensor.setParentLink(parentLinkName);
        sensor.setParentLinkIndex(parentLinkIndex);
        sensor.setLinkSensorTransform(transformFromJSON(sensorJSON.at("link_H_sensor")));
        return model.sensors().addSensor(sensor) >= 0;
    }
    if (sensorType == "three_axis_angular_accelerometer")
    {
        ThreeAxisAngularAccelerometerSensor sensor;
        sensor.setName(sensorJSON.at("name").get<std::string>());
        sensor.setParentLink(parentLinkName);
        sensor.setParentLinkIndex(parentLinkIndex);
        sensor.setLinkSensorTransform(transformFromJSON(sensorJSON.at("link_H_sensor")));
        return model.sensors().addSensor(sensor) >= 0;
    }
    if (sensorType == "three_axis_force_torque_contact")
    {
        ThreeAxisForceTorqueContactSensor sensor;
        sensor.setName(sensorJSON.at("name").get<std::string>());
        sensor.setParentLink(parentLinkName);
        sensor.setParentLinkIndex(parentLinkIndex);
        sensor.setLinkSensorTransform(transformFromJSON(sensorJSON.at("link_H_sensor")));

        std::vector<Position> loadCellLocations;
        for (const auto& loadCellLocationJSON : sensorJSON.at("load_cell_locations"))
        {
            loadCellLocations.push_back(positionFromJSON(loadCellLocationJSON));
        }
        sensor.setLoadCellLocations(loadCellLocations);
        return model.sensors().addSensor(sensor) >= 0;
    }

    reportError("ModelJSONExport", "addSensorToModel", "Unknown sensor type.");
    return false;
}

// Limits for 1-DOF joints
template <typename JointT>
static nlohmann::json oneDOFLimitsToJSON(const JointT& joint)
{
    nlohmann::json lim;
    lim["has_position_limits"] = joint.hasPosLimits();
    if (joint.hasPosLimits())
    {
        lim["min_position"] = joint.getMinPosLimit(0);
        lim["max_position"] = joint.getMaxPosLimit(0);
    }
    lim["has_effort_limits"] = joint.hasEffortLimits();
    if (joint.hasEffortLimits())
        lim["effort_limit"] = joint.getEffortLimit(0);
    lim["has_velocity_limits"] = joint.hasVelocityLimits();
    if (joint.hasVelocityLimits())
        lim["velocity_limit"] = joint.getVelocityLimit(0);
    return lim;
}

// Dynamics for 1-DOF joints
template <typename JointT>
static nlohmann::json oneDOFDynamicsToJSON(const JointT& joint)
{
    nlohmann::json dyn;
    dyn["type"] = (joint.getJointDynamicsType() == URDFJointDynamics) ? "URDFJointDynamics"
                                                                       : "NoJointDynamics";
    dyn["damping"] = joint.getDamping(0);
    dyn["static_friction"] = joint.getStaticFriction(0);
    return dyn;
}

// Limits for SphericalJoint (3-DOF, per-joint booleans / per-DOF values)
static nlohmann::json sphericalLimitsToJSON(const SphericalJoint& joint)
{
    nlohmann::json lim;
    lim["has_position_limits"] = joint.hasPosLimits();
    if (joint.hasPosLimits())
    {
        nlohmann::json minArr = nlohmann::json::array();
        nlohmann::json maxArr = nlohmann::json::array();
        for (size_t i = 0; i < 3; i++)
        {
            minArr.push_back(joint.getMinPosLimit(i));
            maxArr.push_back(joint.getMaxPosLimit(i));
        }
        lim["min_position"] = minArr;
        lim["max_position"] = maxArr;
    }
    lim["has_effort_limits"] = joint.hasEffortLimits();
    if (joint.hasEffortLimits())
    {
        nlohmann::json effortArr = nlohmann::json::array();
        for (size_t i = 0; i < 3; i++)
            effortArr.push_back(joint.getEffortLimit(i));
        lim["effort_limit"] = effortArr;
    }
    lim["has_velocity_limits"] = joint.hasVelocityLimits();
    if (joint.hasVelocityLimits())
    {
        nlohmann::json velArr = nlohmann::json::array();
        for (size_t i = 0; i < 3; i++)
            velArr.push_back(joint.getVelocityLimit(i));
        lim["velocity_limit"] = velArr;
    }
    return lim;
}

// Dynamics for SphericalJoint (per-DOF damping/friction)
static nlohmann::json sphericalDynamicsToJSON(const SphericalJoint& joint)
{
    nlohmann::json dyn;
    dyn["type"] = (joint.getJointDynamicsType() == URDFJointDynamics) ? "URDFJointDynamics"
                                                                       : "NoJointDynamics";
    nlohmann::json dampingArr = nlohmann::json::array();
    nlohmann::json frictionArr = nlohmann::json::array();
    for (size_t i = 0; i < 3; i++)
    {
        dampingArr.push_back(joint.getDamping(i));
        frictionArr.push_back(joint.getStaticFriction(i));
    }
    dyn["damping"] = dampingArr;
    dyn["static_friction"] = frictionArr;
    return dyn;
}

// ─── export ──────────────────────────────────────────────────────────────────

bool modelToJSONString(const Model& model, std::string& jsonString)
{
    nlohmann::json root;
    root["idyntree_model_json_version"] = IDYNTREE_MODEL_JSON_FORMAT_VERSION;
    root["default_base_link"] = model.getLinkName(model.getDefaultBaseLink());

    // Links
    nlohmann::json linksJSON = nlohmann::json::array();
    for (LinkIndex lnkIdx = 0; lnkIdx < model.getNrOfLinks(); lnkIdx++)
    {
        const Link* link = model.getLink(lnkIdx);
        const SpatialInertia& inertia = link->getInertia();

        nlohmann::json inertiaJSON;
        inertiaJSON["mass"] = inertia.getMass();
        inertiaJSON["center_of_mass"] = positionToJSON(inertia.getCenterOfMass());

        // Store raw rotational inertia wrt frame origin (what is directly stored internally)
        const RotationalInertia& rotI = inertia.getRotationalInertiaWrtFrameOrigin();
        nlohmann::json rotIJSON = nlohmann::json::array();
        for (int r = 0; r < 3; r++)
        {
            nlohmann::json row = nlohmann::json::array();
            for (int c = 0; c < 3; c++)
                row.push_back(rotI(r, c));
            rotIJSON.push_back(row);
        }
        inertiaJSON["rotational_inertia_wrt_frame_origin"] = rotIJSON;

        nlohmann::json linkJSON;
        linkJSON["name"] = model.getLinkName(lnkIdx);
        linkJSON["inertia"] = inertiaJSON;
        linksJSON.push_back(linkJSON);
    }
    root["links"] = linksJSON;

    // Joints
    nlohmann::json jointsJSON = nlohmann::json::array();
    for (JointIndex jntIdx = 0; jntIdx < model.getNrOfJoints(); jntIdx++)
    {
        const IJointConstPtr joint = model.getJoint(jntIdx);
        const LinkIndex link1 = joint->getFirstAttachedLink();
        const LinkIndex link2 = joint->getSecondAttachedLink();

        nlohmann::json jointJSON;
        jointJSON["name"] = model.getJointName(jntIdx);
        jointJSON["parent_link"] = model.getLinkName(link1);
        jointJSON["child_link"] = model.getLinkName(link2);
        // rest_transform = link1_X_link2:  getRestTransform(child=link1, parent=link2)
        // means p_link1 = T * p_link2, i.e. link1_X_link2
        jointJSON["rest_transform"] = transformToJSON(joint->getRestTransform(link1, link2));

        const auto* fixedJoint = dynamic_cast<const FixedJoint*>(joint);
        const auto* revoluteJoint = dynamic_cast<const RevoluteJoint*>(joint);
        const auto* prismaticJoint = dynamic_cast<const PrismaticJoint*>(joint);
        const auto* revoluteSO2Joint = dynamic_cast<const RevoluteSO2Joint*>(joint);
        const auto* sphericalJoint = dynamic_cast<const SphericalJoint*>(joint);

        if (fixedJoint)
        {
            jointJSON["type"] = "fixed";
        }
        else if (revoluteJoint)
        {
            jointJSON["type"] = "revolute";
            // axis expressed in child (link2) frame
            jointJSON["axis"] = axisToJSON(revoluteJoint->getAxis(link2, link1));
            jointJSON["limits"] = oneDOFLimitsToJSON(*revoluteJoint);
            jointJSON["dynamics"] = oneDOFDynamicsToJSON(*revoluteJoint);
        }
        else if (prismaticJoint)
        {
            jointJSON["type"] = "prismatic";
            jointJSON["axis"] = axisToJSON(prismaticJoint->getAxis(link2, link1));
            jointJSON["limits"] = oneDOFLimitsToJSON(*prismaticJoint);
            jointJSON["dynamics"] = oneDOFDynamicsToJSON(*prismaticJoint);
        }
        else if (revoluteSO2Joint)
        {
            jointJSON["type"] = "revolute_so2";
            jointJSON["axis"] = axisToJSON(revoluteSO2Joint->getAxis(link2, link1));
            jointJSON["limits"] = oneDOFLimitsToJSON(*revoluteSO2Joint);
            jointJSON["dynamics"] = oneDOFDynamicsToJSON(*revoluteSO2Joint);
        }
        else if (sphericalJoint)
        {
            jointJSON["type"] = "spherical";
            // joint center relative to link1 (parent)
            jointJSON["joint_center_wrt_parent"]
                = positionToJSON(sphericalJoint->getJointCenter(link1));
            jointJSON["limits"] = sphericalLimitsToJSON(*sphericalJoint);
            jointJSON["dynamics"] = sphericalDynamicsToJSON(*sphericalJoint);
        }
        else
        {
            reportError("ModelJSONExport",
                        "modelToJSONString",
                        "Unknown joint type encountered during export.");
            return false;
        }

        jointsJSON.push_back(jointJSON);
    }
    root["joints"] = jointsJSON;

    // Additional frames
    nlohmann::json framesJSON = nlohmann::json::array();
    for (FrameIndex frmIdx = model.getNrOfLinks(); frmIdx < model.getNrOfFrames(); frmIdx++)
    {
        nlohmann::json frameJSON;
        frameJSON["name"] = model.getFrameName(frmIdx);
        const LinkIndex parentLinkIdx = model.getFrameLink(frmIdx);
        frameJSON["parent_link"] = model.getLinkName(parentLinkIdx);
        frameJSON["link_H_frame"] = transformToJSON(model.getFrameTransform(frmIdx));
        framesJSON.push_back(frameJSON);
    }
    root["additional_frames"] = framesJSON;

    const nlohmann::json visualSolidShapesJSON
        = modelSolidShapesToJSON(model, model.visualSolidShapes());
    if (visualSolidShapesJSON.is_null())
    {
        return false;
    }
    root["visual_solid_shapes"] = visualSolidShapesJSON;

    const nlohmann::json collisionSolidShapesJSON
        = modelSolidShapesToJSON(model, model.collisionSolidShapes());
    if (collisionSolidShapesJSON.is_null())
    {
        return false;
    }
    root["collision_solid_shapes"] = collisionSolidShapesJSON;

    nlohmann::json sensorsJSON = nlohmann::json::array();
    for (SensorsList::const_iterator sensorIterator = model.sensors().allSensorsIterator();
         sensorIterator.isValid();
         ++sensorIterator)
    {
        const Sensor* sensor = *sensorIterator;
        if (sensor == nullptr)
        {
            reportError("ModelJSONExport",
                        "modelToJSONString",
                        "Encountered null sensor pointer during export.");
            return false;
        }

        const nlohmann::json sensorJSON = sensorToJSON(*sensor, model);
        if (sensorJSON.is_null())
        {
            return false;
        }
        sensorsJSON.push_back(sensorJSON);
    }
    root["sensors"] = sensorsJSON;

    jsonString = root.dump(2);
    return true;
}

bool modelToJSONFile(const Model& model, const std::string& filename)
{
    std::string jsonString;
    if (!modelToJSONString(model, jsonString))
        return false;

    std::ofstream ofs(filename);
    if (!ofs.is_open())
    {
        reportError("ModelJSONExport",
                    "modelToJSONFile",
                    ("Failed to open file for writing: " + filename).c_str());
        return false;
    }
    ofs << jsonString;
    return ofs.good();
}

// ─── import ──────────────────────────────────────────────────────────────────

// Helper: deserialize 1-DOF limits
template <typename JointT>
static bool applyOneDOFLimits(const nlohmann::json& limJ, JointT& joint)
{
    if (limJ.at("has_position_limits").get<bool>())
    {
        joint.enablePosLimits(true);

        const double minPosition = limJ.at("min_position").get<double>();
        const double maxPosition = limJ.at("max_position").get<double>();

        if (!joint.setPosLimits(0, minPosition, maxPosition))
        {
            return false;
        }
    }
    if (limJ.at("has_effort_limits").get<bool>())
    {
        joint.enableEffortLimits(true);
        joint.setEffortLimit(0, limJ.at("effort_limit").get<double>());
    }
    if (limJ.at("has_velocity_limits").get<bool>())
    {
        joint.enableVelocityLimits(true);
        joint.setVelocityLimit(0, limJ.at("velocity_limit").get<double>());
    }
    return true;
}

// Helper: deserialize 1-DOF dynamics
template <typename JointT>
static void applyOneDOFDynamics(const nlohmann::json& dynJ, JointT& joint)
{
    const std::string dynType = dynJ.at("type").get<std::string>();
    joint.setJointDynamicsType(dynType == "URDFJointDynamics" ? URDFJointDynamics
                                                               : NoJointDynamics);
    joint.setDamping(0, dynJ.at("damping").get<double>());
    joint.setStaticFriction(0, dynJ.at("static_friction").get<double>());
}

bool modelFromJSONString(const std::string& jsonString, Model& model)
{
    nlohmann::json root;
    try
    {
        root = nlohmann::json::parse(jsonString);
    }
    catch (const nlohmann::json::parse_error& e)
    {
        reportError("ModelJSONExport",
                    "modelFromJSONString",
                    ("JSON parse error: " + std::string(e.what())).c_str());
        return false;
    }

    // Version check
    if (!root.contains("idyntree_model_json_version"))
    {
        reportError("ModelJSONExport",
                    "modelFromJSONString",
                    "Missing field 'idyntree_model_json_version'.");
        return false;
    }
    const int version = root["idyntree_model_json_version"].get<int>();
    if (version != IDYNTREE_MODEL_JSON_FORMAT_VERSION)
    {
        reportError("ModelJSONExport",
                    "modelFromJSONString",
                    ("Unsupported idyntree_model_json_version: " + std::to_string(version)
                     + ". Supported version: "
                     + std::to_string(IDYNTREE_MODEL_JSON_FORMAT_VERSION))
                        .c_str());
        return false;
    }

    model = Model();

    // ── Links ────────────────────────────────────────────────────────────────
    for (const auto& linkJSON : root.at("links"))
    {
        const std::string name = linkJSON.at("name").get<std::string>();
        const auto& inertiaJSON = linkJSON.at("inertia");

        const double mass = inertiaJSON.at("mass").get<double>();
        const Position com = positionFromJSON(inertiaJSON.at("center_of_mass"));

        const auto& rotIJSON = inertiaJSON.at("rotational_inertia_wrt_frame_origin");
        double data[9];
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                data[r * 3 + c] = rotIJSON.at(r).at(c).get<double>();
        RotationalInertia rotI(data, 3, 3);

        // The SpatialInertia constructor stores rotInertia as-is (wrt frame origin).
        SpatialInertia spatialInertia(mass, com, rotI);

        Link link;
        link.setInertia(spatialInertia);
        model.addLink(name, link);
    }

    // ── Joints ───────────────────────────────────────────────────────────────
    for (const auto& jointJSON : root.at("joints"))
    {
        const std::string name = jointJSON.at("name").get<std::string>();
        const std::string type = jointJSON.at("type").get<std::string>();
        const std::string parentLinkName = jointJSON.at("parent_link").get<std::string>();
        const std::string childLinkName = jointJSON.at("child_link").get<std::string>();

        const LinkIndex parentIdx = model.getLinkIndex(parentLinkName);
        if (parentIdx == LINK_INVALID_INDEX)
        {
            reportError("ModelJSONExport",
                        "modelFromJSONString",
                        ("Joint '" + name + "': parent link not found: " + parentLinkName)
                            .c_str());
            return false;
        }
        const LinkIndex childIdx = model.getLinkIndex(childLinkName);
        if (childIdx == LINK_INVALID_INDEX)
        {
            reportError("ModelJSONExport",
                        "modelFromJSONString",
                        ("Joint '" + name + "': child link not found: " + childLinkName).c_str());
            return false;
        }

        // rest_transform is link1_X_link2 (parent_X_child)
        const Transform restTransform = transformFromJSON(jointJSON.at("rest_transform"));

        if (type == "fixed")
        {
            FixedJoint fixedJoint(parentIdx, childIdx, restTransform);
            model.addJoint(name, &fixedJoint);
        }
        else if (type == "revolute")
        {
            RevoluteJoint revJoint;
            revJoint.setAttachedLinks(parentIdx, childIdx);
            revJoint.setRestTransform(restTransform);
            const Axis axis = axisFromJSON(jointJSON.at("axis"));
            revJoint.setAxis(axis, childIdx, parentIdx);
            if (!applyOneDOFLimits(jointJSON.at("limits"), revJoint))
            {
                reportError("ModelJSONExport",
                            "modelFromJSONString",
                            ("Unable to set limits for revolute joint: " + name).c_str());
                return false;
            }
            applyOneDOFDynamics(jointJSON.at("dynamics"), revJoint);
            model.addJoint(name, &revJoint);
        }
        else if (type == "prismatic")
        {
            PrismaticJoint prismJoint;
            prismJoint.setAttachedLinks(parentIdx, childIdx);
            prismJoint.setRestTransform(restTransform);
            const Axis axis = axisFromJSON(jointJSON.at("axis"));
            prismJoint.setAxis(axis, childIdx, parentIdx);
            if (!applyOneDOFLimits(jointJSON.at("limits"), prismJoint))
            {
                reportError("ModelJSONExport",
                            "modelFromJSONString",
                            ("Unable to set limits for prismatic joint: " + name).c_str());
                return false;
            }
            applyOneDOFDynamics(jointJSON.at("dynamics"), prismJoint);
            model.addJoint(name, &prismJoint);
        }
        else if (type == "revolute_so2")
        {
            RevoluteSO2Joint revSO2Joint;
            revSO2Joint.setAttachedLinks(parentIdx, childIdx);
            revSO2Joint.setRestTransform(restTransform);
            const Axis axis = axisFromJSON(jointJSON.at("axis"));
            revSO2Joint.setAxis(axis, childIdx, parentIdx);
            if (!applyOneDOFLimits(jointJSON.at("limits"), revSO2Joint))
            {
                reportError("ModelJSONExport",
                            "modelFromJSONString",
                            ("Unable to set limits for revolute_so2 joint: " + name).c_str());
                return false;
            }
            applyOneDOFDynamics(jointJSON.at("dynamics"), revSO2Joint);
            model.addJoint(name, &revSO2Joint);
        }
        else if (type == "spherical")
        {
            SphericalJoint sphericalJoint;
            sphericalJoint.setAttachedLinks(parentIdx, childIdx);
            sphericalJoint.setRestTransform(restTransform);
            const Position center = positionFromJSON(jointJSON.at("joint_center_wrt_parent"));
            sphericalJoint.setJointCenter(parentIdx, center);

            const auto& limJSON = jointJSON.at("limits");
            if (limJSON.at("has_position_limits").get<bool>())
            {
                sphericalJoint.enablePosLimits(true);
                const auto& minArr = limJSON.at("min_position");
                const auto& maxArr = limJSON.at("max_position");
                for (size_t i = 0; i < 3; i++)
                    sphericalJoint.setPosLimits(
                        i, minArr.at(i).get<double>(), maxArr.at(i).get<double>());
            }
            if (limJSON.at("has_effort_limits").get<bool>())
            {
                sphericalJoint.enableEffortLimits(true);
                const auto& effortArr = limJSON.at("effort_limit");
                for (size_t i = 0; i < 3; i++)
                    sphericalJoint.setEffortLimit(i, effortArr.at(i).get<double>());
            }
            if (limJSON.at("has_velocity_limits").get<bool>())
            {
                sphericalJoint.enableVelocityLimits(true);
                const auto& velArr = limJSON.at("velocity_limit");
                for (size_t i = 0; i < 3; i++)
                    sphericalJoint.setVelocityLimit(i, velArr.at(i).get<double>());
            }

            const auto& dynJSON = jointJSON.at("dynamics");
            const std::string dynType = dynJSON.at("type").get<std::string>();
            sphericalJoint.setJointDynamicsType(dynType == "URDFJointDynamics" ? URDFJointDynamics
                                                                               : NoJointDynamics);
            const auto& dampingArr = dynJSON.at("damping");
            const auto& frictionArr = dynJSON.at("static_friction");
            for (size_t i = 0; i < 3; i++)
            {
                sphericalJoint.setDamping(i, dampingArr.at(i).get<double>());
                sphericalJoint.setStaticFriction(i, frictionArr.at(i).get<double>());
            }

            model.addJoint(name, &sphericalJoint);
        }
        else
        {
            reportError("ModelJSONExport",
                        "modelFromJSONString",
                        ("Unknown joint type: " + type).c_str());
            return false;
        }
    }

    // ── Additional frames ────────────────────────────────────────────────────
    for (const auto& frameJSON : root.at("additional_frames"))
    {
        const std::string frameName = frameJSON.at("name").get<std::string>();
        const std::string parentLinkName = frameJSON.at("parent_link").get<std::string>();
        const Transform link_H_frame = transformFromJSON(frameJSON.at("link_H_frame"));
        model.addAdditionalFrameToLink(parentLinkName, frameName, link_H_frame);
    }

    if (root.contains("visual_solid_shapes")
        && !modelSolidShapesFromJSON(root.at("visual_solid_shapes"), model, model.visualSolidShapes()))
    {
        return false;
    }

    if (root.contains("collision_solid_shapes")
        && !modelSolidShapesFromJSON(root.at("collision_solid_shapes"),
                                     model,
                                     model.collisionSolidShapes()))
    {
        return false;
    }

    if (root.contains("sensors"))
    {
        for (const auto& sensorJSON : root.at("sensors"))
        {
            if (!addSensorToModel(sensorJSON, model))
            {
                reportError("ModelJSONExport",
                            "modelFromJSONString",
                            ("Failed to import sensor: "
                             + sensorJSON.value("name", std::string("<unnamed>")))
                                .c_str());
                return false;
            }
        }
    }

    // ── Default base link ────────────────────────────────────────────────────
    const std::string defaultBaseLinkName = root.at("default_base_link").get<std::string>();
    const LinkIndex defaultBaseIdx = model.getLinkIndex(defaultBaseLinkName);
    if (defaultBaseIdx == LINK_INVALID_INDEX)
    {
        reportError("ModelJSONExport",
                    "modelFromJSONString",
                    ("Default base link not found: " + defaultBaseLinkName).c_str());
        return false;
    }
    model.setDefaultBaseLink(defaultBaseIdx);

    return true;
}

bool modelFromJSONFile(const std::string& filename, Model& model)
{
    std::ifstream ifs(filename);
    if (!ifs.is_open())
    {
        reportError("ModelJSONExport",
                    "modelFromJSONFile",
                    ("Failed to open file for reading: " + filename).c_str());
        return false;
    }
    std::stringstream ss;
    ss << ifs.rdbuf();
    return modelFromJSONString(ss.str(), model);
}

} // namespace iDynTree
