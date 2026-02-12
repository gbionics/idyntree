// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "URDFModelExport.h"

#include <iDynTree/Axis.h>
#include <iDynTree/EigenHelpers.h>
#include <iDynTree/FixedJoint.h>
#include <iDynTree/Indices.h>
#include <iDynTree/Model.h>
#include <iDynTree/ModelTransformers.h>
#include <iDynTree/PrismaticJoint.h>
#include <iDynTree/RevoluteJoint.h>
#include <iDynTree/SpatialInertia.h>
#include <iDynTree/SphericalJoint.h>
#include <iDynTree/Transform.h>
#include <iDynTree/Traversal.h>
#include <iDynTree/VectorFixSize.h>

#include "URDFParsingUtils.h"

#include <fstream>
#include <iomanip>
#include <string>

#include <libxml/parser.h>
#include <libxml/tree.h>
#include <libxml/xmlstring.h>

namespace iDynTree
{

/*
 * This file uses extensivly the libxml2's tree API, see
 * http://www.xmlsoft.org/examples/index.html#Tree for more info.
 * Usage of the C-based libxml2 API in C++ means that is necessary to
 * use the BAD_CAST macro for handling const-correctness.
 * See
 * https://stackoverflow.com/questions/45058878/why-libxml2-uses-bad-cast-everywhere-in-a-c-c-code
 * and http://www.xmlsoft.org/html/libxml-xmlstring.html#BAD_CAST for details on this.
 */

/*
 * Add a <origin> URDF element to the specified parent element with the specified transform.
 *
 * If parent_element contains a pointer to a tag <parent_element></parent_element>, this function
 * modifies it to be something like:
 * <parent_element>
 *   <origin xyz="1.0 2.0 3.0" rpy="0.0 1.0 2.0" />
 * </parent_element>
 * where the actual values of the xyz and rpy attributes depend on the input trans argument.
 */
bool exportTransform(const Transform& trans, xmlNodePtr parent_element, int precision = 0)
{
    bool ok = true;
    xmlNodePtr origin = xmlNewChild(parent_element, NULL, BAD_CAST "origin", NULL);
    std::string pose_xyz_str;
    ok = ok && vectorToString(trans.getPosition(), pose_xyz_str, precision);
    std::string pose_rpy_str;
    Vector3 pose_rpy = trans.getRotation().asRPY();
    ok = ok && vectorToString(pose_rpy, pose_rpy_str, precision);
    xmlNewProp(origin, BAD_CAST "xyz", BAD_CAST pose_xyz_str.c_str());
    xmlNewProp(origin, BAD_CAST "rpy", BAD_CAST pose_rpy_str.c_str());
    return ok;
}

/*
 * Add a <inertial> URDF element to the specified parent element with the specified 6D inertia.
 *
 * If parent_element contains a pointer to a tag <parent_element></parent_element>, this function
 * modifies it to be something like: <parent_element> <inertial> <origin xyz="0 0 0.5" rpy="0 0 0"/>
 *     <mass value="1"/>
 *     <inertia ixx="100"  ixy="0"  ixz="0" iyy="100" iyz="0" izz="100" />
 *   </inertial>
 * </parent_element>
 * where the actual values of the origin, mass and inertia element's attributes depend on the input
 * inertia argument.
 */
bool exportInertial(const SpatialInertia& inertia, xmlNodePtr parent_element, int precision = 0)
{
    bool ok = true;
    std::string bufStr;
    xmlNodePtr inertial = xmlNewChild(parent_element, NULL, BAD_CAST "inertial", NULL);

    xmlNodePtr mass_xml = xmlNewChild(inertial, NULL, BAD_CAST "mass", NULL);
    ok = ok && doubleToStringWithClassicLocale(inertia.getMass(), bufStr, precision);
    xmlNewProp(mass_xml, BAD_CAST "value", BAD_CAST bufStr.c_str());

    ok = ok
         && exportTransform(Transform(Rotation::Identity(), inertia.getCenterOfMass()),
                            inertial,
                            precision);

    xmlNodePtr inertia_xml = xmlNewChild(inertial, NULL, BAD_CAST "inertia", NULL);
    RotationalInertia rotInertia = inertia.getRotationalInertiaWrtCenterOfMass();
    ok = ok && doubleToStringWithClassicLocale(rotInertia(0, 0), bufStr, precision);
    xmlNewProp(inertia_xml, BAD_CAST "ixx", BAD_CAST bufStr.c_str());
    ok = ok && doubleToStringWithClassicLocale(rotInertia(0, 1), bufStr, precision);
    xmlNewProp(inertia_xml, BAD_CAST "ixy", BAD_CAST bufStr.c_str());
    ok = ok && doubleToStringWithClassicLocale(rotInertia(0, 2), bufStr, precision);
    xmlNewProp(inertia_xml, BAD_CAST "ixz", BAD_CAST bufStr.c_str());
    ok = ok && doubleToStringWithClassicLocale(rotInertia(1, 1), bufStr, precision);
    xmlNewProp(inertia_xml, BAD_CAST "iyy", BAD_CAST bufStr.c_str());
    ok = ok && doubleToStringWithClassicLocale(rotInertia(1, 2), bufStr, precision);
    xmlNewProp(inertia_xml, BAD_CAST "iyz", BAD_CAST bufStr.c_str());
    ok = ok && doubleToStringWithClassicLocale(rotInertia(2, 2), bufStr, precision);
    xmlNewProp(inertia_xml, BAD_CAST "izz", BAD_CAST bufStr.c_str());

    return ok;
}

/*
 * Add a <visual> or <collision> URDF element to the specified parent element for the specified
 * solid shapes.
 *
 * URDF support <box>, <cylinder>, <sphere> and <mesh> elements as shapes.
 *
 * If parent_element contains a pointer to a tag <parent_element></parent_element>, this function
 * modifies it to be something like: <parent_element> <visual> <origin xyz="0 0 0" rpy="0 0 0" />
 *     <geometry>
 *       <box size="1 1 1" />
 *     </geometry>
 *     <material name="Cyan">
 *       <color rgba="0 1.0 1.0 1.0"/>
 *     </material>
 *   </visual>
 * <parent_element/>
 * where the actual values of the added element and attributes depend on the input solidShape.
 */
enum exportSolidShapePropertyType
{
    VISUAL,
    COLLISION
};
bool exportSolidShape(const SolidShape* solidShape,
                      exportSolidShapePropertyType type,
                      xmlNodePtr parent_element,
                      int precision = 0)
{
    bool ok = true;
    std::string element_name;
    if (type == VISUAL)
    {
        element_name = "visual";
    } else
    {
        // assert(type == COLLISION)
        element_name = "collision";
    }

    xmlNodePtr root_shape_xml
        = xmlNewChild(parent_element, NULL, BAD_CAST element_name.c_str(), NULL);
    // Omit the name (that is an optional attribute) if the shape has no name
    if (solidShape->isNameValid())
    {
        std::string shapeName = solidShape->getName();
        xmlNewProp(root_shape_xml, BAD_CAST "name", BAD_CAST shapeName.c_str());
    }

    // Export transform
    ok = ok && exportTransform(solidShape->getLink_H_geometry(), root_shape_xml, precision);

    // Export geometry
    xmlNodePtr geometry_xml = xmlNewChild(root_shape_xml, NULL, BAD_CAST "geometry", NULL);

    if (solidShape->isBox())
    {
        const Box* box = solidShape->asBox();

        // Export box element
        xmlNodePtr box_xml = xmlNewChild(geometry_xml, NULL, BAD_CAST "box", NULL);

        // Export size attribute
        double size_data[3] = {box->getX(), box->getY(), box->getZ()};
        std::string size_str;
        ok = ok && vectorToString(Vector3(size_data, 3), size_str, precision);
        xmlNewProp(box_xml, BAD_CAST "size", BAD_CAST size_str.c_str());

    } else if (solidShape->isCylinder())
    {
        const Cylinder* cylinder = solidShape->asCylinder();

        // Export cylinder element
        xmlNodePtr cylinder_xml = xmlNewChild(geometry_xml, NULL, BAD_CAST "cylinder", NULL);

        // Export radius attribute
        std::string radius_str;
        ok = ok && doubleToStringWithClassicLocale(cylinder->getRadius(), radius_str, precision);
        xmlNewProp(cylinder_xml, BAD_CAST "radius", BAD_CAST radius_str.c_str());

        // Export length attribute
        std::string length_str;
        ok = ok && doubleToStringWithClassicLocale(cylinder->getLength(), length_str, precision);
        xmlNewProp(cylinder_xml, BAD_CAST "length", BAD_CAST length_str.c_str());

    } else if (solidShape->isSphere())
    {
        const Sphere* sphere = solidShape->asSphere();

        // Export sphere  element
        xmlNodePtr sphere_xml = xmlNewChild(geometry_xml, NULL, BAD_CAST "sphere", NULL);

        // Export radius attribute
        std::string radius_str;
        ok = ok && doubleToStringWithClassicLocale(sphere->getRadius(), radius_str, precision);
        xmlNewProp(sphere_xml, BAD_CAST "radius", BAD_CAST radius_str.c_str());

    } else if (solidShape->isExternalMesh())
    {
        const ExternalMesh* mesh = solidShape->asExternalMesh();

        // Export mesh element
        xmlNodePtr mesh_xml = xmlNewChild(geometry_xml, NULL, BAD_CAST "mesh", NULL);

        // Export filename attribute
        xmlNewProp(mesh_xml, BAD_CAST "filename", BAD_CAST mesh->getFilename().c_str());

        // Export scale attribute
        std::string scale_str;
        ok = ok && vectorToString(mesh->getScale(), scale_str, precision);
        xmlNewProp(mesh_xml, BAD_CAST "scale", BAD_CAST scale_str.c_str());

    } else
    {
        std::cerr << "[ERROR] URDFModelExport: Impossible to convert geometry of type "
                  << typeid(solidShape).name() << " to a URDF geometry." << std::endl;
        return false;
    }

    // Export material if set.
    if (solidShape->isMaterialSet())
    {
        auto material = solidShape->getMaterial();
        xmlNodePtr material_xml = xmlNewChild(root_shape_xml, NULL, BAD_CAST "material", NULL);
        xmlNewProp(material_xml, BAD_CAST "name", BAD_CAST material.name().c_str());

        if (material.hasColor())
        {
            xmlNodePtr color_xml = xmlNewChild(material_xml, NULL, BAD_CAST "color", NULL);
            std::string material_str;
            ok = ok && vectorToString(material.color(), material_str, precision);
            xmlNewProp(color_xml, BAD_CAST "rgba", BAD_CAST material_str.c_str());
        }
        if (material.hasTexture())
        {
            xmlNodePtr texture_xml = xmlNewChild(material_xml, NULL, BAD_CAST "texture", NULL);
            xmlNewProp(texture_xml, BAD_CAST "filename", BAD_CAST material.texture().c_str());
        }
    }

    return ok;
}

/*
 * Add a <link> URDF element to the specified parent element with the specified link info.
 *
 *
 * If parent_element contains a pointer to a tag <parent_element></parent_element>, this function
 * modifies it to be something like:
 *
 * <parent_element>
 *   <link name="my_link">
 *     <inertial>
 *       ...
 *     </inertial>
 *
 *  </link>
 * <parent_element/>
 *
 * where the actual values of the added element and attributes depend on the input link and linkName
 * arguments.
 */
bool exportLink(const Link& link,
                const std::string linkName,
                const Model& model,
                xmlNodePtr parent_element,
                int precision = 0)
{
    bool ok = true;
    xmlNodePtr link_xml = xmlNewChild(parent_element, NULL, BAD_CAST "link", NULL);
    xmlNewProp(link_xml, BAD_CAST "name", BAD_CAST linkName.c_str());

    ok = ok && exportInertial(link.getInertia(), link_xml, precision);

    LinkIndex linkIndex = model.getLinkIndex(linkName);

    // Export visual shapes
    for (unsigned int shapeIdx = 0;
         shapeIdx < model.visualSolidShapes().getLinkSolidShapes()[linkIndex].size();
         shapeIdx++)
    {
        SolidShape* exportedShape
            = model.visualSolidShapes().getLinkSolidShapes()[linkIndex][shapeIdx];
        exportSolidShape(exportedShape, VISUAL, link_xml, precision);
    }

    // Export collision shapes
    for (unsigned int shapeIdx = 0;
         shapeIdx < model.collisionSolidShapes().getLinkSolidShapes()[linkIndex].size();
         shapeIdx++)
    {
        // Clone the shape
        SolidShape* exportedShape
            = model.collisionSolidShapes().getLinkSolidShapes()[linkIndex][shapeIdx];
        exportSolidShape(exportedShape, COLLISION, link_xml, precision);
    }

    return ok;
}

/**
 * Add a <joint> URDF element to the specified parent element with the specified joint info.
 *
 * At the moment, only the URDF joint of types revolute, continuous, prismatic and fixed are
 * supported, as they are the only one supported in the iDynTree::Model class.
 *
 * If parent_element contains a pointer to a tag <parent_element></parent_element>, this function
 * modifies it to be something like:
 *
 * <parent_element>
 *   <joint name="my_joint" type="floating">
 *      <origin xyz="0 0 1" rpy="0 0 3.1416"/>
 *      <parent link="link1"/>
 *      <child link="link2"/>
 *      <calibration rising="0.0"/>
 *      <dynamics damping="0.0" friction="0.0"/>
 *      <limit effort="30" velocity="1.0" lower="-2.2" upper="0.7" />
 *   </joint>
 * <parent_element/>
 *
 * where the actual values of the added element and attributes depend on the input joint, parentLink
 * and childLink arguments.
 */
bool exportJoint(IJointConstPtr joint,
                 LinkConstPtr parentLink,
                 LinkConstPtr childLink,
                 const Model& model,
                 xmlNodePtr parent_element,
                 int precision = 0)
{
    bool ok = true;
    xmlNodePtr joint_xml = xmlNewChild(parent_element, NULL, BAD_CAST "joint", NULL);
    xmlNewProp(joint_xml, BAD_CAST "name", BAD_CAST model.getJointName(joint->getIndex()).c_str());

    if (dynamic_cast<const FixedJoint*>(joint))
    {
        xmlNewProp(joint_xml, BAD_CAST "type", BAD_CAST "fixed");
    } else if (dynamic_cast<const RevoluteJoint*>(joint))
    {
        if (joint->hasPosLimits())
        {
            xmlNewProp(joint_xml, BAD_CAST "type", BAD_CAST "revolute");
        } else
        {
            xmlNewProp(joint_xml, BAD_CAST "type", BAD_CAST "continuous");
        }
    } else if (dynamic_cast<const PrismaticJoint*>(joint))
    {
        xmlNewProp(joint_xml, BAD_CAST "type", BAD_CAST "prismatic");
    } else if (dynamic_cast<const SphericalJoint*>(joint))
    {
        std::cerr << "[ERROR] URDFModelExport: Impossible to convert joint of type SphericalJoint "
                     "to a URDF joint (spherical joints are not supported in URDF)."
                  << " For exporting joint of type SphericalJoint, ensure that the  "
                     "ModelExporterOptions::SphericalJointsAsThreeRevoluteJoints parameter is set "
                     "to true."
                  << std::endl;
        return false;
    } else
    {
        std::cerr << "[ERROR] URDFModelExport: Impossible to convert joint of type "
                  << typeid(joint).name() << " to a URDF joint " << std::endl;
        return false;
    }

    // origin
    exportTransform(joint->getRestTransform(parentLink->getIndex(), childLink->getIndex()),
                    joint_xml,
                    precision);

    if (joint->getNrOfDOFs() != 0)
    {
        Axis axis;

        // Check that the axis of the joint is supported by URDF
        if (dynamic_cast<const RevoluteJoint*>(joint))
        {
            const RevoluteJoint* revJoint = dynamic_cast<const RevoluteJoint*>(joint);
            axis = revJoint->getAxis(childLink->getIndex());
        } else if (dynamic_cast<const PrismaticJoint*>(joint))
        {
            const PrismaticJoint* prismJoint = dynamic_cast<const PrismaticJoint*>(joint);
            axis = prismJoint->getAxis(childLink->getIndex());
        } else
        {
            std::cerr << "[ERROR] URDFModelExport: Impossible to convert joint of type "
                      << typeid(joint).name() << " to a URDF joint " << std::endl;
            return false;
        }

        // Check that the axis is URDF-compatible, i.e. that the distance between the
        // axis and the child link origin is zero
        double distanceBetweenAxisAndOrigin
            = axis.getDistanceBetweenAxisAndPoint(iDynTree::Position::Zero());
        if (distanceBetweenAxisAndOrigin > 1e-7)
        {
            std::cerr << "[ERROR] URDFModelExport: Impossible to convert joint "
                      << model.getJointName(joint->getIndex())
                      << " to a URDF joint without moving the link frame of link "
                      << model.getLinkName(childLink->getIndex())
                      << " , because the axis origin is " << axis.getOrigin().toString()
                      << " the axis direction is " << axis.getDirection().toString()
                      << " and the distance between the axis and (0,0,0) is "
                      << distanceBetweenAxisAndOrigin << std::endl;
            return false;
        }

        // axis
        xmlNodePtr axis_xml = xmlNewChild(joint_xml, NULL, BAD_CAST "axis", NULL);
        std::string bufStr;
        ok = ok && vectorToString(axis.getDirection(), bufStr, precision);
        xmlNewProp(axis_xml, BAD_CAST "xyz", BAD_CAST bufStr.c_str());
    }

    // parent
    xmlNodePtr parent_xml = xmlNewChild(joint_xml, NULL, BAD_CAST "parent", NULL);
    xmlNewProp(parent_xml,
               BAD_CAST "link",
               BAD_CAST model.getLinkName(parentLink->getIndex()).c_str());

    // child
    xmlNodePtr child_xml = xmlNewChild(joint_xml, NULL, BAD_CAST "child", NULL);
    xmlNewProp(child_xml,
               BAD_CAST "link",
               BAD_CAST model.getLinkName(childLink->getIndex()).c_str());

    // Position limits
    // Limits are required for prismatic joints even if no limits are enabled, see URDF spec for
    // reference
    bool isJointPrismatic = dynamic_cast<const PrismaticJoint*>(joint);
    if ((joint->hasPosLimits() && joint->getNrOfDOFs() == 1) || isJointPrismatic)
    {
        xmlNodePtr limit_xml = xmlNewChild(joint_xml, NULL, BAD_CAST "limit", NULL);
        std::string bufStr;
        double min, max;
        double reallyHighLimit = 1e9;

        // If joint is prismatic and has no limits, we just put really high values as limits are
        // required
        if (isJointPrismatic && !joint->hasPosLimits())
        {
            min = -reallyHighLimit;
            max = +reallyHighLimit;
        } else
        {
            ok = ok && joint->getPosLimits(0, min, max);
        }

        ok = ok && doubleToStringWithClassicLocale(min, bufStr, precision);
        xmlNewProp(limit_xml, BAD_CAST "lower", BAD_CAST bufStr.c_str());
        ok = ok && doubleToStringWithClassicLocale(max, bufStr, precision);
        xmlNewProp(limit_xml, BAD_CAST "upper", BAD_CAST bufStr.c_str());

        // Write effort limit if available, otherwise use a high default value
        double effortLimit = reallyHighLimit;
        if (joint->hasEffortLimits())
        {
            effortLimit = joint->getEffortLimit(0);
        }
        ok = ok && doubleToStringWithClassicLocale(effortLimit, bufStr, precision);
        xmlNewProp(limit_xml, BAD_CAST "effort", BAD_CAST bufStr.c_str());

        // Write velocity limit if available, otherwise use a high default value
        double velocityLimit = reallyHighLimit;
        if (joint->hasVelocityLimits())
        {
            velocityLimit = joint->getVelocityLimit(0);
        }
        ok = ok && doubleToStringWithClassicLocale(velocityLimit, bufStr, precision);
        xmlNewProp(limit_xml, BAD_CAST "velocity", BAD_CAST bufStr.c_str());
    }

    if (joint->getJointDynamicsType() == URDFJointDynamics && joint->getNrOfDOFs() == 1)
    {
        xmlNodePtr dynamics_xml = xmlNewChild(joint_xml, NULL, BAD_CAST "dynamics", NULL);
        std::string bufStr;
        double damping = 0.0, static_friction = 0.0;
        damping = joint->getDamping(0);
        static_friction = joint->getStaticFriction(0);
        ok = ok && doubleToStringWithClassicLocale(damping, bufStr, precision);
        xmlNewProp(dynamics_xml, BAD_CAST "damping", BAD_CAST bufStr.c_str());
        ok = ok && doubleToStringWithClassicLocale(static_friction, bufStr, precision);
        xmlNewProp(dynamics_xml, BAD_CAST "friction", BAD_CAST bufStr.c_str());
    }

    return ok;
}

/*
 * Add a <link> and <joint> URDF elements to the specified parent element for the specified
 * additional frame.
 *
 * URDF does not have the concept of frame (see
 * https://discourse.ros.org/t/urdf-ng-link-and-frame-concepts/56), so for each additional frame we
 * need to add a URDF "fake" mass-less link.
 *
 * If parent_element contains a pointer to a tag <parent_element></parent_element>, this function
 * modifies it to be something like:
 *
 * <parent_element>
 *   <link name="<frame_name>"/>
 *   <joint name="<frame_name>_fixed_joint" type="fixed">
 *      <origin xyz="0.1 0.2 0.3" rpy="0 0 3.1416"/>
 *      <parent link="<link_name>"/>
 *      <child link="<frame_name>"/>
 *   </joint>
 * <parent_element/>
 *
 * where the actual values of the added element and attributes depend on the input frame_name,
 * link_H_frame, parent_link_name and direction_options arguments.
 */
enum exportAdditionalFrameDirectionOption
{
    FAKE_LINK_IS_CHILD,
    FAKE_LINK_IS_PARENT
};
bool exportXMLBlob(std::string xmlBlob, xmlNodePtr parent_element)
{
    xmlNodePtr new_node = nullptr;
    // Let's remove non printable characters
    xmlBlob.erase(std::remove_if(xmlBlob.begin(),
                                 xmlBlob.end(),
                                 [](unsigned char c) { return !std::isprint(c); }),
                  xmlBlob.end());
    // we assume that 'parent_element' node is already defined
    xmlParseInNodeContext(parent_element, xmlBlob.c_str(), strlen(xmlBlob.c_str()), 0, &new_node);
    if (new_node)
        xmlAddChild(parent_element, new_node);
    return true;
}
bool exportAdditionalFrame(const std::string frame_name,
                           Transform link_H_frame,
                           const std::string link_name,
                           exportAdditionalFrameDirectionOption direction_option,
                           xmlNodePtr parent_element,
                           int precision = 0)
{
    bool ok = true;

    // Export fake link
    xmlNodePtr link_xml = xmlNewChild(parent_element, NULL, BAD_CAST "link", NULL);
    xmlNewProp(link_xml, BAD_CAST "name", BAD_CAST frame_name.c_str());

    // Export fake joint
    xmlNodePtr joint_xml = xmlNewChild(parent_element, NULL, BAD_CAST "joint", NULL);
    std::string fake_joint_name = frame_name + "_fixed_joint";
    xmlNewProp(joint_xml, BAD_CAST "name", BAD_CAST fake_joint_name.c_str());
    xmlNewProp(joint_xml, BAD_CAST "type", BAD_CAST "fixed");

    // origin
    exportTransform(link_H_frame, joint_xml, precision);

    std::string parent_of_fake_joint, child_of_fake_joint;

    if (direction_option == FAKE_LINK_IS_CHILD)
    {
        parent_of_fake_joint = link_name;
        child_of_fake_joint = frame_name;
    } else
    {
        // direction_option == FAKE_LINK_IS_PARENT
        parent_of_fake_joint = frame_name;
        child_of_fake_joint = link_name;
    }

    // parent
    xmlNodePtr parent_xml = xmlNewChild(joint_xml, NULL, BAD_CAST "parent", NULL);
    xmlNewProp(parent_xml, BAD_CAST "link", BAD_CAST parent_of_fake_joint.c_str());

    // child
    xmlNodePtr child_xml = xmlNewChild(joint_xml, NULL, BAD_CAST "child", NULL);
    xmlNewProp(child_xml, BAD_CAST "link", BAD_CAST child_of_fake_joint.c_str());

    return ok;
}

bool URDFStringFromModel(const iDynTree::Model& model,
                         std::string& urdf_string,
                         const ModelExporterOptions options)
{
    Model processedModel;
    // Convert spherical joints if option is enabled
    if (options.exportSphericalJointsAsThreeRevoluteJoints)
    {
        if (!convertSphericalJointsToThreeRevoluteJoints(model,
                                                         processedModel,
                                                         options.sphericalJointFakeLinkPrefix,
                                                         options.sphericalJointRevoluteJointPrefix))
        {
            std::cerr << "[ERROR] URDFStringFromModel: Failed to convert spherical joints for URDF "
                         "export"
                      << std::endl;
            return false;
        }
    } else
    {
        processedModel = model;
    }

    bool ok = true;
    xmlDocPtr urdf_xml = xmlNewDoc(BAD_CAST "1.0");
    xmlNodePtr robot = xmlNewNode(NULL, BAD_CAST "robot");
    // TODO(traversaro) properly export this :)
    xmlNewProp(robot, BAD_CAST "name", BAD_CAST options.robotExportedName.c_str());
    xmlDocSetRootElement(urdf_xml, robot);

    // Extract precision parameter
    int precision = options.numericalPrecision;

    // TODO(traversaro) : We are assuming that the model has no loops,
    //                    we should add a check for this

    // URDF assumes a directed tree of the multibody model structure, so we need to assume that a
    // given link is the base
    std::string baseLink = options.baseLink;
    if (baseLink.empty())
    {
        baseLink = processedModel.getLinkName(processedModel.getDefaultBaseLink());
    }

    LinkIndex baseLinkIndex = processedModel.getLinkIndex(baseLink);
    if (baseLinkIndex == LINK_INVALID_INDEX)
    {
        std::cerr << "[ERROR] URDFStringFromModel : specified baseLink " << baseLink
                  << " is not part of the model" << std::endl;
        xmlFreeDoc(urdf_xml);
        return false;
    }

    // If the base link has at least an additional frame, add it as parent URDF link
    // as a workaround for https://github.com/ros/kdl_parser/issues/27, unless
    // options.exportFirstBaseLinkAdditionalFrameAsFakeURDFBase is set to false
    // If options.exportFirstBaseLinkAdditionalFrameAsFakeURDFBase is set to false,
    // baseFakeLinkFrameIndex remains set to FRAME_INVALID_INDEX, a
    // and all the additional frames of the base link get exported as child fake links
    // in the loop and the end of this function
    FrameIndex baseFakeLinkFrameIndex = FRAME_INVALID_INDEX;
    std::vector<FrameIndex> frameIndices;
    ok = model.getLinkAdditionalFrames(baseLinkIndex, frameIndices);
    if (ok && frameIndices.size() >= 1 && options.exportFirstBaseLinkAdditionalFrameAsFakeURDFBase)
    {
        baseFakeLinkFrameIndex = frameIndices[0];
        ok = ok
             && exportAdditionalFrame(processedModel.getFrameName(baseFakeLinkFrameIndex),
                                      processedModel.getFrameTransform(baseFakeLinkFrameIndex),
                                      processedModel.getLinkName(
                                          processedModel.getFrameLink(baseFakeLinkFrameIndex)),
                                      FAKE_LINK_IS_PARENT,
                                      robot,
                                      precision);
    }

    // Create a Traversal
    Traversal exportTraversal;
    if (!processedModel.computeFullTreeTraversal(exportTraversal, baseLinkIndex))
    {
        std::cerr << "[ERROR] URDFStringFromModel : error in computeFullTreeTraversal" << std::endl;
        xmlFreeDoc(urdf_xml);
        return false;
    }

    // Export links and joints following the traversal
    for (TraversalIndex trvIdx = 0;
         trvIdx < static_cast<TraversalIndex>(exportTraversal.getNrOfVisitedLinks());
         trvIdx++)
    {
        LinkConstPtr visitedLink = exportTraversal.getLink(trvIdx);
        std::string visitedLinkName = processedModel.getLinkName(visitedLink->getIndex());

        // Export parent joint, if this is not the base
        if (trvIdx != 0)
        {
            LinkConstPtr parentLink = exportTraversal.getParentLink(trvIdx);
            IJointConstPtr parentJoint = exportTraversal.getParentJoint(trvIdx);
            ok = ok
                 && exportJoint(parentJoint,
                                parentLink,
                                visitedLink,
                                processedModel,
                                robot,
                                precision);
        }

        // Export link
        ok = ok && exportLink(*visitedLink, visitedLinkName, processedModel, robot, precision);
    }

    // Export all the additional frames.
    for (FrameIndex frameIndex = processedModel.getNrOfLinks();
         frameIndex < static_cast<FrameIndex>(processedModel.getNrOfFrames());
         frameIndex++)
    {
        // Check that the frame is not parent of the base link and that the
        // frame link is present in the traversal.
        if (frameIndex != baseFakeLinkFrameIndex
            && exportTraversal.getTraversalIndexFromLinkIndex(
                   processedModel.getFrameLink(frameIndex))
                   != TRAVERSAL_INVALID_INDEX)
        {

            ok = ok
                 && exportAdditionalFrame(processedModel.getFrameName(frameIndex),
                                          processedModel.getFrameTransform(frameIndex),
                                          processedModel.getLinkName(
                                              processedModel.getFrameLink(frameIndex)),
                                          FAKE_LINK_IS_CHILD,
                                          robot,
                                          precision);
        }
    }
    for (auto& xmlBlob : options.xmlBlobs)
    {
        ok = ok && exportXMLBlob(xmlBlob, robot);
    }
    xmlChar* xmlbuff = 0;
    int buffersize = 0;
    xmlDocDumpFormatMemory(urdf_xml, &xmlbuff, &buffersize, 1);
    urdf_string.resize(buffersize);
    std::memcpy((void*)urdf_string.data(), (void*)xmlbuff, buffersize);

    xmlFree(xmlbuff);
    xmlFreeDoc(urdf_xml);

    return ok;
}

bool URDFFromModel(const iDynTree::Model& model,
                   const std::string& urdf_filename,
                   const ModelExporterOptions options)
{
    std::ofstream ofs(urdf_filename.c_str());

    if (!ofs.is_open())
    {
        std::cerr << "[ERROR] iDynTree::URDFFromModel : error opening file " << urdf_filename
                  << std::endl;
        return false;
    }

    std::string xml_string;
    if (!URDFStringFromModel(model, xml_string, options))
    {
        std::cerr << "[ERROR] URDFFromModel: Failed to generate URDF string from processed model"
                  << std::endl;
        return false;
    }

    // Write string to file
    ofs << xml_string;
    ofs.close();

    return true;
}

} // namespace iDynTree
