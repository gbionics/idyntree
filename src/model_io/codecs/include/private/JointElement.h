// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_MODELIO_URDF_JOINTELEMENT_H
#define IDYNTREE_MODELIO_URDF_JOINTELEMENT_H

#include <iDynTree/XMLElement.h>

#include <iDynTree/IJoint.h>

#include <iDynTree/Axis.h>
#include <iDynTree/Position.h>
#include <iDynTree/Transform.h>

#include <memory>
#include <string>
#include <unordered_map>

namespace iDynTree
{
class JointElement;
class XMLAttribute;

class IJoint;
class XMLParserState;
} // namespace iDynTree

class iDynTree::JointElement : public iDynTree::XMLElement
{

public:
    // ???: Not sure where this struct should be located
    struct JointInfo
    {
        std::shared_ptr<IJoint> joint;
        iDynTree::Axis axis;
        std::string parentLinkName;
        std::string childLinkName;
        bool hasJointCenter{false};
        bool isFloating{false};
        iDynTree::Position jointCenterInParentFrame{iDynTree::Position::Zero()};
    };

private:
    std::unordered_map<std::string, JointElement::JointInfo>& m_joints;
    std::unordered_map<std::string, JointElement::JointInfo>& m_fixedJoints;

    std::string m_jointName;
    std::string m_jointType;
    iDynTree::Transform m_jointFrame;
    iDynTree::Axis m_axis;
    std::string m_parentLink;
    std::string m_childLink;

    struct Limits
    {
        double positionLower;
        double positionUpper;
        double effort;
        double velocity;
    };

    struct JointDynamicsParams
    {
        JointDynamicsType jointDynamicsType;
        double damping;
        double staticFriction;
    };

    std::shared_ptr<Limits> m_limits;
    std::shared_ptr<JointDynamicsParams> m_dynamic_params;

public:
    explicit JointElement(XMLParserState& parserState,
                          std::unordered_map<std::string, JointElement::JointInfo>& joints,
                          std::unordered_map<std::string, JointElement::JointInfo>& fixedJoints);

    bool setAttributes(
        const std::unordered_map<std::string, std::shared_ptr<XMLAttribute>>& attributes) override;

    std::shared_ptr<XMLElement> childElementForName(const std::string& name) override;

    virtual void exitElementScope() override;
};

#endif /* end of include guard: IDYNTREE_MODELIO_URDF_JOINTELEMENT_H */
