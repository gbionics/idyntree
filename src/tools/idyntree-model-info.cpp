// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model.h>
#include <iDynTree/ModelLoader.h>
#include <iDynTree/URIUtils.h>

#include "cmdline.h"

#include <cstdlib>
#include <iostream>

/**
 * Add the option supported by the idyntree-model-info utility
 */
void addOptions(cmdline::parser& cmd)
{
    cmd.add<std::string>("model", 'm', "Model to load.", true);

    cmd.add("print", 'p', "Print the model.");

    cmd.add("total-mass", '\0', "Get the total mass of a model.");

    cmd.add<std::string>("link-com", '\0', "Print inertial information of a given link", false);

    cmd.add<std::string>("link-com-frame",
                         '\0',
                         "If link-com is passed, link-com-frame specifies the frame in which the "
                         "inertia information is printed (assuming that the model is in zero "
                         "position)",
                         false);

    cmd.add<std::string>("frame-pose", '\0', "Print the referenceFrame_H_frame transform.", false);

    cmd.add<std::string>("frame-pose-reference-frame",
                         '\0',
                         "If frame-pose is passed, frame-pose-reference-frame specifies the "
                         "reference frame in which the frame pose is expressed (assuming that the "
                         "model is in zero position)",
                         false);
}

void handlePrintOption(iDynTree::KinDynComputations& comp, cmdline::parser& cmd)
{
    if (!cmd.exist("print"))
    {
        return;
    }

    std::cout << comp.getRobotModel().toString() << std::endl;
}

void handleTotalMassOptions(iDynTree::KinDynComputations& comp, cmdline::parser& cmd)
{
    if (!cmd.exist("total-mass"))
    {
        return;
    }

    double totalMass = 0.0;

    const iDynTree::Model& model = comp.getRobotModel();

    for (size_t l = 0; l < model.getNrOfLinks(); l++)
    {
        totalMass += model.getLink(l)->getInertia().getMass();
    }

    std::cout << "The total mass of model is " << totalMass << " Kg." << std::endl;
}

void handleFramePoseOptions(iDynTree::KinDynComputations& comp, cmdline::parser& cmd)
{
    using namespace iDynTree;

    const Model& model = comp.getRobotModel();

    if (!cmd.exist("frame-pose"))
    {
        return;
    }

    std::string frameName = cmd.get<std::string>("frame-pose");

    std::string referenceFrameName;
    if (cmd.exist("frame-pose-reference-frame"))
    {
        referenceFrameName = cmd.get<std::string>("frame-pose-reference-frame");
    } else
    {
        referenceFrameName = model.getFrameName(comp.getRobotModel().getDefaultBaseLink());
    }

    if (!model.isValidFrameIndex(model.getFrameIndex(frameName)))
    {
        std::cerr << "Frame " << frameName << " not found in the model" << std::endl;
        return;
    }

    if (!model.isValidFrameIndex(model.getFrameIndex(referenceFrameName)))
    {
        std::cerr << "Frame " << referenceFrameName << " not found in the model" << std::endl;
        return;
    }

    Transform referenceFrame_H_name = comp.getRelativeTransform(referenceFrameName, frameName);

    std::cout << referenceFrameName << "_H_" << frameName << " is \n"
              << referenceFrame_H_name.asHomogeneousTransform().toString() << std::endl;
}

void handleLinkCOMOptions(iDynTree::KinDynComputations& comp, cmdline::parser& cmd)
{
    using namespace iDynTree;

    if (!cmd.exist("link-com"))
    {
        return;
    }

    std::string linkComName = cmd.get<std::string>("link-com");

    std::string comFrameName;
    if (cmd.exist("link-com-frame"))
    {
        comFrameName = cmd.get<std::string>("link-com-frame");
    } else
    {
        comFrameName = linkComName;
    }

    Transform frame_H_link = comp.getRelativeTransform(comFrameName, linkComName);

    const Model& model = comp.getRobotModel();
    LinkIndex linkComIndex = model.getLinkIndex(linkComName);

    if (!model.isValidLinkIndex(linkComIndex))
    {
        std::cerr << "Link " << linkComName << " not found in the model" << std::endl;
    }

    Position linkComWrtLink
        = comp.getRobotModel().getLink(linkComIndex)->getInertia().getCenterOfMass();

    Position linkComWrtFrame = frame_H_link * linkComWrtLink;

    std::cout << "The COM of link " << linkComName << " with respect to frame " << comFrameName
              << " is " << linkComWrtFrame.toString() << " m . " << std::endl;
}

int main(int argc, char** argv)
{
    cmdline::parser cmd;
    addOptions(cmd);
    cmd.parse_check(argc, argv);

    // Read model
    std::string modelPath = cmd.get<std::string>("model");

    // Resolve URI (package:// or model://) to local file system path
    modelPath = iDynTree::resolveURI(modelPath);

    iDynTree::ModelLoader loader;
    bool ok = loader.loadModelFromFile(modelPath);
    iDynTree::KinDynComputations model;
    if (!ok || !model.loadRobotModel(loader.model()))
    {
        std::cerr << "Impossible to read model at file " << modelPath << std::endl;
        return EXIT_FAILURE;
    }

    // Handle print option
    handlePrintOption(model, cmd);

    // Handle total mass option
    handleTotalMassOptions(model, cmd);

    // Handle link com
    handleLinkCOMOptions(model, cmd);

    // Handle frame pose
    handleFramePoseOptions(model, cmd);

    return EXIT_SUCCESS;
}
