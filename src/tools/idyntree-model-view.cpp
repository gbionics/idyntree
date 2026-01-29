// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/Model.h>
#include <iDynTree/ModelLoader.h>
#include <iDynTree/URIUtils.h>

#include <iDynTree/Visualizer.h>

#include "cmdline.h"

#include <cstdlib>
#include <iostream>

/**
 * Add the option supported by the idyntree-model-info utility
 */
void addOptions(cmdline::parser& cmd)
{
    cmd.add<std::string>("model", 'm', "Model to load.", true);

    cmd.add<std::string>("frames",
                         'f',
                         "Frames to visualize, defined as a single string with a \", \" as "
                         "separator. "
                         "For example \"l_sole, r_sole\".",
                         false);

    cmd.add<std::string>("color-palette", 'c', "Color palette.", false);
    cmd.add("print", 'p', "Print the model.");
}

int main(int argc, char** argv)
{
    cmdline::parser cmd;
    addOptions(cmd);
    cmd.parse_check(argc, argv);

    // Read model and resolve URI (package:// or model://) to local file system path
    std::string modelPath = cmd.get<std::string>("model");
    modelPath = iDynTree::resolveURI(modelPath);

    iDynTree::ModelLoader mdlLoader;
    bool ok = mdlLoader.loadModelFromFile(modelPath);

    if (!ok)
    {
        std::cerr << "Impossible to read model at file " << modelPath << std::endl;
        return EXIT_FAILURE;
    }

    if (cmd.exist("print"))
    {
        std::cout << mdlLoader.model().toString() << std::endl;
    }

    // Visualize the model
    iDynTree::Visualizer visualizer;
    ok = visualizer.addModel(mdlLoader.model(), "model");
    const std::string& palette = cmd.get<std::string>("color-palette");
    if (!palette.empty())
    {
        ok = visualizer.setColorPalette(palette);
        if (!ok)
        {
            std::cerr << "Impossible to set the color palette." << std::endl;
            return EXIT_FAILURE;
        }
    }

    std::string frames = cmd.get<std::string>("frames");
    if (!frames.empty())
    {
        std::vector<std::string> framesList;
        std::string delimiter = ",";
        size_t pos = frames.find(delimiter);
        std::string token;
        while (pos != std::string::npos)
        {
            framesList.push_back(frames.substr(0, pos));
            if (pos + 1 < frames.length() && frames[pos + 1] == ' ')
            {
                pos += 1;
            }
            frames.erase(0, pos + delimiter.length());
            pos = frames.find(delimiter);
        }
        framesList.push_back(frames);

        for (const std::string& frame : framesList)
        {
            iDynTree::IFrameVisualization& frameViz = visualizer.frames();
            size_t frameIndex = frameViz.addFrame(iDynTree::Transform::Identity(), 0.2);
            ok = frameViz.setFrameParent(frameIndex, "model", frame);
            if (!ok)
            {
                std::cerr << "Impossible to add frame " << frame << std::endl;
                return EXIT_FAILURE;
            }
            iDynTree::ILabel* label = frameViz.getFrameLabel(frameIndex);
            label->setText(frame);
            label->setPosition(iDynTree::Position(0.1, 0.1, -0.01));
        }
    }

    visualizer.camera().animator()->enableMouseControl();

    if (!ok)
    {
        std::cerr << "Impossible to read visualizer." << std::endl;
        return EXIT_FAILURE;
    }

    // Visualization loop
    while (visualizer.run())
    {
        visualizer.draw();
    }

    return EXIT_SUCCESS;
}
