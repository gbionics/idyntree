// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/Model.h>

// To load models from file
#include <iDynTree/ModelLoader.h>
#include <iDynTree/URIUtils.h>

// To approximate shapes of a model
#include <iDynTree/ModelTransformersSolidShapes.h>

// To write model to file
#include <iDynTree/ModelExporter.h>

// To resolve URIs
#include <iDynTree/URIUtils.h>

#include "cmdline.h"

#include <cstdlib>
#include <iostream>

/**
 * Add the option supported by the idyntree-model-simplify-shapes utility
 */
void addOptions(cmdline::parser& cmd)
{
    cmd.add<std::string>("model", 'm', "Input model to load and simplify.", true);

    cmd.add<std::string>("output-model", 'o', "Output simplified model.", true);

    // Specify which shapes need to be approximated
    cmd.add<std::string>("shapes-approximation",
                         's',
                         "Specify which shapes need to be approximated. Supported values are: "
                         "visual, collision, both.",
                         false,
                         "both");
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

    const std::string& outputModelPath = cmd.get<std::string>("output-model");
    const std::string& shapesApproximation = cmd.get<std::string>("shapes-approximation");

    // check if the shapes approximation is valid
    if (shapesApproximation != "visual" && shapesApproximation != "collision"
        && shapesApproximation != "both")
    {
        std::cerr << "Invalid shapes approximation value: " << shapesApproximation
                  << " . Supported values are: visual, collision, both." << std::endl;
        return EXIT_FAILURE;
    }

    iDynTree::ModelLoader mdlLoader;
    bool ok = mdlLoader.loadModelFromFile(modelPath);

    if (!ok)
    {
        std::cerr << "Impossible to read model at file " << modelPath << std::endl;
        return EXIT_FAILURE;
    }

    // Simplify the model
    iDynTree::ApproximateSolidShapesWithPrimitiveShapeOptions options
        = iDynTree::ApproximateSolidShapesWithPrimitiveShapeOptions();
    options.conversionType = iDynTree::ApproximateSolidShapesWithPrimitiveShapeConversionType::
        ConvertSolidShapesWithEnclosingAxisAlignedBoundingBoxes;
    if (shapesApproximation == "visual")
    {
        options.shapesToApproximate
            = iDynTree::ApproximateSolidShapesWithPrimitiveShapeShapesToApproximate::VisualShapes;
    } else if (shapesApproximation == "collision")
    {
        options.shapesToApproximate
            = iDynTree::ApproximateSolidShapesWithPrimitiveShapeShapesToApproximate::CollisionShapes;
    } else if (shapesApproximation == "both")
    {
        options.shapesToApproximate
            = iDynTree::ApproximateSolidShapesWithPrimitiveShapeShapesToApproximate::BothShapes;
    }

    iDynTree::Model simplifiedModel;
    ok = approximateSolidShapesWithPrimitiveShape(mdlLoader.model(), simplifiedModel, options);

    if (!ok)
    {
        std::cerr << "Impossible to simplify  model loaded from file " << modelPath << std::endl;
        return EXIT_FAILURE;
    }

    // Export the model to file
    iDynTree::ModelExporter mdlExporter;
    iDynTree::ModelExporterOptions exportOptions = iDynTree::ModelExporterOptions();
    ok = mdlExporter.init(simplifiedModel, iDynTree::SensorsList(), exportOptions);
    ok = ok && mdlExporter.exportModelToFile(outputModelPath);

    if (!ok)
    {
        std::cerr << "Impossible to export simplified model at file " << outputModelPath
                  << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
