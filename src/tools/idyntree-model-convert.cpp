// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/ModelExporter.h>
#include <iDynTree/ModelLoader.h>
#include <iDynTree/URIUtils.h>

#include "ModelIOFormatUtils.h"

#include "cmdline.h"

#include <cstdlib>
#include <iostream>
#include <string>

void addOptions(cmdline::parser& cmd)
{
    cmd.add<std::string>("input", 'i', "Input model path.", true);
    cmd.add<std::string>("output", 'o', "Output model path.", true);

    cmd.add<std::string>("input-format",
                         '\0',
                         "Explicit input model format (`urdf`, `sdf`, "
                         "`idyntree-model-json`). If omitted, infer from extension.",
                         false);

    cmd.add<std::string>("output-format",
                         '\0',
                         "Explicit output model format (`urdf`, `idyntree-model-json`). "
                         "If omitted, infer from extension.",
                         false);
}

int main(int argc, char** argv)
{
    cmdline::parser cmd;
    addOptions(cmd);
    cmd.parse_check(argc, argv);

    std::string inputPath = cmd.get<std::string>("input");
    std::string outputPath = cmd.get<std::string>("output");

    inputPath = iDynTree::resolveURI(inputPath);

    std::string inputFormat;
    if (cmd.exist("input-format"))
    {
        inputFormat = iDynTree::normalizeModelFormatName(cmd.get<std::string>("input-format"));
    } else
    {
        inputFormat = iDynTree::inferModelFormatFromFilename(inputPath);
    }

    std::string outputFormat;
    if (cmd.exist("output-format"))
    {
        outputFormat = iDynTree::normalizeModelFormatName(cmd.get<std::string>("output-format"));
    } else
    {
        outputFormat = iDynTree::inferModelFormatFromFilename(outputPath);
    }

    if (inputFormat.empty())
    {
        std::cerr << "Unable to infer input format from '" << inputPath
                  << "'. Please pass --input-format." << std::endl;
        return EXIT_FAILURE;
    }

    if (!iDynTree::isSupportedImportModelFormat(inputFormat))
    {
        std::cerr << "Input format '" << inputFormat
                  << "' is not supported by ModelLoader. Supported import formats are: "
                  << "urdf, sdf, idyntree-model-json." << std::endl;
        return EXIT_FAILURE;
    }

    if (outputFormat.empty())
    {
        std::cerr << "Unable to infer output format from '" << outputPath
                  << "'. Please pass --output-format." << std::endl;
        return EXIT_FAILURE;
    }

    if (!iDynTree::isSupportedExportModelFormat(outputFormat))
    {
        std::cerr << "Output format '" << outputFormat
                  << "' is not supported by ModelExporter. Supported output formats are: "
                  << "urdf, idyntree-model-json." << std::endl;
        return EXIT_FAILURE;
    }

    iDynTree::ModelLoader loader;
    if (!loader.loadModelFromFile(inputPath, inputFormat))
    {
        std::cerr << "Impossible to read model at file '" << inputPath << "'"
                  << " with input format '" << inputFormat << "'." << std::endl;
        return EXIT_FAILURE;
    }

    iDynTree::ModelExporter exporter;
    if (!exporter.init(loader.model()) || !exporter.exportModelToFile(outputPath, outputFormat))
    {
        std::cerr << "Impossible to export model to file '" << outputPath << "'"
                  << " with output format '" << outputFormat << "'." << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "Converted model from '" << inputPath << "' (" << inputFormat << ")"
              << " to '" << outputPath << "' (" << outputFormat << ")." << std::endl;

    return EXIT_SUCCESS;
}
