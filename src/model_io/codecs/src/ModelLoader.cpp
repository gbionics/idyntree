// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "iDynTree/ModelLoader.h"

#include "ModelIOFormatUtils.h"
#include "SDFormatDocument.h"
#include "URDFDocument.h"

#ifdef IDYNTREE_USES_NLOHMANN_JSON
#include "ModelJSONImportExport.h"
#endif

#include <iDynTree/ModelTransformers.h>
#include <iDynTree/XMLParser.h>

#include <string>
#include <vector>

namespace iDynTree
{

ModelParserOptions::ModelParserOptions()
    : addSensorFramesAsAdditionalFrames(true)
    , originalFilename("")
    , convertThreeRevoluteJointsToSphericalJoint(false)
    , sphericalJointZeroMassTolerance(1e-6)
    , sphericalJointOrthogonalityTolerance(1e-6)
    , sphericalJointIntersectionTolerance(1e-6)
{
}

class ModelLoader::ModelLoaderPimpl
{
public:
    Model m_model;
    bool m_isModelValid;
    ModelParserOptions m_options;

    bool setModel(const Model& _model);
};

bool ModelLoader::ModelLoaderPimpl::setModel(const Model& _model)
{
    m_model = _model;

    // TODO \todo add a self consistency check of model/sensors
    m_isModelValid = true;

    if (!m_isModelValid)
    {
        reportError("ModelLoader",
                    "setModel",
                    "Loading failed, resetting ModelLoader to be invalid");
        m_model = Model();
    }

    return m_isModelValid;
}

ModelLoader::ModelLoader()
    : m_pimpl(new ModelLoaderPimpl())
{
    m_pimpl->m_isModelValid = false;
}

ModelLoader::~ModelLoader()
{
}

const Model& ModelLoader::model()
{
    return m_pimpl->m_model;
}

const SensorsList& ModelLoader::sensors()
{
    return m_pimpl->m_model.sensors();
}

bool ModelLoader::isValid()
{
    return m_pimpl->m_isModelValid;
}

const ModelParserOptions& ModelLoader::parsingOptions() const
{
    return m_pimpl->m_options;
}

void ModelLoader::setParsingOptions(const ModelParserOptions& options)
{
    m_pimpl->m_options = options;
}

bool ModelLoader::loadModelFromFile(const std::string& filename,
                                    const std::string& filetype,
                                    const std::vector<std::string>& packageDirs /* = {} */)
{
    std::string actualFileType = normalizeModelFormatName(filetype);
    if (actualFileType.empty())
    {
        actualFileType = inferModelFormatFromFilename(filename);
    }

    if (!actualFileType.empty() && !isSupportedImportModelFormat(actualFileType))
    {
        reportError("ModelLoader",
                    "loadModelFromFile",
                    "Unsupported filetype. Supported import formats are: urdf, sdf, "
                    "idyntree-model-json.");
        return false;
    }

    if (actualFileType == "idyntree-model-json")
    {
#ifdef IDYNTREE_USES_NLOHMANN_JSON
        Model parsedModel;
        if (!modelFromJSONFile(filename, parsedModel))
        {
            reportError("ModelLoader",
                        "loadModelFromFile",
                        "Error in parsing model from idyntree-model-json file.");
            return false;
        }
        return m_pimpl->setModel(parsedModel);
#else
        reportError("ModelLoader",
                    "loadModelFromFile",
                    "idyntree-model-json format detected but iDynTree was not compiled with "
                    "nlohmann_json support. "
                    "Please rebuild iDynTree with IDYNTREE_USES_NLOHMANN_JSON option enabled.");
        return false;
#endif
    }

    if (actualFileType == "sdf")
    {
#ifdef IDYNTREE_USES_SDFORMAT
        // Use SDFormat parser
        auto parserOptions = this->m_pimpl->m_options;
        auto sdfDocument = std::make_shared<SDFormatDocument>(parserOptions);

        if (!sdfDocument->loadFromFile(filename, packageDirs))
        {
            reportError("ModelLoader",
                        "loadModelFromFile",
                        "Error in parsing model from SDF file.");
            return false;
        }

        return m_pimpl->setModel(sdfDocument->model());
#else
        reportError("ModelLoader",
                    "loadModelFromFile",
                    "SDF file format detected but iDynTree was not compiled with "
                    "SDFormat support. "
                    "Please rebuild iDynTree with IDYNTREE_USES_SDFORMAT option "
                    "enabled.");
        return false;
#endif
    }

    // Default to URDF parsing
    // Allocate parser
    std::shared_ptr<XMLParser> parser = std::make_shared<XMLParser>();
    auto parserOptions = this->m_pimpl->m_options;
    auto documentFactoryWithOptions = [parserOptions](XMLParserState& state) {
        return std::shared_ptr<XMLDocument>(new URDFDocument(state, parserOptions));
    };
    parser->setDocumentFactory(documentFactoryWithOptions);
    parser->setPackageDirs(packageDirs);
    if (!parser->parseXMLFile(filename))
    {
        reportError("ModelLoader", "loadModelFromFile", "Error in parsing model from URDF.");
        return false;
    }
    // Retrieving the parsed document, which is an instance of URDFDocument
    std::shared_ptr<const XMLDocument> document = parser->document();
    std::shared_ptr<const URDFDocument> urdfDocument
        = std::dynamic_pointer_cast<const URDFDocument>(document);
    if (!urdfDocument)
    {
        reportError("ModelLoader",
                    "loadModelFromFile",
                    "Fatal error in retrieving the parsed model.");
        return false;
    }

    return m_pimpl->setModel(urdfDocument->model());
}

bool ModelLoader::loadModelFromString(const std::string& modelString,
                                      const std::string& filetype,
                                      const std::vector<std::string>& packageDirs /* = {} */)
{
    const std::string normalizedFiletype = normalizeModelFormatName(filetype);

    if (!normalizedFiletype.empty() && !isSupportedImportModelFormat(normalizedFiletype))
    {
        reportError("ModelLoader",
                    "loadModelFromString",
                    "Unsupported filetype. Supported import formats are: urdf, sdf, "
                    "idyntree-model-json.");
        return false;
    }

    // Check if this is an SDF string (look for <sdf> tag)
    bool isSDF = false;
    if (normalizedFiletype == "sdf")
    {
        isSDF = true;
    } else if (normalizedFiletype.empty())
    {
        // Try to detect from content
        size_t sdfPos = modelString.find("<sdf");
        if (sdfPos != std::string::npos && sdfPos < 1000) // Check in first 1000 chars
        {
            isSDF = true;
        }
    }

    if (normalizedFiletype == "idyntree-model-json")
    {
#ifdef IDYNTREE_USES_NLOHMANN_JSON
        Model parsedModel;
        if (!modelFromJSONString(modelString, parsedModel))
        {
            reportError("ModelLoader",
                        "loadModelFromString",
                        "Error in parsing model from idyntree-model-json string.");
            return false;
        }
        return m_pimpl->setModel(parsedModel);
#else
        reportError("ModelLoader",
                    "loadModelFromString",
                    "idyntree-model-json format requested but iDynTree was not compiled with "
                    "nlohmann_json support. "
                    "Please rebuild iDynTree with IDYNTREE_USES_NLOHMANN_JSON option enabled.");
        return false;
#endif
    }

    if (isSDF)
    {
#ifdef IDYNTREE_USES_SDFORMAT
        // Use SDFormat parser
        auto parserOptions = this->m_pimpl->m_options;
        auto sdfDocument = std::make_shared<SDFormatDocument>(parserOptions);

        if (!sdfDocument->loadFromString(modelString, packageDirs))
        {
            reportError("ModelLoader",
                        "loadModelFromString",
                        "Error in parsing model from SDF string.");
            return false;
        }

        return m_pimpl->setModel(sdfDocument->model());
#else
        reportError("ModelLoader",
                    "loadModelFromString",
                    "SDF format detected but iDynTree was not compiled with "
                    "SDFormat support. "
                    "Please rebuild iDynTree with IDYNTREE_USES_SDFORMAT option "
                    "enabled.");
        return false;
#endif
    }

    // Default to URDF parsing
    // Allocate parser
    std::shared_ptr<XMLParser> parser = std::make_shared<XMLParser>();
    auto parserOptions = this->m_pimpl->m_options;
    auto documentFactoryWithOptions = [parserOptions](XMLParserState& state) {
        return std::shared_ptr<XMLDocument>(new URDFDocument(state, parserOptions));
    };

    parser->setDocumentFactory(documentFactoryWithOptions);
    parser->setPackageDirs(packageDirs);
    if (!parser->parseXMLString(modelString))
    {
        reportError("ModelLoader", "loadModelFromString", "Error in parsing model from URDF.");
        return false;
    }
    // Retrieving the parsed document, which is an instance of URDFDocument
    std::shared_ptr<const XMLDocument> document = parser->document();
    std::shared_ptr<const URDFDocument> urdfDocument
        = std::dynamic_pointer_cast<const URDFDocument>(document);
    if (!urdfDocument)
    {
        reportError("ModelLoader",
                    "loadModelFromString",
                    "Fatal error in retrieving the parsed model.");
        return false;
    }

    return m_pimpl->setModel(urdfDocument->model());
}

bool ModelLoader::loadReducedModelFromFullModel(const Model& fullModel,
                                                const std::vector<std::string>& consideredJoints,
                                                const std::string filetype)
{
    std::unordered_map<std::string, std::vector<double>> emptyRemovedJointPositions;
    return this->loadReducedModelFromFullModel(fullModel,
                                               consideredJoints,
                                               emptyRemovedJointPositions,
                                               filetype);
}

bool ModelLoader::loadReducedModelFromString(const std::string modelString,
                                             const std::vector<std::string>& consideredJoints,
                                             const std::string filetype,
                                             const std::vector<std::string>& packageDirs /*= {}*/)
{
    std::unordered_map<std::string, std::vector<double>> emptyRemovedJointPositions;
    return this->loadReducedModelFromString(modelString,
                                            consideredJoints,
                                            emptyRemovedJointPositions,
                                            filetype,
                                            packageDirs);
}

bool ModelLoader::loadReducedModelFromFile(const std::string filename,
                                           const std::vector<std::string>& consideredJoints,
                                           const std::string filetype,
                                           const std::vector<std::string>& packageDirs /*= {}*/)
{
    std::unordered_map<std::string, std::vector<double>> emptyRemovedJointPositions;
    return this->loadReducedModelFromFile(filename,
                                          consideredJoints,
                                          emptyRemovedJointPositions,
                                          filetype,
                                          packageDirs);
}

bool ModelLoader::loadReducedModelFromFullModel(
    const Model& fullModel,
    const std::vector<std::string>& consideredJoints,
    const std::unordered_map<std::string, std::vector<double>>& removedJointPositions,
    const std::string /*filetype*/)
{
    Model _modelReduced;
    _modelReduced.setPackageDirs(fullModel.getPackageDirs());
    bool ok = createReducedModel(fullModel, consideredJoints, _modelReduced, removedJointPositions);

    if (!ok)
    {
        return false;
    }

    return m_pimpl->setModel(_modelReduced);
}

bool ModelLoader::loadReducedModelFromFullModel(
    const Model& fullModel,
    const std::vector<std::string>& consideredJoints,
    const std::unordered_map<std::string, double>& removedJointPositions,
    const std::string filetype)
{
    // Convert the double-based map to vector-based map for backward compatibility
    std::unordered_map<std::string, std::vector<double>> removedJointPositionsVector;
    for (const auto& pair : removedJointPositions)
    {
        removedJointPositionsVector[pair.first] = {pair.second};
    }

    return loadReducedModelFromFullModel(fullModel,
                                         consideredJoints,
                                         removedJointPositionsVector,
                                         filetype);
}

bool ModelLoader::loadReducedModelFromString(
    const std::string modelString,
    const std::vector<std::string>& consideredJoints,
    const std::unordered_map<std::string, std::vector<double>>& removedJointPositions,
    const std::string filetype,
    const std::vector<std::string>& packageDirs /*= {}*/)
{
    bool parsingCorrect = loadModelFromString(modelString, filetype, packageDirs);
    if (!parsingCorrect)
        return false;
    Model _modelFull = m_pimpl->m_model, _modelReduced;
    _modelReduced.setPackageDirs(packageDirs);

    parsingCorrect
        = createReducedModel(_modelFull, consideredJoints, _modelReduced, removedJointPositions);

    if (!parsingCorrect)
    {
        return false;
    }

    return m_pimpl->setModel(_modelReduced);
}

bool ModelLoader::loadReducedModelFromString(
    const std::string modelString,
    const std::vector<std::string>& consideredJoints,
    const std::unordered_map<std::string, double>& removedJointPositions,
    const std::string filetype,
    const std::vector<std::string>& packageDirs /*= {}*/)
{
    // Convert the double-based map to vector-based map for backward compatibility
    std::unordered_map<std::string, std::vector<double>> removedJointPositionsVector;
    for (const auto& pair : removedJointPositions)
    {
        removedJointPositionsVector[pair.first] = {pair.second};
    }

    return loadReducedModelFromString(modelString,
                                      consideredJoints,
                                      removedJointPositionsVector,
                                      filetype,
                                      packageDirs);
}

bool ModelLoader::loadReducedModelFromFile(
    const std::string filename,
    const std::vector<std::string>& consideredJoints,
    const std::unordered_map<std::string, std::vector<double>>& removedJointPositions,
    const std::string filetype,
    const std::vector<std::string>& packageDirs /*= {}*/)
{
    bool parsingCorrect = loadModelFromFile(filename, filetype, packageDirs);
    if (!parsingCorrect)
        return false;
    Model _modelFull = m_pimpl->m_model, _modelReduced;
    _modelReduced.setPackageDirs(packageDirs);

    parsingCorrect
        = createReducedModel(_modelFull, consideredJoints, _modelReduced, removedJointPositions);

    if (!parsingCorrect)
    {
        return false;
    }

    return m_pimpl->setModel(_modelReduced);
}

bool ModelLoader::loadReducedModelFromFile(
    const std::string filename,
    const std::vector<std::string>& consideredJoints,
    const std::unordered_map<std::string, double>& removedJointPositions,
    const std::string filetype,
    const std::vector<std::string>& packageDirs /*= {}*/)
{
    // Convert the double-based map to vector-based map for backward compatibility
    std::unordered_map<std::string, std::vector<double>> removedJointPositionsVector;
    for (const auto& pair : removedJointPositions)
    {
        removedJointPositionsVector[pair.first] = {pair.second};
    }

    return loadReducedModelFromFile(filename,
                                    consideredJoints,
                                    removedJointPositionsVector,
                                    filetype,
                                    packageDirs);
}
} // namespace iDynTree
