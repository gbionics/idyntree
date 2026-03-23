// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "iDynTree/ModelExporter.h"

#include "ModelIOFormatUtils.h"
#include "URDFModelExport.h"

#ifdef IDYNTREE_USES_NLOHMANN_JSON
#include "ModelJSONImportExport.h"
#endif

#include <string>
#include <vector>

namespace iDynTree
{

ModelExporterOptions::ModelExporterOptions()
    : baseLink("")
    , exportFirstBaseLinkAdditionalFrameAsFakeURDFBase(true)
    , robotExportedName("iDynTreeURDFModelExportModelName")
    , xmlBlobs{}
    , exportSphericalJointsAsThreeRevoluteJoints(true)
    , sphericalJointFakeLinkPrefix("spherical_fake_")
    , sphericalJointRevoluteJointPrefix("spherical_rev_")
    , numericalPrecision(0)
{
}

class ModelExporter::Pimpl
{
public:
    Model m_model;
    bool m_isModelValid;
    ModelExporterOptions m_options;

    bool setModel(const Model& _model);
};

bool ModelExporter::Pimpl::setModel(const Model& _model)
{
    m_model = _model;

    m_isModelValid = true;

    return true;
}

ModelExporter::ModelExporter()
    : m_pimpl(new Pimpl())
{
    m_pimpl->m_isModelValid = false;
}

ModelExporter::~ModelExporter()
{
}

const Model& ModelExporter::model()
{
    return m_pimpl->m_model;
}

const SensorsList& ModelExporter::sensors()
{
    return m_pimpl->m_model.sensors();
}

bool ModelExporter::isValid()
{
    return m_pimpl->m_isModelValid;
}

const ModelExporterOptions& ModelExporter::exportingOptions() const
{
    return m_pimpl->m_options;
}

void ModelExporter::setExportingOptions(const ModelExporterOptions& options)
{
    m_pimpl->m_options = options;
}

bool ModelExporter::init(const Model& model, const ModelExporterOptions options)
{
    m_pimpl->m_options = options;
    return m_pimpl->setModel(model);
}

bool ModelExporter::init(const Model& model,
                         const SensorsList& sensors,
                         const ModelExporterOptions options)
{
    Model modelCopy = model;
    modelCopy.sensors() = sensors;
    return init(modelCopy, options);
}

bool ModelExporter::exportModelToString(std::string& modelString, const std::string filetype)
{
    const std::string normalizedFiletype = normalizeModelFormatName(filetype);

    if (normalizedFiletype == "urdf")
    {
        return URDFStringFromModel(m_pimpl->m_model, modelString, m_pimpl->m_options);
    }

    if (normalizedFiletype == "idyntree-model-json")
    {
#ifdef IDYNTREE_USES_NLOHMANN_JSON
        return modelToJSONString(m_pimpl->m_model, modelString);
#else
        reportError("ModelExporter",
                    "exportModelToString",
                    "idyntree-model-json format requested but iDynTree was not compiled with "
                    "nlohmann_json support. "
                    "Please rebuild iDynTree with IDYNTREE_USES_NLOHMANN_JSON option enabled.");
        return false;
#endif
    }

    std::stringstream error_msg;
    error_msg << "Filetype " << filetype
              << " not supported. Supported export formats: urdf, idyntree-model-json.";
    reportError("ModelExporter", "exportModelToString", error_msg.str().c_str());
    return false;
}

/**
 * Export the model of the robot to an external file.
 *
 * @param filename path to the file to export
 * @param filetype type of the file to load, currently supporting only urdf type.
 *
 */
bool ModelExporter::exportModelToFile(const std::string& filename, const std::string filetype)
{
    const std::string normalizedFiletype = normalizeModelFormatName(filetype);

    if (normalizedFiletype == "urdf")
    {
        return URDFFromModel(m_pimpl->m_model, filename, m_pimpl->m_options);
    }

    if (normalizedFiletype == "idyntree-model-json")
    {
#ifdef IDYNTREE_USES_NLOHMANN_JSON
        return modelToJSONFile(m_pimpl->m_model, filename);
#else
        reportError("ModelExporter",
                    "exportModelToFile",
                    "idyntree-model-json format requested but iDynTree was not compiled with "
                    "nlohmann_json support. "
                    "Please rebuild iDynTree with IDYNTREE_USES_NLOHMANN_JSON option enabled.");
        return false;
#endif
    }

    std::stringstream error_msg;
    error_msg << "Filetype " << filetype
              << " not supported. Supported export formats: urdf, idyntree-model-json.";
    reportError("ModelExporter", "exportModelToFile", error_msg.str().c_str());
    return false;
}

} // namespace iDynTree
