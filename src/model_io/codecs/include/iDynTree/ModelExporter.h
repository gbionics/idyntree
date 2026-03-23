// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_MODEL_EXPORTER_H
#define IDYNTREE_MODEL_EXPORTER_H

#include <iDynTree/Model.h>
#include <iDynTree/Sensors.h>

#include <memory>
#include <string>
#include <vector>

namespace iDynTree
{

/**
 * \ingroup iDynTreeModelIO
 *
 * Options for the iDynTree exporter.
 */
struct ModelExporterOptions
{
    /**
     * Specify the base link of the exported model.
     *
     * Differently from the iDynTree::Model class, some file formats (such as the URDF) represent
     * the multibody structure using a directed tree representation, for which it is necessary to
     * explicitly specify a base link.
     *
     * If this string is empty (default value), the default base link contained in the model will be
     * used.
     *
     * Default value: "".
     * Supported formats: urdf.
     */
    std::string baseLink;

    /**
     * Select if the first additional frame of the base link is exported as fake base link.
     *
     * The URDF exporter by default exports the first additional frame of the base link as
     * a parent "fake" link to the actual base link, as a workaround for
     * https://github.com/ros/kdl_parser/issues/27). By setting this option to false, is possible to
     * disable this behaviour, for more info see iDynTree::ModelExporter docs. This option is
     * ignored in non-URDF exporter.
     *
     * Default value: true.
     * Supported formats: urdf.
     */
    bool exportFirstBaseLinkAdditionalFrameAsFakeURDFBase;

    /**
     * Specify the robot name.
     *
     * Default: "iDynTreeURDFModelExportModelName".
     * Supported formats: urdf.
     */
    std::string robotExportedName;
    /**
     * Specify the xml blobs to be exported at the end of the urdf as child of the robot tag.
     *
     * Default: {}.
     * Supported formats: urdf.
     */
    std::vector<std::string> xmlBlobs;

    /**
     * If true, spherical joints are exported as a series of three revolute joints
     * with fake intermediate links having zero mass/inertia. If false,
     * the spherical joint exported as a single joint (which is not supported in URDF).
     * Default: true
     * Supported formats: urdf.
     */
    bool exportSphericalJointsAsThreeRevoluteJoints;

    /**
     * Prefix used for fake links created when exporting spherical joints.
     * Default: "spherical_fake_"
     * Supported formats: urdf.
     */
    std::string sphericalJointFakeLinkPrefix;

    /**
     * Prefix used for revolute joints created when exporting spherical joints.
     * Default: "spherical_rev_"
     * Supported formats: urdf.
     */
    std::string sphericalJointRevoluteJointPrefix;

    /**
     * Numerical precision for floating-point values in exported URDF.
     * If greater then 0, specify the precision (i.e. how many digits are generated) of floating
     * point output when converting doubles to strings. Default: 0, i.e. do not use a fixed number
     * of digits, but use the minimum number of digits required to represent exactly the floating
     * point value
     */
    int numericalPrecision;

    /**
     * Constructor.
     */
    ModelExporterOptions();
};

/**
 * \ingroup iDynTreeModelIO
 *
 * Helper class to export a model to the supported textual formats.
 *
 * Supported formats for export are:
 * - URDF, as described in http://wiki.ros.org/urdf/XML
 * - iDynTree native JSON (`idyntree-model-json`)
 *
 * Only iDynTree::Model classes that represent multibody system with no loops
 * can be exported.
 *
 * Furthermore, currently the model exporter only exports a subset of the features
 * supported in iDynTree::Model. In particular, the following features are not exported:
 *
 * * Sensors
 * * Joint limits, damping and static friction.
 *
 * # Format documentation
 *
 * The following format are  supported  by the exporter.
 *
 * | Format | Extendend Name | Website |  String for filetype argument  |
 * |:-----------------------------:|:-------:|:-------:|:--------:|
 * | URDF | Unified Robot Description Format  | http://wiki.ros.org/urdf | `urdf` |
 * | idyntree-model-json | iDynTree native JSON format | N/A | `idyntree-model-json` |
 *
 * ## URDF
 *
 * As the URDF format does not distinguish between frames and links (see
 * https://discourse.ros.org/t/urdf-ng-link-and-frame-concepts/56 for an extensive discussion on
 * this) the URDF model exporter converts iDynTree's *additional frames* to mass-less and shape-less
 * *fake* URDF links that are connected as child links via `fixed` joints to the corresponding
 * **real** URDF links.
 *
 * Furthermore, it is widespread use in URDF models to never use a real link (with mass) as the root
 * link of a model, mainly due to workaround a bug in official %KDL parser used in ROS (see
 * https://github.com/ros/kdl_parser/issues/27 for more info). For this reason, if the selected
 * base_link has at least one additional frame, by default the first additional frame of the base
 * link is added as a **parent** fake URDF link, instead as a **child** fake URDF link as done with
 * the rest of %iDynTree's additional frames. If no additional frame is available for the base link,
 * the base link of the URDF will have a mass, and will generate a warning then used with the ROS's
 * [`kdl_parser`](https://github.com/ros/kdl_parser) . This behaviour can be disabled by setting to
 * false the `exportFirstBaseLinkAdditionalFrameAsFakeURDFBase` attribute of ModelExporterOptions.
 *
 */
class ModelExporter
{
private:
    class Pimpl;
    std::unique_ptr<Pimpl> m_pimpl;

public:
    /**
     * @name Constructor/Destructor
     */
    //@{

    /**
     * Constructor
     */
    ModelExporter();

    ~ModelExporter();

    //@}

    /**
     * @name Model exporting and definition methods
     * This methods are used to export the structure of your model.
     */
    //@{
    const ModelExporterOptions& exportingOptions() const;

    void setExportingOptions(const ModelExporterOptions& options);

    /**
     * Specifies the model of the robot to export.
     *
     * @param[in] model The used model.
     * @param[in] options The used options.
     * @return true if all went well, false otherwise.
     */
    bool init(const Model& model, const ModelExporterOptions options = ModelExporterOptions());

    /**
     * Specifies the model of the robot to export.
     *
     * @param[in] model The used model.
     * @param[in] sensors The used sensors.
     * @param[in] options The used options.
     * @return true if all went well, false otherwise.
     */
    IDYNTREE_DEPRECATED_WITH_MSG("Deprecated, please use the variant in which the sensors are "
                                 "passed as part of the iDynTree::Model.")
    bool init(const Model& model,
              const SensorsList& sensors,
              const ModelExporterOptions options = ModelExporterOptions());

    /**
     * Get the loaded model that will be exported.
     *
     */
    const Model& model();

    /**
     * Get the loaded sensors that will be exported.
     */
    IDYNTREE_DEPRECATED_WITH_MSG("Deprecated, please use ModelExporter::model::sensors method.")
    const SensorsList& sensors();

    /**
     * Return true if the model have been correctly loaded, and can be exported.
     *
     * @return True if the model was loaded correctly.
     */
    bool isValid();

    /**
     * Export the model of the robot to a string.
     *
     * @param modelString string containg the model of the robot.
     * @param filetype type of the model format to export.
     *                 Supported values: `urdf`, `idyntree-model-json`.
     *
     */
    bool exportModelToString(std::string& modelString, const std::string filetype = "urdf");

    /**
     * Export the model of the robot to an external file.
     *
     * @param filename path to the file to export.
     *                 It can be either a relative filename with respect to the current working
     *                 directory, or an absolute filename.
     * @param filetype type of the model format to export.
     *                 Supported values: `urdf`, `idyntree-model-json`.
     *
     */
    bool exportModelToFile(const std::string& filename, const std::string filetype = "urdf");
    //@}
};

} // namespace iDynTree

#endif
