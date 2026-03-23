// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_MODEL_LOADER_H
#define IDYNTREE_MODEL_LOADER_H

#include <iDynTree/Model.h>
#include <iDynTree/Sensors.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace iDynTree
{

/**
 * \ingroup iDynTreeModelIO
 *
 * Options for the iDynTree parser.
 */
struct ModelParserOptions
{
    // TODO: migrate to set/get
    // ???: what about adding a search dir instead of a filename?
public:
    /**
     * If true, add to the model the sensor frames
     * as additional frames with the same name of the sensor.
     * If there is already a link or additional frame with the same
     * name of the sensor, a warning is printed and no frame is added.
     */
    bool addSensorFramesAsAdditionalFrames;

    /**
     * Original filename of the URDF sensor parsed.
     *
     * This attribute is the original filename of the URDF sensor parsed.
     * It is useful when loading a model from a string, if that URDF string
     * has <geometry> tags that point to external meshes. To find the location
     * of this external meshes, we need also the original filename of the URDF
     * file.
     */
    std::string originalFilename;

    /**
     * If true, three consecutive revolute joints with zero-mass intermediate
     * links and orthogonal axes intersecting at a point are converted to a single
     * spherical joint. Default: true
     */
    bool convertThreeRevoluteJointsToSphericalJoint;

    /**
     * Tolerance for checking if intermediate links have zero mass when converting
     * three revolute joints to spherical joints.
     * Default: 1e-6
     */
    double sphericalJointZeroMassTolerance;

    /**
     * Tolerance for checking if revolute joint axes are orthogonal when
     * converting three revolute joints to spherical joints.
     * Default: 1e-6
     */
    double sphericalJointOrthogonalityTolerance;

    /**
     * Tolerance for checking if revolute joint axes intersect at a point when
     * converting three revolute joints to spherical joints.
     * Default: 1e-6
     */
    double sphericalJointIntersectionTolerance;

    /** Default options
     *
     * - addSensorFramesAsAdditionalFrames = True
     * - originalFilename = empty string
     */
    ModelParserOptions();
};

/**
 * \ingroup iDynTreeModelIO
 *
 * Helper class to load a model from a generic format.
 *
 * Unless the methods for loading a model with an explicit serialization are
 * used, the default joint serialization of the model loaded will be a
 * "normalized" joint serialization based on the default base link, see the
 * iDynTree::createModelWithNormalizedJointNumbering function for more details.
 */
class ModelLoader
{
private:
    class ModelLoaderPimpl;
    std::unique_ptr<ModelLoaderPimpl> m_pimpl;

public:
    /**
     * @name Constructor/Destructor
     */
    //@{

    /**
     * Constructor
     *
     */
    ModelLoader();

    ~ModelLoader();

    //@}

    /**
     * @name Model loading and definition methods
     * This methods are used to load the structure of your model.
     */
    //@{
    const ModelParserOptions& parsingOptions() const;

    void setParsingOptions(const ModelParserOptions& options);

    /**
     * Load the model of the robot  from a string.
     *
     * @param modelString string containg the model of the robot.
     * @param filetype type of the model to load. Supported values are:
     *                 `urdf`, `sdf`, `idyntree-model-json`.
     *                 If empty, the format is auto-detected from content when possible.
     * @param packageDirs a vector containing the different directories where to
     *                   search for model meshes
     * @note In case no package is specified ModelLoader will look for the meshes
     *       in `GZ_SIM_RESOURCE_PATH`, `GAZEBO_MODEL_PATH`, `ROS_PACKAGE_PATH` and
     *       `AMENT_PREFIX_PATH`
     * @note If a given model searches for the meshes in
     *       `package://StrangeModel/Nested/mesh.stl`, and the actual mesh is in
     *       `/usr/local/share/StrangeModel/Nested/mesh.stl`, `packageDirs` should
     *       contain `/usr/local/share`.
     */
    bool loadModelFromString(const std::string& modelString,
                             const std::string& filetype = "",
                             const std::vector<std::string>& packageDirs = {});

    /**
     * Load the model of the robot from an external file.
     *
     * @param filename path to the file to load
     * @param filetype type of the model to load. Supported values are:
     *                 `urdf`, `sdf`, `idyntree-model-json`.
     *                 If empty, the format is auto-detected from file extension
     *                 (`.urdf` -> `urdf`, `.sdf`/`.world` -> `sdf`, `.json` ->
     * `idyntree-model-json`).
     * @param packageDirs a vector containing the different directories where to
     *                   search for model meshes
     * @note In case no package is specified ModelLoader will look for the meshes
     *       in `GZ_SIM_RESOURCE_PATH`, `GAZEBO_MODEL_PATH`, `ROS_PACKAGE_PATH` and
     *       `AMENT_PREFIX_PATH`
     * @note If a given model searches for the meshes in
     *       `package://StrangeModel/Nested/mesh.stl`, and the actual mesh is in
     *       `/usr/local/share/StrangeModel/Nested/mesh.stl`, `packageDirs` should
     *       contain `/usr/local/share`.
     */
    bool loadModelFromFile(const std::string& filename,
                           const std::string& filetype = "",
                           const std::vector<std::string>& packageDirs = {});

    /**
     * Load reduced model from another model, specifyng only the desired joints in
     * the model.
     *
     * All other joints will be considered to be fixed to their default position,
     * and their child links will be lumped together.
     *
     * @note the order of the degreese of freedom of the newly loaded model
     * will be the one specified by the input joints serialization, i.e.
     * consideredJoints
     *
     * @param[in] filename path to the file to load.
     * @param[in] consideredJoints list of joints to consider in the model.
     * @param[in] filetype (optional) explicit definition of the type of the
     * loaded file. Only "urdf" is supported at the moment.
     * @return true if all went well (files were correctly loaded and consistent),
     * false otherwise.
     *
     * \note Until https://github.com/robotology/idyntree/issues/132 is fixed,
     * this method does not accounts for sensors.
     *
     */
    bool loadReducedModelFromFullModel(const Model& fullModel,
                                       const std::vector<std::string>& consideredJoints,
                                       const std::string filetype = "");

    /**
     * Load reduced model from string, specifyng only the desired joints in the model.
     *
     * All other joints will be considered to be fixed to their default position,
     * and their child links will be lumped together.
     *
     * @param[in] modelString string containg the model of the robot.
     * @param[in] consideredJoints list of joints to consider in the model.
     * @param[in] filetype (optional) explicit definiton of the filetype to load.
     *                     Only "urdf" is supported at the moment.
     * @param[in] packageDirs (optional) a vector containing the different
     * directories where to search for model meshes
     * @return true if all went well (files were correctly loaded and consistent),
     * false otherwise.
     *
     * @note In case no package is specified ModelLoader will look for the meshes
     * in `GZ_SIM_RESOURCE_PATH`, `GAZEBO_MODEL_PATH`, `ROS_PACKAGE_PATH` and `AMENT_PREFIX_PATH`
     * @note If a given model searches for the meshes in
     * `package://StrangeModel/Nested/mesh.stl`, and the actual mesh is in
     * `/usr/local/share/StrangeModel/Nested/mesh.stl`, `packageDirs` should
     * contain `/usr/local/share`.
     *
     */
    bool loadReducedModelFromString(const std::string modelString,
                                    const std::vector<std::string>& consideredJoints,
                                    const std::string filetype = "",
                                    const std::vector<std::string>& packageDirs = {});

    /**
     * Load reduced model from file, specifyng only the desired joints in the model.
     *
     * All other joints will be considered to be fixed to their default position,
     * and their child links will be lumped together.
     *
     * @param[in] filename path to the file to load.
     * @param[in] consideredJoints list of joints to consider in the model.
     * @param[in] filetype (optional) explicit definiton of the filetype to load.
     *                     Only "urdf" is supported at the moment.
     * @param[in] packageDirs (optional) a vector containing the different
     * directories where to search for model meshes
     * @return true if all went well (files were correctly loaded and consistent),
     * false otherwise.
     *
     * @note In case no package is specified ModelLoader will look for the meshes
     * in `GZ_SIM_RESOURCE_PATH`, `GAZEBO_MODEL_PATH`, `ROS_PACKAGE_PATH` and `AMENT_PREFIX_PATH`
     * @note If a given model searches for the meshes in
     * `package://StrangeModel/Nested/mesh.stl`, and the actual mesh is in
     * `/usr/local/share/StrangeModel/Nested/mesh.stl`, `packageDirs` should
     * contain `/usr/local/share`.
     */
    bool loadReducedModelFromFile(const std::string filename,
                                  const std::vector<std::string>& consideredJoints,
                                  const std::string filetype = "",
                                  const std::vector<std::string>& packageDirs = {});
    /**
     * Load reduced model from another model, specifyng only the desired joints in
     * the model.
     *
     * All other joints will be considered to be fixed to their default position,
     * and their child links will be lumped together.
     *
     * @note the order of the degreese of freedom of the newly loaded model
     * will be the one specified by the input joints serialization, i.e.
     * consideredJoints
     *
     * @param[in] filename path to the file to load.
     * @param[in] consideredJoints list of joints to consider in the model.
     * @param[in] removedJointPositions map between the dof name of the dof no in
     * consideredJoints and their (fixed) position in the reduced model
     * @param[in] filetype (optional) explicit definition of the type of the
     * loaded file. Only "urdf" is supported at the moment.
     * @return true if all went well (files were correctly loaded and consistent),
     * false otherwise.
     *
     * \note Until https://github.com/robotology/idyntree/issues/132 is fixed,
     * this method does not accounts for sensors.
     *
     */
    bool loadReducedModelFromFullModel(
        const Model& fullModel,
        const std::vector<std::string>& consideredJoints,
        const std::unordered_map<std::string, std::vector<double>>& removedJointPositions,
        const std::string filetype = "");

    /**
     * Load reduced model from another model, specifyng only the desired joints in
     * the model.
     *
     * All other joints will be considered to be fixed to their default position,
     * and their child links will be lumped together.
     *
     * @note the order of the degreese of freedom of the newly loaded model
     * will be the one specified by the input joints serialization, i.e.
     * consideredJoints
     *
     * @param[in] filename path to the file to load.
     * @param[in] consideredJoints list of joints to consider in the model.
     * @param[in] removedJointPositions map between the dof name of the dof no in
     * consideredJoints and their (fixed) position in the reduced model
     * @param[in] filetype (optional) explicit definition of the type of the
     * loaded file. Only "urdf" is supported at the moment.
     * @return true if all went well (files were correctly loaded and consistent),
     * false otherwise.
     *
     * @warning This overload will not work with joints with position coordinates
     * dimension > 1. Use the std::vector<double> version for full support of all
     * joint types.
     *
     * \note Until https://github.com/robotology/idyntree/issues/132 is fixed,
     * this method does not accounts for sensors.
     *
     */
    bool loadReducedModelFromFullModel(
        const Model& fullModel,
        const std::vector<std::string>& consideredJoints,
        const std::unordered_map<std::string, double>& removedJointPositions,
        const std::string filetype = "");

    /**
     * Load reduced model from string, specifyng only the desired joints in the model.
     *
     * All other joints will be considered to be fixed to their default position,
     * and their child links will be lumped together.
     *
     * @param[in] modelString string containg the model of the robot.
     * @param[in] consideredJoints list of joints to consider in the model.
     * @param[in] removedJointPositions map between the dof name of the dof no in
     * consideredJoints and their (fixed) position in the reduced model
     * @param[in] filetype (optional) explicit definiton of the filetype to load.
     *                     Only "urdf" is supported at the moment.
     * @param[in] packageDirs (optional) a vector containing the different
     * directories where to search for model meshes
     * @return true if all went well (files were correctly loaded and consistent),
     * false otherwise.
     *
     * @note In case no package is specified ModelLoader will look for the meshes
     * in `GZ_SIM_RESOURCE_PATH`, `GAZEBO_MODEL_PATH`, `ROS_PACKAGE_PATH` and `AMENT_PREFIX_PATH`
     * @note If a given model searches for the meshes in
     * `package://StrangeModel/Nested/mesh.stl`, and the actual mesh is in
     * `/usr/local/share/StrangeModel/Nested/mesh.stl`, `packageDirs` should
     * contain `/usr/local/share`.
     *
     */
    bool loadReducedModelFromString(
        const std::string modelString,
        const std::vector<std::string>& consideredJoints,
        const std::unordered_map<std::string, std::vector<double>>& removedJointPositions,
        const std::string filetype = "",
        const std::vector<std::string>& packageDirs = {});

    /**
     * Load reduced model from string, specifyng only the desired joints in the model.
     *
     * All other joints will be considered to be fixed to their default position,
     * and their child links will be lumped together.
     *
     * @param[in] modelString string containg the model of the robot.
     * @param[in] consideredJoints list of joints to consider in the model.
     * @param[in] removedJointPositions map between the dof name of the dof no in
     * consideredJoints and their (fixed) position in the reduced model
     * @param[in] filetype (optional) explicit definiton of the filetype to load.
     *                     Only "urdf" is supported at the moment.
     * @param[in] packageDirs (optional) a vector containing the different
     * directories where to search for model meshes
     * @return true if all went well (files were correctly loaded and consistent),
     * false otherwise.
     *
     * @warning This overload will not work with joints with position coordinates
     * dimension > 1. Use the std::vector<double> version for full support of all
     * joint types.
     *
     * @note In case no package is specified ModelLoader will look for the meshes
     * in `GZ_SIM_RESOURCE_PATH`, `GAZEBO_MODEL_PATH`, `ROS_PACKAGE_PATH` and `AMENT_PREFIX_PATH`
     * @note If a given model searches for the meshes in
     * `package://StrangeModel/Nested/mesh.stl`, and the actual mesh is in
     * `/usr/local/share/StrangeModel/Nested/mesh.stl`, `packageDirs` should
     * contain `/usr/local/share`.
     *
     */
    bool
    loadReducedModelFromString(const std::string modelString,
                               const std::vector<std::string>& consideredJoints,
                               const std::unordered_map<std::string, double>& removedJointPositions,
                               const std::string filetype = "",
                               const std::vector<std::string>& packageDirs = {});

    /**
     * Load reduced model from file, specifyng only the desired joints in the model.
     *
     * All other joints will be considered to be fixed to their default position,
     * and their child links will be lumped together.
     *
     * @param[in] filename path to the file to load.
     * @param[in] consideredJoints list of joints to consider in the model.
     * @param[in] removedJointPositions map between the dof name of the dof no in
     * consideredJoints and their (fixed) position in the reduced model
     * @param[in] filetype (optional) explicit definiton of the filetype to load.
     *                     Only "urdf" is supported at the moment.
     * @param[in] packageDirs (optional) a vector containing the different
     * directories where to search for model meshes
     * @return true if all went well (files were correctly loaded and consistent),
     * false otherwise.
     *
     * @note In case no package is specified ModelLoader will look for the meshes
     * in `GZ_SIM_RESOURCE_PATH`, `GAZEBO_MODEL_PATH`, `ROS_PACKAGE_PATH` and `AMENT_PREFIX_PATH`
     * @note If a given model searches for the meshes in
     * `package://StrangeModel/Nested/mesh.stl`, and the actual mesh is in
     * `/usr/local/share/StrangeModel/Nested/mesh.stl`, `packageDirs` should
     * contain `/usr/local/share`.
     */
    bool loadReducedModelFromFile(
        const std::string filename,
        const std::vector<std::string>& consideredJoints,
        const std::unordered_map<std::string, std::vector<double>>& removedJointPositions,
        const std::string filetype = "",
        const std::vector<std::string>& packageDirs = {});

    /**
     * Load reduced model from file, specifyng only the desired joints in the
     * model.
     *
     * All other joints will be considered to be fixed to their default position,
     * and their child links will be lumped together.
     *
     * @param[in] filename path to the file to load.
     * @param[in] consideredJoints list of joints to consider in the model.
     * @param[in] removedJointPositions map between the dof name of the dof no in
     * consideredJoints and their (fixed) position in the reduced model
     * @param[in] filetype (optional) explicit definiton of the filetype to load.
     *                     Only "urdf" is supported at the moment.
     * @param[in] packageDirs (optional) a vector containing the different
     * directories where to search for model meshes
     * @return true if all went well (files were correctly loaded and consistent),
     * false otherwise.
     *
     * @warning This overload will not work with joints with position coordinates
     * dimension > 1. Use the std::vector<double> version for full support of all
     * joint types.
     *
     * @note In case no package is specified ModelLoader will look for the meshes
     * in `GZ_SIM_RESOURCE_PATH`, `GAZEBO_MODEL_PATH`, `ROS_PACKAGE_PATH` and `AMENT_PREFIX_PATH`
     * @note If a given model searches for the meshes in
     * `package://StrangeModel/Nested/mesh.stl`, and the actual mesh is in
     * `/usr/local/share/StrangeModel/Nested/mesh.stl`, `packageDirs` should
     * contain `/usr/local/share`.
     */
    bool
    loadReducedModelFromFile(const std::string filename,
                             const std::vector<std::string>& consideredJoints,
                             const std::unordered_map<std::string, double>& removedJointPositions,
                             const std::string filetype = "",
                             const std::vector<std::string>& packageDirs = {});

    /**
     * Get the loaded model.
     *
     */
    const Model& model();

    /**
     * Get the loaded sensors.
     */
    IDYNTREE_DEPRECATED_WITH_MSG("Deprecated, please use ModelLoader::model::sensors method.")
    const SensorsList& sensors();

    /**
     * Return true if the model have been correctly true.
     *
     * @return True if the model was loaded correctly.
     */
    bool isValid();
    //@}
};

} // namespace iDynTree

#endif
