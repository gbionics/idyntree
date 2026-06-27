// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_URI_UTILS_H
#define IDYNTREE_URI_UTILS_H

#include <string>
#include <vector>

namespace iDynTree
{

/**
 * Resolve a URI (such as package:// or model://) to an absolute path on the local file system.
 *
 * This function resolves URIs with package:// or model:// prefixes by searching for the file
 * using the paths specified in the packageDirs vector or, if empty, in various environment
 * variables including "GZ_SIM_RESOURCE_PATH", "GAZEBO_MODEL_PATH", "ROS_PACKAGE_PATH",
 * "AMENT_PREFIX_PATH", and others. Additionally, it automatically searches in the active
 * conda/virtual environment prefix if available.
 * See https://github.com/gbionics/resolve-robotics-uri-cpp/blob/main/rru_spec.md for more details.
 *
 * @param uri The URI to resolve (e.g., "package://MyPackage/models/model.urdf")
 * @param packageDirs Optional vector of directories to search for packages. If empty,
 *                    environment variables and active environment prefix will be used.
 * @return The absolute path to the file on the local file system. If the file cannot be
 *         resolved, the original URI is returned.
 *
 * Example:
 * If the URI is "package://iCub/robots/iCubGazeboV3/model.urdf" and the iCub package is found
 * in "/usr/local/share/iCub", the function will return:
 * "/usr/local/share/iCub/robots/iCubGazeboV3/model.urdf"
 */
std::string resolveURI(const std::string& uri, const std::vector<std::string>& packageDirs = {});

} // namespace iDynTree

#endif // IDYNTREE_URI_UTILS_H
