// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/URIUtils.h>

#include <ResolveRoboticsURICpp.h>

namespace iDynTree
{

std::string resolveURI(const std::string& uri, const std::vector<std::string>& packageDirs)
{
    ResolveRoboticsURICpp::ResolveRoboticsURIOptions options;

    // Set the package directories if provided
    options.packageDirs = packageDirs;

    // Use ResolveRoboticsURICpp to resolve the URI
    std::string errorMessage;
    auto resolved
        = ResolveRoboticsURICpp::resolveRoboticsURI(uri, options, errorMessage);

    // Return the resolved path if successful, otherwise return the original URI
    if (resolved.has_value())
    {
        return resolved.value();
    }

    return uri;
}

} // namespace iDynTree
