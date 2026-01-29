// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/URIUtils.h>

#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <unordered_set>

namespace iDynTree
{

std::string resolveURI(const std::string& uri, const std::vector<std::string>& packageDirs)
{
    bool isWindows = false;
#ifdef _WIN32
    isWindows = true;
#endif

    std::unordered_set<std::string> pathList;

    // If the user provides the packageDirs, use them instead of environment variables
    if (!packageDirs.empty())
    {
        pathList = std::unordered_set<std::string>(packageDirs.begin(), packageDirs.end());
    } else
    {
        // List of variables that contain <prefix>/share paths
        std::vector<std::string> envListShare
            = {"GZ_SIM_RESOURCE_PATH", "GAZEBO_MODEL_PATH", "ROS_PACKAGE_PATH"};
        // List of variables that contains <prefix> paths (to which /share needs to be added)
        std::vector<std::string> envListPrefix = {"AMENT_PREFIX_PATH"};

        for (size_t i = 0; i < envListShare.size(); ++i)
        {
            const char* env_var_value = std::getenv(envListShare[i].c_str());

            if (env_var_value)
            {
                std::stringstream env_var_string(env_var_value);
                std::string individualPath;

                while (std::getline(env_var_string, individualPath, isWindows ? ';' : ':'))
                {
                    pathList.insert(individualPath);
                }
            }
        }

        for (size_t i = 0; i < envListPrefix.size(); ++i)
        {
            const char* env_var_value = std::getenv(envListPrefix[i].c_str());

            if (env_var_value)
            {
                std::stringstream env_var_string(env_var_value);
                std::string individualPath;

                while (std::getline(env_var_string, individualPath, isWindows ? ';' : ':'))
                {
                    pathList.insert(individualPath + "/share");
                }
            }
        }
    }

    // Helper to clean path separators
    auto cleanPathSeparator = [isWindows](const std::string& filename) -> std::string {
        std::string output = filename;
        char pathSeparator = isWindows ? '\\' : '/';
        char wrongPathSeparator = isWindows ? '/' : '\\';

        for (size_t i = 0; i < output.size(); ++i)
        {
            if (output[i] == wrongPathSeparator)
            {
                output[i] = pathSeparator;
            }
        }

        return output;
    };

    // Helper to check if a file exists
    auto isFileExisting = [](const std::string& filename) -> bool {
        if (FILE* file = fopen(filename.c_str(), "r"))
        {
            fclose(file);
            return true;
        } else
        {
            return false;
        }
    };

    // Helper to get the file path by resolving the URI prefix
    auto getFilePath
        = [isFileExisting, cleanPathSeparator](const std::string& filename,
                                               const std::string& prefixToRemove,
                                               const std::unordered_set<std::string>& paths) {
              // If the file already exists as specified, return it as-is
              if (isFileExisting(filename))
              {
                  return filename;
              }

              // Check if the filename starts with the prefix to remove
              if (filename.substr(0, prefixToRemove.size()) == prefixToRemove)
              {
                  std::string filename_noprefix = filename;
                  filename_noprefix.erase(0, prefixToRemove.size());

                  // Try each path in the list
                  for (const std::string& path : paths)
                  {
                      const std::string testPath = cleanPathSeparator(path + filename_noprefix);
                      if (isFileExisting(testPath))
                      {
                          return testPath;
                      }
                  }
              }

              // By default, return the input if resolution fails
              return filename;
          };

    // Support both package:// and model:// URI schemes
    // Try package:// first
    std::string result = getFilePath(uri, "package:/", pathList);

    // If package:// didn't resolve and the URI starts with model://, try that
    if (result == uri && uri.substr(0, 8) == "model://")
    {
        result = getFilePath(uri, "model:/", pathList);
    }

    return result;
}

} // namespace iDynTree
