// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_MODEL_IO_FORMAT_UTILS_H
#define IDYNTREE_MODEL_IO_FORMAT_UTILS_H

#include <algorithm>
#include <cctype>
#include <string>

namespace iDynTree
{

inline std::string modelFormatToLower(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

inline std::string normalizeModelFormatName(const std::string& format)
{
    return modelFormatToLower(format);
}

inline std::string inferModelFormatFromFilename(const std::string& filePath)
{
    const size_t dotPos = filePath.rfind('.');
    if (dotPos == std::string::npos)
    {
        return "";
    }

    const std::string extension = modelFormatToLower(filePath.substr(dotPos + 1));

    if (extension == "urdf")
    {
        return "urdf";
    }

    if (extension == "sdf" || extension == "world")
    {
        return "sdf";
    }

    if (extension == "json")
    {
        return "idyntree-model-json";
    }

    return "";
}

inline bool isSupportedImportModelFormat(const std::string& format)
{
    return format == "urdf" || format == "sdf" || format == "idyntree-model-json";
}

inline bool isSupportedExportModelFormat(const std::string& format)
{
    return format == "urdf" || format == "idyntree-model-json";
}

} // namespace iDynTree

#endif // IDYNTREE_MODEL_IO_FORMAT_UTILS_H