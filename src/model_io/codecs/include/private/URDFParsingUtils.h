// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_URDF_PARSING_UTILS_H
#define IDYNTREE_URDF_PARSING_UTILS_H

#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include <iDynTree/Transform.h>
#include <iDynTree/VectorFixSize.h>

#include <fpconv.h>

namespace iDynTree
{

bool inline stringToDoubleWithClassicLocale(const std::string& inStr, double& outDouble)
{
    std::istringstream ss(inStr);
    ss.imbue(std::locale::classic());
    ss >> outDouble;
    return !(ss.fail());
}

bool inline stringToIntWithClassicLocale(const std::string& inStr, int& outInt)
{
    std::istringstream ss(inStr);
    ss.imbue(std::locale::classic());
    ss >> outInt;
    return !(ss.fail());
}

bool inline stringToUnsignedIntWithClassicLocale(const std::string& inStr, unsigned int& outInt)
{
    std::istringstream ss(inStr);
    ss.imbue(std::locale::classic());
    ss >> outInt;
    return !(ss.fail());
}

bool inline doubleToStringWithClassicLocale(const double& inDouble,
                                            std::string& outStr,
                                            int precision = 0)
{
    if (std::isnan(inDouble) || std::isinf(inDouble))
    {
        return false;
    }

    if (precision > 0)
    {
        // Use std::stringstream with specified precision
        std::ostringstream ss;
        ss.imbue(std::locale::classic());
        ss << std::setprecision(precision) << inDouble;
        outStr = ss.str();
    } else
    {
        // fpconv returns nul-terminated strings, that can be converted directly to C++ std::string
        // see
        // https://github.com/night-shift/fpconv/tree/4a087d1b2df765baa409536931916a2c082cdda4#example-usage
        char buf[24 + 1];
        int str_len = idyntree_private_fpconv_dtoa(inDouble, buf);
        buf[str_len] = '\0';
        outStr = buf;
    }
    return true;
}

std::string inline intToString(const int inInt)
{
    std::stringstream ss;
    ss << inInt;
    return ss.str();
}

std::string inline intToString(const size_t inInt)
{
    std::stringstream ss;
    ss << inInt;
    return ss.str();
}

/**
 * Split string along spaces
 */
bool inline splitString(const std::string& inStr, std::vector<std::string>& pieces)
{
    std::istringstream iss(inStr);

    pieces.resize(0);

    do
    {
        std::string sub;
        iss >> sub;
        if (sub != "")
        {
            pieces.push_back(sub);
        }
    } while (iss);

    return true;
}

/**
 *
 */
bool inline vector3FromString(const std::string& vector_str, Vector3& out)
{
    std::vector<std::string> pieces;
    std::vector<double> xyz;
    splitString(vector_str, pieces);
    for (unsigned int i = 0; i < pieces.size(); ++i)
    {
        if (pieces[i] != "")
        {
            double newDouble;
            if (stringToDoubleWithClassicLocale(pieces[i], newDouble))
            {
                xyz.push_back(newDouble);
            } else
            {
                std::string errStr = "Unable to parse component [" + pieces[i]
                                     + "] to a double (while parsing a vector value)";
                reportError("", "vector3FromString", errStr.c_str());
                return false;
            }
        }
    }

    if (xyz.size() != 3)
    {
        std::string errStr = "Parser found " + intToString(xyz.size())
                             + " elements but 3 expected while parsing vector [" + vector_str + "]";
        reportError("", "vector3FromString", errStr.c_str());
        return false;
    }

    out(0) = xyz[0];
    out(1) = xyz[1];
    out(2) = xyz[2];

    return true;
}

template <typename iDynTreeVectorType>
bool inline vectorToString(const iDynTreeVectorType& in, std::string& out_str, int precision = 0)
{
    std::stringstream ss;
    bool ok = true;
    for (unsigned int i = 0; i < in.size(); ++i)
    {
        std::string bufStr;
        ok = ok && doubleToStringWithClassicLocale(in(i), bufStr, precision);
        if (i != 0)
        {
            ss << " ";
        }
        ss << bufStr;
    }

    out_str = ss.str();

    return ok;
}

bool inline vector4FromString(const std::string& vector_str, Vector4& out)
{
    std::vector<std::string> pieces;
    std::vector<double> rgba;
    splitString(vector_str, pieces);
    for (unsigned int i = 0; i < pieces.size(); ++i)
    {
        if (pieces[i] != "")
        {
            double newDouble;
            if (stringToDoubleWithClassicLocale(pieces[i], newDouble))
            {
                rgba.push_back(newDouble);
            } else
            {
                std::string errStr = "Unable to parse component [" + pieces[i]
                                     + "] to a double (while parsing a vector value)";
                reportError("", "vector4FromString", errStr.c_str());
                return false;
            }
        }
    }

    if (rgba.size() != 4)
    {
        std::string errStr = "Parser found " + intToString(rgba.size())
                             + " elements but 4 expected while parsing vector [" + vector_str + "]";
        reportError("", "vector4FromString", errStr.c_str());
        return false;
    }

    out(0) = rgba[0];
    out(1) = rgba[1];
    out(2) = rgba[2];
    out(3) = rgba[3];

    return true;
}

} // namespace iDynTree

#endif
