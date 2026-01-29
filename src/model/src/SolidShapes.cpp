// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/Model.h>
#include <iDynTree/SolidShapes.h>

#include <cstdlib>
#include <fstream>
#include <string>
#include <unordered_set>

namespace iDynTree
{
Material::Material()
    : Material("")
{
}

Material::Material(const std::string& name)
    : m_isColorSet(false)
    , m_name(name)
{
}

std::string Material::name() const
{
    return m_name;
}

bool Material::hasColor() const
{
    return m_isColorSet;
}

Vector4 Material::color() const
{
    return m_color;
}

void Material::setColor(const Vector4& color)
{
    m_color = color;
    m_isColorSet = true;
}

bool Material::hasTexture() const
{
    return !m_texture.empty();
}

std::string Material::texture() const
{
    return m_texture;
}

void Material::setTexture(const std::string& texture)
{
    m_texture = texture;
}

SolidShape::SolidShape()
    : nameIsValid(false)
    , link_H_geometry(iDynTree::Transform::Identity())
    , m_isMaterialSet(false)
{
}

SolidShape::~SolidShape()
{
}

const std::string& SolidShape::getName() const
{
    return name;
}

void SolidShape::setName(const std::string& name)
{
    this->name = name;
    nameIsValid = !name.empty();
}

bool SolidShape::isNameValid() const
{
    return nameIsValid;
}

const Transform& SolidShape::getLink_H_geometry() const
{
    return link_H_geometry;
}

void SolidShape::setLink_H_geometry(const Transform& newTransform)
{
    this->link_H_geometry = newTransform;
}

bool SolidShape::isMaterialSet() const
{
    return m_isMaterialSet;
}

const Material& SolidShape::getMaterial() const
{
    return m_material;
}

void SolidShape::setMaterial(const Material& material)
{
    m_material = material;
    m_isMaterialSet = true;
}

bool SolidShape::isSphere() const
{
    return (dynamic_cast<const Sphere*>(this) != 0);
}

Sphere* SolidShape::asSphere()
{
    return dynamic_cast<Sphere*>(this);
}

const Sphere* SolidShape::asSphere() const
{
    return dynamic_cast<const Sphere*>(this);
}

bool SolidShape::isCylinder() const
{
    return (dynamic_cast<const Cylinder*>(this) != 0);
}

Cylinder* SolidShape::asCylinder()
{
    return dynamic_cast<Cylinder*>(this);
}

const Cylinder* SolidShape::asCylinder() const
{
    return dynamic_cast<const Cylinder*>(this);
}

bool SolidShape::isBox() const
{
    return (dynamic_cast<const Box*>(this) != 0);
}

Box* SolidShape::asBox()
{
    return dynamic_cast<Box*>(this);
}

const Box* SolidShape::asBox() const
{
    return dynamic_cast<const Box*>(this);
}

bool SolidShape::isExternalMesh() const
{
    return (dynamic_cast<const ExternalMesh*>(this) != 0);
}

ExternalMesh* SolidShape::asExternalMesh()
{
    return dynamic_cast<ExternalMesh*>(this);
}

const ExternalMesh* SolidShape::asExternalMesh() const
{
    return dynamic_cast<const ExternalMesh*>(this);
}

Sphere::~Sphere()
{
}

SolidShape* Sphere::clone() const
{
    return new Sphere(*this);
}

double Sphere::getRadius() const
{
    return radius;
}

void Sphere::setRadius(double radius)
{
    this->radius = radius;
}

Box::~Box()
{
}

SolidShape* Box::clone() const
{
    return new Box(*this);
}

double Box::getX() const
{
    return x;
}

void Box::setX(double x)
{
    this->x = x;
}

double Box::getY() const
{
    return y;
}

void Box::setY(double y)
{
    this->y = y;
}

double Box::getZ() const
{
    return z;
}

void Box::setZ(double z)
{
    this->z = z;
}

Cylinder::~Cylinder()
{
}

SolidShape* Cylinder::clone() const
{
    return new Cylinder(*this);
}

double Cylinder::getLength() const
{
    return length;
}

void Cylinder::setLength(double length)
{
    this->length = length;
}

double Cylinder::getRadius() const
{
    return radius;
}

void Cylinder::setRadius(double radius)
{
    this->radius = radius;
}

ExternalMesh::~ExternalMesh()
{
}

SolidShape* ExternalMesh::clone() const
{
    return new ExternalMesh(*this);
}

const std::string& ExternalMesh::getFilename() const
{
    return filename;
}

const std::vector<std::string>& ExternalMesh::getPackageDirs() const
{
    return this->packageDirs;
}

std::string ExternalMesh::getFileLocationOnLocalFileSystem() const
{
    bool isWindows = false;
#ifdef _WIN32
    isWindows = true;
#endif

    std::unordered_set<std::string> pathList;

    // if the user provides the packageDirs we do not check inside the dirs listed in the
    // environment variables
    if (!this->packageDirs.empty())
    {
        pathList
            = std::unordered_set<std::string>(this->packageDirs.begin(), this->packageDirs.end());
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

    auto getFilePath
        = [isFileExisting, cleanPathSeparator](const std::string& filename,
                                               const std::string& prefixToRemove,
                                               const std::unordered_set<std::string>& paths) {
              if (isFileExisting(filename))
              {
                  return filename;
              }

              if (filename.substr(0, prefixToRemove.size()) == prefixToRemove)
              {
                  std::string filename_noprefix = filename;
                  filename_noprefix.erase(0, prefixToRemove.size());
                  for (const std::string& path : paths)
                  {
                      const std::string testPath = cleanPathSeparator(path + filename_noprefix);
                      if (isFileExisting(testPath))
                      {
                          return testPath;
                      }
                  }
              }

              return filename; // By default return the input;
          };

    return getFilePath(filename, "package:/", pathList);
}

void ExternalMesh::setFilename(const std::string& filename)
{
    this->filename = filename;
}

void ExternalMesh::setPackageDirs(const std::vector<std::string>& packageDirs)
{
    this->packageDirs = packageDirs;
}

const iDynTree::Vector3& ExternalMesh::getScale() const
{
    return scale;
}

void ExternalMesh::setScale(const iDynTree::Vector3& scale)
{
    this->scale = scale;
}

void ModelSolidShapes::clear()
{
    for (size_t link = 0; link < linkSolidShapes.size(); link++)
    {
        for (size_t geom = 0; geom < linkSolidShapes[link].size(); geom++)
        {
            delete linkSolidShapes[link][geom];
            linkSolidShapes[link][geom] = 0;
        }
    }

    linkSolidShapes.resize(0);
}

ModelSolidShapes& ModelSolidShapes::copy(const ModelSolidShapes& other)
{
    clear();
    this->linkSolidShapes.resize(other.linkSolidShapes.size());
    for (size_t link = 0; link < other.linkSolidShapes.size(); link++)
    {
        this->linkSolidShapes[link].resize(other.linkSolidShapes[link].size());
        for (size_t geom = 0; geom < other.linkSolidShapes[link].size(); geom++)
        {
            this->linkSolidShapes[link][geom] = other.linkSolidShapes[link][geom]->clone();
        }
    }

    return *this;
}

void ModelSolidShapes::resize(const Model& model)
{
    this->resize(model.getNrOfLinks());
}

void ModelSolidShapes::resize(const size_t nrOfLinks)
{
    clear();
    this->linkSolidShapes.resize(nrOfLinks);
}

ModelSolidShapes::ModelSolidShapes()
{
}

ModelSolidShapes::ModelSolidShapes(const ModelSolidShapes& other)
{
    copy(other);
}

ModelSolidShapes& ModelSolidShapes::operator=(const ModelSolidShapes& other)
{
    return copy(other);
}

ModelSolidShapes::~ModelSolidShapes()
{
    this->clear();
}

bool ModelSolidShapes::isConsistent(const Model& model) const
{
    return (this->linkSolidShapes.size() == model.getNrOfLinks());
}

void ModelSolidShapes::clearSingleLinkSolidShapes(LinkIndex linkIndex)
{
    if (linkIndex >= linkSolidShapes.size())
    {
        return;
    }

    for (size_t geom = 0; geom < linkSolidShapes[linkIndex].size(); geom++)
    {
        delete linkSolidShapes[linkIndex][geom];
        linkSolidShapes[linkIndex][geom] = 0;
    }
    linkSolidShapes[linkIndex].resize(0);
}

void ModelSolidShapes::addSingleLinkSolidShape(LinkIndex linkIndex, const SolidShape& shape)
{
    if (linkIndex >= linkSolidShapes.size())
    {
        return;
    }

    linkSolidShapes[linkIndex].push_back(shape.clone());
}

std::vector<std::vector<SolidShape*>>& ModelSolidShapes::getLinkSolidShapes()
{
    return linkSolidShapes;
}

const std::vector<std::vector<SolidShape*>>& ModelSolidShapes::getLinkSolidShapes() const
{
    return linkSolidShapes;
}
} // namespace iDynTree
