// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_SOLID_SHAPES_H
#define IDYNTREE_SOLID_SHAPES_H

#include <cstdlib>
#include <iDynTree/Indices.h>
#include <iDynTree/Transform.h>
#include <string>
#include <vector>

namespace iDynTree
{
class Material
{
public:
    explicit Material();
    explicit Material(const std::string& name);

    std::string name() const;

    bool hasColor() const;
    Vector4 color() const;
    void setColor(const Vector4& color);

    bool hasTexture() const;
    std::string texture() const;
    void setTexture(const std::string& texture);

private:
    Vector4 m_color;
    bool m_isColorSet;
    std::string m_texture;
    std::string m_name;
};

class Sphere;
class Box;
class Cylinder;
class ExternalMesh;
class Model;

class SolidShape
{
public:
    explicit SolidShape();

    virtual ~SolidShape() = 0;
    virtual SolidShape* clone() const = 0;

    /**
     * Returns the name of the shape.
     */
    const std::string& getName() const;

    /**
     * Sets the specified name.
     */
    void setName(const std::string& name);

    /**
     * Returns if the name is valid.
     */
    bool isNameValid() const;

    /**
     * Returns the homogeneus transformation of the geometry w.r.t. the attached link.
     */
    const Transform& getLink_H_geometry() const;

    /**
     * Sets the homogeneus transformation of the geometry w.r.t. the attached link.
     */
    void setLink_H_geometry(const Transform& newTransform);

    /**
     * Returns if the material is valid, i.e. you can call getMaterial().
     */
    bool isMaterialSet() const;

    /**
     * Returns the current material.
     */
    const Material& getMaterial() const;

    /**
     * Sets the material. isMaterialSet will return true after this call.
     */
    void setMaterial(const Material& material);

    bool isSphere() const;
    bool isBox() const;
    bool isCylinder() const;
    bool isExternalMesh() const;

    // Utility methods to traverse the SolidShape class hierachy.
    Sphere* asSphere();
    Box* asBox();
    Cylinder* asCylinder();
    ExternalMesh* asExternalMesh();

    const Sphere* asSphere() const;
    const Box* asBox() const;
    const Cylinder* asCylinder() const;
    const ExternalMesh* asExternalMesh() const;

private:
    std::string name;
    /**
     * True if the name is valid, false otherwise.
     */
    bool nameIsValid;

    Transform link_H_geometry;

    /**
     * Material of the geometry, encoded as a rgba vector.
     */
    Vector4 material;
    bool m_isMaterialSet;
    Material m_material;
};

class Sphere : public SolidShape
{
public:
    virtual ~Sphere();

    virtual SolidShape* clone() const;

    /**
     * Returns the current radius.
     */
    double getRadius() const;

    /**
     * Sets the new radius.
     */
    void setRadius(double radius);

private:
    double radius;
};

/**
 * @brief Box, i.e. 3D rectangular parallelepiped.
 *
 * The box is centered in the mesh frame, its sides
 * are aligned with the axis of the mesh frame, and
 * the side lenghts in the x, y and z direction are given
 * by the attributes x, y and z.
 */
class Box : public SolidShape
{
public:
    virtual ~Box();
    virtual SolidShape* clone() const;

    /**
     * Returns the current x side length.
     */
    double getX() const;

    /**
     * Sets the x side length.
     */
    void setX(double x);

    /**
     * Returns the current y side length.
     */
    double getY() const;

    /**
     * Sets the y side length.
     */
    void setY(double y);

    /**
     * Returns the current z side length.
     */
    double getZ() const;

    /**
     * Sets the z side length.
     */
    void setZ(double z);

private:
    double x;
    double y;
    double z;
};

class Cylinder : public SolidShape
{
public:
    virtual ~Cylinder();
    virtual SolidShape* clone() const;

    /**
     * Returns the current cylinder length.
     */
    double getLength() const;

    /**
     * Sets the cylinder length.
     */
    void setLength(double length);

    /**
     * Returns the current cylinder radius.
     */
    double getRadius() const;

    /**
     * Sets the cylinder radius.
     */
    void setRadius(double radius);

private:
    double length;
    double radius;
};

class ExternalMesh : public SolidShape
{
public:
    virtual ~ExternalMesh();
    virtual SolidShape* clone() const;

    /**
     * Returns the current filename.
     */
    const std::string& getFilename() const;

    /**
     * Returns the current package directories.
     */
    const std::vector<std::string>& getPackageDirs() const;

    /**
     * Returns the filename substituting the prefix "package://" with the corresponding absolute
     * path. The absolute path is determined by searching for the file using the paths specified in
     * the packageDirs vector or if empty in the "GZ_SIM_RESOURCE_PATH", "GAZEBO_MODEL_PATH",
     * "ROS_PACKAGE_PATH" and "AMENT_PREFIX_PATH" environmental variables.
     */
    std::string getFileLocationOnLocalFileSystem() const;

    /**
     * Sets the filename.
     */
    void setFilename(const std::string& filename);

    /**
     * Sets the the package directories.
     * @note if not set the absolute path is determined by searching for the file using the
     * paths specified in the "GZ_SIM_RESOURCE_PATH", "GAZEBO_MODEL_PATH", "ROS_PACKAGE_PATH" and
     * "AMENT_PREFIX_PATH" environmental variables.
     */
    void setPackageDirs(const std::vector<std::string>& packageDirs);

    /**
     * Returns the current scale.
     */
    const iDynTree::Vector3& getScale() const;

    /**
     * Sets the scale.
     */
    void setScale(const iDynTree::Vector3& scale);

private:
    std::string filename;
    std::vector<std::string> packageDirs;
    iDynTree::Vector3 scale;
};

class ModelSolidShapes
{
private:
    ModelSolidShapes& copy(const ModelSolidShapes& other);
    /**
     * Storage ot ModelSolidShapes.
     */
    std::vector<std::vector<SolidShape*>> linkSolidShapes;

public:
    ModelSolidShapes();
    ~ModelSolidShapes();

    ModelSolidShapes(const ModelSolidShapes& other);
    ModelSolidShapes& operator=(const ModelSolidShapes& other);

    void clear();
    void resize(size_t nrOfLinks);
    void resize(const Model& model);
    bool isConsistent(const Model& model) const;

    void clearSingleLinkSolidShapes(LinkIndex linkIndex);
    void addSingleLinkSolidShape(LinkIndex linkIndex, const SolidShape& shape);

    std::vector<std::vector<SolidShape*>>& getLinkSolidShapes();

    const std::vector<std::vector<SolidShape*>>& getLinkSolidShapes() const;
};

} // namespace iDynTree

#endif
