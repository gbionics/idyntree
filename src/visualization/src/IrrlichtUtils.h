// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_IRRLICHT_UTILS_H
#define IDYNTREE_IRRLICHT_UTILS_H

#include <EMaterialTypes.h>
#include <iDynTree/SolidShapes.h>
#include <iDynTree/Visualizer.h>

#include <irrlicht.h>

#ifdef IDYNTREE_USES_ASSIMP
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#endif

#include <Eigen/Core>
#include <Eigen/LU>

#include <iDynTree/EigenHelpers.h>

#include "FloorGridSceneNode.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <vector>

namespace iDynTree
{

inline irr::video::SColorf idyntree2irrlicht(iDynTree::ColorViz color)
{
    return irr::video::SColorf(color.r, color.g, color.b, color.a);
}

inline iDynTree::ColorViz irrlicht2idyntree(irr::video::SColorf color)
{
    return iDynTree::ColorViz((double)color.r, (double)color.g, (double)color.b, (double)color.a);
}

inline irr::core::vector3df idyntree2irr_rpy(const iDynTree::Vector3& vecId)
{
    double kRad2deg = 180 / M_PI;
    return irr::core::vector3df(kRad2deg * vecId(0), kRad2deg * vecId(1), kRad2deg * vecId(2));
}

inline irr::core::vector3df idyntree2irr_pos(const iDynTree::Vector3& vecId)
{
    return irr::core::vector3df(vecId(0), vecId(1), vecId(2));
}

inline iDynTree::Position irr2idyntree_pos(const irr::core::vector3df& vecIrr)
{
    return iDynTree::Position(vecIrr.X, vecIrr.Y, vecIrr.Z);
}

inline iDynTree::Vector3 irr2idyntree_rpy(const irr::core::vector3df& vecIrr)
{
    double kDeg2rad = M_PI / 180;
    iDynTree::Vector3 ret;
    ret(0) = kDeg2rad * vecIrr.X;
    ret(1) = kDeg2rad * vecIrr.Y;
    ret(2) = kDeg2rad * vecIrr.Z;
    return ret;
}

inline iDynTree::Rotation irr2idyntree_rot(const irr::core::vector3df& rotIrr)
{
    iDynTree::Vector3 rpy = irr2idyntree_rpy(rotIrr);
    return iDynTree::Rotation::RPY(rpy(0), rpy(1), rpy(2));
}

inline const irr::core::vector3df idyntree2irr_rot(const iDynTree::Rotation& rot)
{
    return idyntree2irr_rpy(rot.asRPY());
}

inline irr::video::SMaterial idyntree2irr(const iDynTree::Vector4& rgbaMaterialId)
{
    double ambientCoeff = 0.6;
    double diffuseCoeff = 1.0;
    irr::video::SMaterial ret;
    ret.AmbientColor = irr::video::SColorf(ambientCoeff * rgbaMaterialId(0),
                                           ambientCoeff * rgbaMaterialId(1),
                                           ambientCoeff * rgbaMaterialId(2),
                                           rgbaMaterialId(3))
                           .toSColor();
    ret.DiffuseColor = irr::video::SColorf(diffuseCoeff * rgbaMaterialId(0),
                                           diffuseCoeff * rgbaMaterialId(1),
                                           diffuseCoeff * rgbaMaterialId(2),
                                           rgbaMaterialId(3))
                           .toSColor();

    return ret;
}

inline irr::video::SMaterial idyntree2irr(const iDynTree::ColorViz& rgbaMaterial)
{
    iDynTree::Vector4 vec4;
    vec4(0) = rgbaMaterial.r;
    vec4(1) = rgbaMaterial.g;
    vec4(2) = rgbaMaterial.b;
    vec4(3) = rgbaMaterial.a;
    return idyntree2irr(vec4);
}

inline iDynTree::Transform irr2idyntree_trans(const irr::core::matrix4& transIrr)
{
    iDynTree::Transform output;

    output.setPosition(irr2idyntree_pos(transIrr.getTranslation()));
    output.setRotation(irr2idyntree_rot(transIrr.getRotationDegrees()));

    return output;
}

/**
 * Get a rotation matrix whose z column is the provided direction.
 */
inline iDynTree::Rotation RotationWithPrescribedZColumn(const iDynTree::Direction zAxis)
{
    // We need a normal vector perpendicular to z : we can extract this
    // from the nullspace of the transpose of z
    Eigen::Vector3d z = iDynTree::toEigen(zAxis);
    Eigen::Matrix<double, 1, 3> zTrans = z.transpose();

    // x is a normal vector orthognal to z
    Eigen::Vector3d x = zTrans.fullPivLu().kernel().block<3, 1>(0, 0).normalized();

    Eigen::Vector3d y = z.cross(x);

    iDynTree::Rotation R;

    toEigen(R).block<3, 1>(0, 0) = x;
    toEigen(R).block<3, 1>(0, 1) = y;
    toEigen(R).block<3, 1>(0, 2) = z;

    return R;
}

inline std::string getFileExt(const std::string& filename)
{
    auto idx = filename.rfind('.');

    if (idx != std::string::npos && idx + 1 < filename.size())
    {
        std::string ext = filename.substr(idx + 1);
        std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) {
            return std::tolower(c);
        });

        return ext;
    } else
    {
        return ""; // No extension found
    }
}

inline irr::scene::IAnimatedMesh* loadExternalMeshWithIrrlicht(irr::scene::ISceneManager* smgr,
                                                               const std::string& meshPath,
                                                               const std::string& ext)
{
    if (ext == "stl")
    {
        if (auto* cached = smgr->getMeshCache()->getMeshByName(meshPath.c_str()))
        {
            return cached;
        }
        std::ifstream file(meshPath, std::ios::binary | std::ios::ate);
        if (file)
        {
            const auto endPos = file.tellg();
            if (endPos >= 84)
            {
                const auto fileSize = static_cast<std::uint64_t>(endPos);
                std::array<std::uint8_t, 84> header{};
                file.seekg(0);
                if (file.read(reinterpret_cast<char*>(header.data()), header.size()))
                {
                    const bool startsWithSolid
                        = ((header[0] | 0x20u) == 's') && ((header[1] | 0x20u) == 'o')
                          && ((header[2] | 0x20u) == 'l') && ((header[3] | 0x20u) == 'i')
                          && ((header[4] | 0x20u) == 'd');
                    const std::uint32_t triCount
                        = static_cast<std::uint32_t>(header[80])
                          | (static_cast<std::uint32_t>(header[81]) << 8u)
                          | (static_cast<std::uint32_t>(header[82]) << 16u)
                          | (static_cast<std::uint32_t>(header[83]) << 24u);
                    if (startsWithSolid && (84ull + 50ull * triCount == fileSize))
                    {
                        std::vector<char> buffer(static_cast<std::size_t>(fileSize));
                        std::memcpy(buffer.data(), header.data(), header.size());
                        buffer[0] = '_';
                        if (file.read(buffer.data() + header.size(),
                                      static_cast<std::streamsize>(fileSize - header.size())))
                        {
                            irr::io::IReadFile* memFile
                                = smgr->getFileSystem()->createMemoryReadFile(buffer.data(),
                                                                              static_cast<irr::s32>(
                                                                                  buffer.size()),
                                                                              meshPath.c_str(),
                                                                              false);
                            irr::scene::IAnimatedMesh* mesh = smgr->getMesh(memFile);
                            memFile->drop();
                            return mesh;
                        }
                    }
                }
            }
        }
    }
    return smgr->getMesh(meshPath.c_str());
}

inline irr::scene::ISceneNode* addGeometryToSceneManager(const iDynTree::SolidShape* geom,
                                                         irr::scene::ISceneNode* linkNode,
                                                         irr::scene::ISceneManager* smgr)
{
    // In general we use the material provided by iDynTree, unless the external
    // mesh has a specific formal (for now collada) that we now specifies internally materials
    bool use_iDynTree_material = true;

    irr::scene::IMeshSceneNode* geomNode = 0;

    if (geom->isBox())
    {
        const iDynTree::Box* box = geom->asBox();

        irr::scene::IMesh* boxMesh = smgr->getGeometryCreator()->createCubeMesh(
            irr::core::vector3df(box->getX(), box->getY(), box->getZ()));

        geomNode = smgr->addMeshSceneNode(boxMesh, linkNode);

        boxMesh->drop();
    }

    if (geom->isSphere())
    {
        const iDynTree::Sphere* sphere = geom->asSphere();

        // geomNode = smgr->addSphereSceneNode(sphere->radius,16,linkNode);

        irr::scene::IMesh* sphereMesh
            = smgr->getGeometryCreator()->createSphereMesh(sphere->getRadius());

        geomNode = smgr->addMeshSceneNode(sphereMesh, linkNode);
    }

    if (geom->isCylinder())
    {
        const iDynTree::Cylinder* cylinder = geom->asCylinder();

        irr::scene::IMesh* cylinderMesh
            = smgr->getGeometryCreator()->createCylinderMesh(cylinder->getRadius(),
                                                             cylinder->getLength(),
                                                             16);

        // The Irrlicht geometry creator creates meshes that are symmetric wrt to the y axis and the
        // origin in the base while iDynTree/URDF has them parallel to the z axis, and the origin in
        // the center so we rotate and translate the mesh before creating the scene node
        irr::core::matrix4 irr2idyntree;
        irr2idyntree.buildRotateFromTo(irr::core::vector3df(0.0, 1.0, 0.0),
                                       irr::core::vector3df(0.0, 0.0, 1.0));
        irr2idyntree(3, 2) = -cylinder->getLength() / 2.0;

        smgr->getMeshManipulator()->transform(cylinderMesh, irr2idyntree);

        geomNode = smgr->addMeshSceneNode(cylinderMesh, linkNode);
    }

    if (geom->isExternalMesh())
    {
        const iDynTree::ExternalMesh* externalMesh = geom->asExternalMesh();
        const std::string ext = getFileExt(externalMesh->getFilename());
        const std::string resolvedPath = externalMesh->getFileLocationOnLocalFileSystem();

        irr::scene::IAnimatedMesh* loadedAnimatedMesh
            = loadExternalMeshWithIrrlicht(smgr, resolvedPath, ext);

        if (!loadedAnimatedMesh)
        {
            std::cerr << "Error in loading mesh " << resolvedPath << std::endl;
            return 0;
        }

        // If multiple mesh are loaded, add them
        if (ext == "dae" || ext == "obj")
        {
            use_iDynTree_material = false;
        }

        iDynTree::Vector3 scale = externalMesh->getScale();

        // Hack: if stl mesh, flip x
        if (ext == "stl" || ext == "obj")
        {
            scale(0) = -scale(0); // STL and OBJ meshes are interpreted as left handed by irrlicht
        }

        auto vCount = loadedAnimatedMesh->getMesh(0)->getMeshBuffer(0)->getVertexCount();

        // in case mesh has vCount > 65535, create new mesh  with 32 bit indices using
        // CDynamicMeshBuffer
        if (vCount > 65535)
        {
            std::cerr << "Loaded mesh has " << vCount << " vertices, creating CDynamicMeshBuffer"
                      << std::endl;
            // TODO: while this code fixes the itype issue. Still Irrlich will not call
            // drawVertexPrimitiveList and drawIndexedTriangleList on meshes with larger than 65535
            // vertices. This means that the error will not be triggered but mesh will not be
            // rendered correctly anyways, check
            // https://github.com/zaki/irrlicht/blob/97472da9c22ae4a49dfcefb9da5156601fa6a82a/include/IVideoDriver.h#L671

            irr::scene::CDynamicMeshBuffer* meshBuffer
                = new irr::scene::CDynamicMeshBuffer(irr::video::EVT_TANGENTS,
                                                     irr::video::EIT_32BIT);

#ifdef IDYNTREE_USES_ASSIMP
            Assimp::Importer importer;
            const aiScene* scene
                = importer.ReadFile(externalMesh->getFileLocationOnLocalFileSystem().c_str(),
                                    aiProcess_Triangulate | aiProcess_JoinIdenticalVertices
                                        | aiProcess_GenSmoothNormals | aiProcess_CalcTangentSpace
                                        | aiProcess_OptimizeMeshes | aiProcess_OptimizeGraph);

            if (!scene || !scene->HasMeshes())
            {
                std::cerr << "[addGeometryToSceneManager] Assimp failed to load or no meshes found "
                             "in the mesh file."
                          << std::endl;
            }

            const aiMesh* mesh = scene->mMeshes[0];
            const unsigned int numVertices = mesh->mNumVertices;
            const unsigned int numFaces = mesh->mNumFaces;

            meshBuffer->getVertexBuffer().set_used(numVertices);
            meshBuffer->getIndexBuffer().set_used(numFaces * 3);

            for (unsigned int i = 0; i < numVertices; ++i)
            {
                irr::video::S3DVertex vtx;
                vtx.Pos.X = mesh->mVertices[i].x;
                vtx.Pos.Y = mesh->mVertices[i].y;
                vtx.Pos.Z = mesh->mVertices[i].z;
                vtx.Normal.X = mesh->mNormals[i].x;
                vtx.Normal.Y = mesh->mNormals[i].y;
                vtx.Normal.Z = mesh->mNormals[i].z;
                vtx.Color = irr::video::SColor(255, 255, 255, 255);
                meshBuffer->getVertexBuffer()[i] = vtx;
            }

            irr::u32* indexBuffer = static_cast<irr::u32*>(meshBuffer->getIndexBuffer().pointer());
            for (unsigned int i = 0; i < numFaces; ++i)
            {
                indexBuffer[i * 3] = mesh->mFaces[i].mIndices[0];
                indexBuffer[i * 3 + 1] = mesh->mFaces[i].mIndices[1];
                indexBuffer[i * 3 + 2] = mesh->mFaces[i].mIndices[2];
            }
            irr::scene::SMesh* irrMesh = new irr::scene::SMesh();
            irrMesh->addMeshBuffer(meshBuffer);
            irrMesh->setHardwareMappingHint(irr::scene::EHM_STATIC);
            irrMesh->recalculateBoundingBox();

            auto animatedMesh = smgr->getMeshManipulator()->createAnimatedMesh(irrMesh);
            geomNode = smgr->addMeshSceneNode(animatedMesh, linkNode);
            scale(0) = -scale(0); // assimp does not need to flip x
            geomNode->setScale(idyntree2irr_pos(scale));

            // Remember to release resources
            animatedMesh->drop();
            irrMesh->drop();
#else
            std::cerr << "[Info] addGeometryToSceneManager: if rendering issues still occur, "
                         "consider reducing the number of vertices in the mesh (e.g. via meshlab)"
                      << std::endl;

            meshBuffer->getVertexBuffer().reallocate(vCount);
            meshBuffer->getIndexBuffer().reallocate(
                loadedAnimatedMesh->getMesh(0)->getMeshBuffer(0)->getIndexCount());
            for (irr::u64 i = 0; i < vCount; i++)
            {
                irr::video::S3DVertex vtx = static_cast<irr::video::S3DVertex*>(
                    loadedAnimatedMesh->getMesh(0)->getMeshBuffer(0)->getVertices())[i];
                vtx.Normal.normalize();
                meshBuffer->getVertexBuffer().push_back(vtx);
            }
            for (irr::u64 i = 0;
                 i < loadedAnimatedMesh->getMesh(0)->getMeshBuffer(0)->getIndexCount();
                 i++)
            {
                meshBuffer->getIndexBuffer().push_back(
                    loadedAnimatedMesh->getMesh(0)->getMeshBuffer(0)->getIndices()[i]);
            }
            meshBuffer->setBoundingBox(
                loadedAnimatedMesh->getMesh(0)->getMeshBuffer(0)->getBoundingBox());
            irr::scene::SMesh* mesh = new irr::scene::SMesh();
            mesh->addMeshBuffer(meshBuffer);
            mesh->setHardwareMappingHint(irr::scene::EHM_STATIC);
            mesh->recalculateBoundingBox();
            auto animatedMesh = smgr->getMeshManipulator()->createAnimatedMesh(mesh);
            geomNode = smgr->addMeshSceneNode(animatedMesh, linkNode);
            geomNode->setScale(idyntree2irr_pos(scale));

            animatedMesh->drop();
            meshBuffer->drop();
#endif
        } else
        {
            geomNode = smgr->addMeshSceneNode(loadedAnimatedMesh->getMesh(0), linkNode);
            geomNode->setScale(idyntree2irr_pos(scale));
        }
    }

    if (!geomNode)
    {
        std::cerr << "[ERROR] addGeometryToSceneManager: geometry not loaded" << std::endl;
        return 0;
    }

    // Set materials
    if (use_iDynTree_material)
    {
        for (irr::u32 mat = 0; mat < geomNode->getMaterialCount(); mat++)
        {
            geomNode->getMaterial(mat) = idyntree2irr(geom->getMaterial().color());
        }
    }

    geomNode->setMaterialFlag(irr::video::EMF_BACK_FACE_CULLING, false);
    geomNode->setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS, true);
    geomNode->setPosition(idyntree2irr_pos(geom->getLink_H_geometry().getPosition()));
    geomNode->setRotation(idyntree2irr_rot(geom->getLink_H_geometry().getRotation()));

    geomNode->setName(geom->getName().c_str());

    return geomNode;
}

inline irr::scene::ISceneNode*
addFrameAxes(irr::scene::ISceneManager* smgr,
             irr::scene::ISceneNode* parentNode = 0,
             irr::f32 arrowLength = 1.0,
             const irr::video::SColor& red = irr::video::SColor(20, 255, 0, 0),
             const irr::video::SColor& green = irr::video::SColor(20, 0, 255, 0),
             const irr::video::SColor& blue = irr::video::SColor(20, 0, 0, 255))
{
    irr::video::SMaterial transRed;
    transRed.AmbientColor = red;
    transRed.DiffuseColor = red;

    irr::video::SMaterial transGreen;
    transGreen.AmbientColor = green;
    transGreen.DiffuseColor = green;

    irr::video::SMaterial transBlue;
    transBlue.AmbientColor = blue;
    transBlue.DiffuseColor = blue;

    irr::scene::ISceneNode* frameNode = smgr->addEmptySceneNode(parentNode);

    irr::f32 arrowHeight = 1.2 * arrowLength;
    irr::f32 cylinderHeight = arrowLength;
    irr::f32 cylinderWidth = std::max(0.01 * arrowLength, 0.005);
    irr::f32 coneWidth = 5 * cylinderWidth;

    irr::scene::IMesh* arrowMesh = smgr->getGeometryCreator()->createArrowMesh(4,
                                                                               8,
                                                                               arrowHeight,
                                                                               cylinderHeight,
                                                                               cylinderWidth,
                                                                               coneWidth);

    irr::scene::ISceneNode* xArrow = smgr->addMeshSceneNode(arrowMesh, frameNode);
    xArrow->setPosition(irr::core::vector3df(0.0, 0.0, 0.0));
    xArrow->setRotation(irr::core::vector3df(0.0, 0.0, -90.0));
    xArrow->getMaterial(0) = transRed;
    xArrow->getMaterial(1) = transRed;

    irr::scene::ISceneNode* yArrow = smgr->addMeshSceneNode(arrowMesh, frameNode);
    yArrow->setPosition(irr::core::vector3df(0.0, 0.0, 0.0));
    yArrow->setRotation(irr::core::vector3df(0.0, 0.0, 0.0));
    yArrow->getMaterial(0) = transGreen;
    yArrow->getMaterial(1) = transGreen;

    irr::scene::ISceneNode* zArrow = smgr->addMeshSceneNode(arrowMesh, frameNode);
    zArrow->setPosition(irr::core::vector3df(0.0, 0.0, 0.0));
    zArrow->setRotation(irr::core::vector3df(90.0, 0.0, 0.0));
    zArrow->getMaterial(0) = transBlue;
    zArrow->getMaterial(1) = transBlue;

    arrowMesh->drop();

    return frameNode;
}

inline CFloorGridSceneNode*
addFloorGridNode(irr::scene::ISceneManager* smgr, irr::scene::ISceneNode* parentNode = 0)
{
    return new CFloorGridSceneNode(parentNode, smgr);
}

inline void setWorldHNode(irr::scene::ISceneNode* node, const iDynTree::Transform& trans)
{
    node->setPosition(idyntree2irr_pos(trans.getPosition()));
    node->setRotation(idyntree2irr_rot(trans.getRotation()));
}

/**
 * Add the visualization enviroment (plane, frame of ref) to the scene.
 */
void inline addVizEnviroment(irr::scene::ISceneManager* smgr)
{
    // Add origin frame
    irr::u32 alphaLev = 20;
    irr::video::SMaterial transRed;
    transRed.AmbientColor = irr::video::SColor(alphaLev, 255, 0, 0);
    transRed.DiffuseColor = irr::video::SColor(alphaLev, 255, 0, 0);

    irr::video::SMaterial transGreen;
    transGreen.AmbientColor = irr::video::SColor(alphaLev, 0, 255, 0);
    transGreen.DiffuseColor = irr::video::SColor(alphaLev, 0, 255, 0);

    irr::video::SMaterial transBlue;
    transBlue.AmbientColor = irr::video::SColor(alphaLev, 0, 0, 255);
    transBlue.DiffuseColor = irr::video::SColor(alphaLev, 0, 0, 255);

    irr::video::SMaterial transYellow;
    transYellow.AmbientColor = irr::video::SColor(alphaLev, 255, 255, 0);
    transYellow.DiffuseColor = irr::video::SColor(alphaLev, 255, 255, 0);

    irr::video::SMaterial transGray;
    transYellow.AmbientColor = irr::video::SColor(alphaLev, 100, 100, 100);
    transYellow.DiffuseColor = irr::video::SColor(alphaLev, 100, 100, 100);

    irr::scene::IMesh* arrowMesh
        = smgr->getGeometryCreator()->createArrowMesh(4, 8, 1.2, 1.0, 0.01, 0.05);

    irr::scene::ISceneNode* xArrow = smgr->addMeshSceneNode(arrowMesh);
    xArrow->setPosition(irr::core::vector3df(0.0, 0.0, 0.0));
    xArrow->setRotation(irr::core::vector3df(0.0, 0.0, -90.0));
    xArrow->getMaterial(0) = transRed;
    xArrow->getMaterial(1) = transRed;

    irr::scene::ISceneNode* yArrow = smgr->addMeshSceneNode(arrowMesh);
    yArrow->setPosition(irr::core::vector3df(0.0, 0.0, 0.0));
    yArrow->setRotation(irr::core::vector3df(0.0, 0.0, 0.0));
    yArrow->getMaterial(0) = transGreen;
    yArrow->getMaterial(1) = transGreen;

    irr::scene::ISceneNode* zArrow = smgr->addMeshSceneNode(arrowMesh);
    zArrow->setPosition(irr::core::vector3df(0.0, 0.0, 0.0));
    zArrow->setRotation(irr::core::vector3df(90.0, 0.0, 0.0));
    zArrow->getMaterial(0) = transBlue;
    zArrow->getMaterial(1) = transBlue;
}

void inline addVizLights(irr::scene::ISceneManager* smgr)
{
    smgr->setAmbientLight(irr::video::SColor(255, 255, 255, 255));
}

inline irr::scene::ICameraSceneNode* addVizCamera(irr::scene::ISceneManager* smgr)
{
    irr::scene::ICameraSceneNode* camera = smgr->addCameraSceneNode(0,
                                                                    irr::core::vector3df(-1, 0, 0),
                                                                    irr::core::vector3df(1, 0, 0));
    // scene::ICameraSceneNode* camera = smgr->addCameraSceneNodeMaya();
    camera->setPosition(irr::core::vector3df(0.8, 0.8, 0.8));
    camera->setTarget(irr::core::vector3df(0, 0, 0));
    camera->setUpVector(irr::core::vector3df(0.0, 0.0, 1.0));
    camera->bindTargetAndRotation(true); // To change the target and the rotation at the same time

    // See http://irrlicht.sourceforge.net/forum/viewtopic.php?f=4&t=47734
    irr::core::matrix4 matproj = camera->getProjectionMatrix();
    matproj(0, 0) *= -1;
    camera->setProjectionMatrix(matproj);

    return camera;
}

// Imported from http://irrlicht.sourceforge.net/forum/viewtopic.php?f=9&t=45413
// TODO : rewrite for efficency
inline irr::scene::IMesh* createFrustumMesh(irr::f32 topRadius,
                                            irr::f32 bottomRadius,
                                            irr::f32 height,
                                            irr::u32 tesselation = 8)
{
    irr::scene::SMeshBuffer* buffer = new irr::scene::SMeshBuffer();
    irr::video::S3DVertex v;
    irr::f32 radius;
    irr::f32 angle;
    irr::f32 x, y, z;
    irr::f32 s, t;

    for (irr::u32 m = 0; m <= tesselation; ++m) // height step:
    {
        z = height * irr::f32(m) / irr::f32(tesselation);
        radius = ((height - z) * (bottomRadius - topRadius) / height) + topRadius;
        // radius = 15.0f;
        t = 1.0f - irr::f32(m) / irr::f32(tesselation);
        for (irr::u32 n = 0; n <= tesselation; ++n) // subdivide circle:
        {
            angle = irr::core::PI * 2.0f * irr::f32(n) / irr::f32(tesselation);
            x = radius * cosf(angle);
            y = radius * sinf(angle);
            s = irr::f32(n) / irr::f32(tesselation);

            v.Pos.X = x;
            v.Pos.Y = y;
            v.Pos.Z = z;

            v.Normal = v.Pos;
            v.Normal.normalize();
            v.Color = 0xFFFFFFFF;

            // v.TCoords.X = s;
            // v.TCoords.Y = t;
            v.TCoords = irr::core::vector2df(s, t);

            buffer->Vertices.push_back(v);
            // printf("(s,t)=%f,%f\n", s, t);
        }
    }

    irr::u32 index00, index10, index11, index01;
    for (irr::u32 m = 0; m < tesselation; ++m)
    {
        for (irr::u32 n = 0; n < tesselation; ++n)
        {
            index00 = m * (tesselation + 1) + n;
            index10 = (m + 1) * (tesselation + 1) + n;
            index11 = (m + 1) * (tesselation + 1) + n + 1;
            index01 = m * (tesselation + 1) + n + 1;

            buffer->Indices.push_back(index00);
            buffer->Indices.push_back(index10);
            buffer->Indices.push_back(index11);

            buffer->Indices.push_back(index00);
            buffer->Indices.push_back(index11);
            buffer->Indices.push_back(index01);
        }
    }

    buffer->recalculateBoundingBox();
    irr::scene::SMesh* mesh = new irr::scene::SMesh();
    mesh->addMeshBuffer(buffer);
    buffer->drop();

    mesh->setHardwareMappingHint(irr::scene::EHM_STATIC);
    mesh->recalculateBoundingBox();
    return mesh;
}
} // namespace iDynTree

#endif
