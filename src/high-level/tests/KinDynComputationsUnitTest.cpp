// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "iDynTree/FreeFloatingMatrices.h"
#include "iDynTree/MatrixDynSize.h"
#include "iDynTree/Rotation.h"
#include "testModels.h"
#include <iDynTree/TestUtils.h>

#include <iDynTree/Position.h>
#include <iDynTree/SpatialAcc.h>
#include <iDynTree/SpatialInertia.h>
#include <iDynTree/SpatialMomentum.h>
#include <iDynTree/Transform.h>
#include <iDynTree/Twist.h>
#include <iDynTree/VectorDynSize.h>

#include <iDynTree/EigenHelpers.h>

#include <iDynTree/FreeFloatingState.h>
#include <iDynTree/JointState.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Link.h>
#include <iDynTree/Model.h>
#include <iDynTree/ModelTransformers.h>
#include <iDynTree/SubModel.h>
#include <iDynTree/Traversal.h>

#include <iDynTree/ModelTestUtils.h>

#include <iDynTree/ModelLoader.h>
#include <iostream>

using namespace iDynTree;

double random_double()
{
    return 1.0 * ((double)rand() - RAND_MAX / 2) / ((double)RAND_MAX);
}

double real_random_double()
{
    return 1.0 * ((double)rand() - RAND_MAX / 2) / ((double)RAND_MAX);
}

int real_random_int(int initialValue, int finalValue)
{
    int length = finalValue - initialValue;
    return initialValue + rand() % length;
}

void setRandomState(iDynTree::KinDynComputations& dynComp)
{
    size_t dofs = dynComp.getNrOfDegreesOfFreedom();
    size_t posCoords = dynComp.model().getNrOfPosCoords();
    Transform worldTbase;
    Twist baseVel;
    Vector3 gravity;

    iDynTree::VectorDynSize qj(posCoords), dqj(dofs), ddqj(dofs);

    // Use utility functions for generating random state
    worldTbase = getRandomTransform();
    baseVel = getRandomTwist();
    getRandomVector(gravity);

    // Use model-aware joint position generation for proper handling of all joint types
    getRandomJointPositions(qj, dynComp.model());
    getRandomVector(dqj);
    getRandomVector(ddqj);

    bool ok = dynComp.setRobotState(worldTbase, qj, baseVel, dqj, gravity);

    iDynTree::VectorDynSize qj_read(posCoords), dqj_read(dofs);
    Vector3 gravity_read;
    Transform worldTbase_read;
    Twist baseVel_read;
    dynComp.getRobotState(worldTbase_read, qj_read, baseVel_read, dqj_read, gravity_read);

    ASSERT_EQUAL_VECTOR(qj_read, qj);
    ASSERT_EQUAL_VECTOR(dqj_read, dqj);
    ASSERT_EQUAL_VECTOR(gravity_read, gravity);
    ASSERT_EQUAL_TRANSFORM(worldTbase_read, worldTbase);
    ASSERT_EQUAL_VECTOR(baseVel_read.asVector(), baseVel.asVector());

    ASSERT_EQUAL_DOUBLE(ok, true);
}

/**
 * Set a random state for fixed-base robot (base velocity = 0).
 */
void setRandomStateFixedBase(iDynTree::KinDynComputations& dynComp)
{
    size_t dofs = dynComp.getNrOfDegreesOfFreedom();
    size_t posCoords = dynComp.model().getNrOfPosCoords();
    Transform worldTbase;
    Twist baseVel;
    Vector3 gravity;

    iDynTree::VectorDynSize qj(posCoords), dqj(dofs), ddqj(dofs);

    // Use utility functions for generating random state
    worldTbase = getRandomTransform();
    // Fixed base: zero base velocity
    baseVel.zero();
    getRandomVector(gravity);

    // Use model-aware joint position generation for proper handling of all joint types
    getRandomJointPositions(qj, dynComp.model());
    getRandomVector(dqj);
    getRandomVector(ddqj);

    bool ok = dynComp.setRobotState(worldTbase, qj, baseVel, dqj, gravity);
    ASSERT_EQUAL_DOUBLE(ok, true);
}

void testRelativeTransform(iDynTree::KinDynComputations& dynComp)
{
    using namespace iDynTree;

    size_t frames = dynComp.getNrOfFrames();

    // Create some frames for random consistency checks
    FrameIndex A = getRandomInteger(0, frames - 1);
    FrameIndex B = getRandomInteger(0, frames - 1);
    FrameIndex C = getRandomInteger(0, frames - 1);
    FrameIndex D = getRandomInteger(0, frames - 1);
    FrameIndex E = getRandomInteger(0, frames - 1);
    FrameIndex F = getRandomInteger(0, frames - 1);

    Transform A_B_X_E_F = dynComp.getRelativeTransformExplicit(A, B, C, D)
                          * dynComp.getRelativeTransformExplicit(C, D, E, F);
    Transform A_B_X_E_F_check = dynComp.getRelativeTransformExplicit(A, B, E, F);
    Transform E_F_X_A_B = dynComp.getRelativeTransformExplicit(E, F, A, B);

    ASSERT_EQUAL_TRANSFORM(A_B_X_E_F, A_B_X_E_F_check);
    ASSERT_EQUAL_TRANSFORM(A_B_X_E_F, E_F_X_A_B.inverse());
}

void testAverageVelocityAndTotalMomentumJacobian(iDynTree::KinDynComputations& dynComp)
{
    iDynTree::Twist avgVel, centroidalAvgVel;
    iDynTree::SpatialMomentum mom, centroidalMom, computedMom, computedCentroidalMom;
    iDynTree::Vector6 avgVelCheck, centroidalAvgVelCheck, momCheck, centroidalMomCheck;
    iDynTree::VectorDynSize nu(dynComp.getNrOfDegreesOfFreedom() + 6);
    iDynTree::SpatialInertia lockedInertia, centroidalLockedInertia;
    dynComp.getModelVel(nu);

    MomentumFreeFloatingJacobian momJac(dynComp.getRobotModel());
    MomentumFreeFloatingJacobian centroidalMomJac(dynComp.getRobotModel());
    FrameFreeFloatingJacobian avgVelJac(dynComp.getRobotModel());
    FrameFreeFloatingJacobian centroidalAvgVelJac(dynComp.getRobotModel());

    // momentum quantities

    avgVel = dynComp.getAverageVelocity();
    bool ok = dynComp.getAverageVelocityJacobian(avgVelJac);

    ASSERT_IS_TRUE(ok);

    mom = dynComp.getLinearAngularMomentum();
    ok = dynComp.getLinearAngularMomentumJacobian(momJac);

    ASSERT_IS_TRUE(ok);

    lockedInertia = dynComp.getRobotLockedInertia();
    computedMom = lockedInertia * avgVel;

    // centroidal quantities

    centroidalAvgVel = dynComp.getCentroidalAverageVelocity();
    ok = dynComp.getCentroidalAverageVelocityJacobian(centroidalAvgVelJac);

    ASSERT_IS_TRUE(ok);

    centroidalMom = dynComp.getCentroidalTotalMomentum();
    ok = dynComp.getCentroidalTotalMomentumJacobian(centroidalMomJac);
    ASSERT_IS_TRUE(ok);

    centroidalLockedInertia = dynComp.getCentroidalRobotLockedInertia();
    computedCentroidalMom = centroidalLockedInertia * centroidalAvgVel;

    toEigen(momCheck) = toEigen(momJac) * toEigen(nu);
    toEigen(centroidalMomCheck) = toEigen(centroidalMomJac) * toEigen(nu);
    toEigen(avgVelCheck) = toEigen(avgVelJac) * toEigen(nu);
    toEigen(centroidalAvgVelCheck) = toEigen(centroidalAvgVelJac) * toEigen(nu);

    ASSERT_EQUAL_VECTOR(momCheck, mom.asVector());
    ASSERT_EQUAL_VECTOR(centroidalMomCheck, centroidalMom.asVector());

    ASSERT_EQUAL_VECTOR(avgVelCheck, avgVel.asVector());
    ASSERT_EQUAL_VECTOR(centroidalAvgVelCheck, centroidalAvgVel.asVector());

    ASSERT_EQUAL_VECTOR(mom.asVector(), computedMom);
    ASSERT_EQUAL_VECTOR(centroidalMom.asVector(), computedCentroidalMom);
}

inline Eigen::VectorXd toEigen(const Vector6& baseAcc, const VectorDynSize& jntAccs)
{
    Eigen::VectorXd concat(6 + jntAccs.size());
    if (jntAccs.size() > 0)
    {
        concat << toEigen(baseAcc), toEigen(jntAccs);
    } else
    {
        concat = toEigen(baseAcc);
    }
    return concat;
}

inline Eigen::VectorXd toEigen(const FreeFloatingGeneralizedTorques& genForces)
{
    Eigen::VectorXd concat(6 + genForces.jointTorques().size());
    // TODO(traversaro) : We should teach toEigen to handle empty matrices correctly?
    // relevant: https://forum.kde.org/viewtopic.php?f=74&t=107974
    if (genForces.jointTorques().size() > 0)
    {
        concat << toEigen(genForces.baseWrench()), toEigen(genForces.jointTorques());
    } else
    {
        concat = toEigen(genForces.baseWrench());
    }
    return concat;
}

inline iDynTree::SpatialAcc Vector6ToSpatialAcc(const iDynTree::Vector6& vec6)
{
    SpatialAcc ret;
    ret(0) = vec6(0);
    ret(1) = vec6(1);
    ret(2) = vec6(2);
    ret(3) = vec6(3);
    ret(4) = vec6(4);
    ret(5) = vec6(5);
    return ret;
}

inline iDynTree::SpatialAcc Vector3ToSpatialAcc(const iDynTree::Vector3& vec6)
{
    SpatialAcc ret;
    ret(0) = vec6(0);
    ret(1) = vec6(1);
    ret(2) = vec6(2);
    ret(3) = 0.0;
    ret(4) = 0.0;
    ret(5) = 0.0;
    return ret;
}

// Test different ways of computing inverse dynamics
void testInverseDynamics(KinDynComputations& dynComp)
{
    int dofs = dynComp.getNrOfDegreesOfFreedom();
    iDynTree::Vector6 baseAcc;
    iDynTree::JointDOFsDoubleArray shapeAccs(dynComp.model());

    iDynTree::LinkNetExternalWrenches netExternalWrenches(dynComp.model());
    for (unsigned int link = 0; link < dynComp.model().getNrOfLinks(); link++)
    {
        netExternalWrenches(link) = getRandomWrench();
    }

    for (int i = 0; i < 6; i++)
    {
        baseAcc(i) = real_random_double();
    }

    for (size_t dof = 0; dof < dofs; dof++)

    {
        shapeAccs(dof) = random_double();
    }

    {
        FreeFloatingGeneralizedTorques invDynForces(dynComp.model());
        FreeFloatingGeneralizedTorques massMatrixInvDynForces(dynComp.model());
        FreeFloatingGeneralizedTorques regressorInvDynForces(dynComp.model());

        // Run classical inverse dynamics
        bool ok = dynComp.inverseDynamics(baseAcc, shapeAccs, netExternalWrenches, invDynForces);
        ASSERT_IS_TRUE(ok);

        // Run inverse dynamics with mass matrix
        FreeFloatingMassMatrix massMatrix(dynComp.model());
        ok = dynComp.getFreeFloatingMassMatrix(massMatrix);
        ASSERT_IS_TRUE(ok);

        FreeFloatingGeneralizedTorques invDynBiasForces(dynComp.model());
        ok = dynComp.generalizedBiasForces(invDynBiasForces);
        ASSERT_IS_TRUE(ok);

        FreeFloatingGeneralizedTorques invDynExtForces(dynComp.model());
        ok = dynComp.generalizedExternalForces(netExternalWrenches, invDynExtForces);
        ASSERT_IS_TRUE(ok);

        VectorDynSize massMatrixInvDynForcesContinuous(6 + dofs);
        toEigen(massMatrixInvDynForcesContinuous)
            = toEigen(massMatrix) * toEigen(baseAcc, shapeAccs) + toEigen(invDynBiasForces)
              + toEigen(invDynExtForces);
        toEigen(massMatrixInvDynForces.baseWrench().getLinearVec3())
            = toEigen(massMatrixInvDynForcesContinuous).segment<3>(0);
        toEigen(massMatrixInvDynForces.baseWrench().getAngularVec3())
            = toEigen(massMatrixInvDynForcesContinuous).segment<3>(3);
        toEigen(massMatrixInvDynForces.jointTorques())
            = toEigen(massMatrixInvDynForcesContinuous).segment(6, dofs);

        ASSERT_EQUAL_SPATIAL_FORCE(massMatrixInvDynForces.baseWrench(), invDynForces.baseWrench());
        ASSERT_EQUAL_VECTOR(massMatrixInvDynForces.jointTorques(), invDynForces.jointTorques());

        // Run inverse dynamics with regressor matrix
        MatrixDynSize regressor(dynComp.model().getNrOfDOFs() + 6,
                                10 * dynComp.model().getNrOfLinks());
        ok = dynComp.inverseDynamicsInertialParametersRegressor(baseAcc, shapeAccs, regressor);
        ASSERT_IS_TRUE(ok);

        VectorDynSize inertialParams(10 * dynComp.model().getNrOfLinks());
        ok = dynComp.model().getInertialParameters(inertialParams);
        ASSERT_IS_TRUE(ok);

        VectorDynSize regressorInvDynForcesContinuous(6 + dofs);
        toEigen(regressorInvDynForcesContinuous)
            = toEigen(regressor) * toEigen(inertialParams) + toEigen(invDynExtForces);
        toEigen(regressorInvDynForces.baseWrench().getLinearVec3())
            = toEigen(regressorInvDynForcesContinuous).segment<3>(0);
        toEigen(regressorInvDynForces.baseWrench().getAngularVec3())
            = toEigen(regressorInvDynForcesContinuous).segment<3>(3);
        toEigen(regressorInvDynForces.jointTorques())
            = toEigen(regressorInvDynForcesContinuous).segment(6, dofs);

        ASSERT_EQUAL_SPATIAL_FORCE(regressorInvDynForces.baseWrench(), invDynForces.baseWrench());
        ASSERT_EQUAL_VECTOR(regressorInvDynForces.jointTorques(), invDynForces.jointTorques());

        // Run inverse dynamics with regressor matrix with internal joint force/torque
        iDynTree::FreeFloatingGeneralizedTorques invDynForcesWithInternal(dynComp.model());
        iDynTree::LinkInternalWrenches linkInternalWrenches(dynComp.model());

        ok = dynComp.inverseDynamicsWithInternalJointForceTorques(baseAcc,
                                                                  shapeAccs,
                                                                  netExternalWrenches,
                                                                  invDynForcesWithInternal,
                                                                  linkInternalWrenches);
        ASSERT_IS_TRUE(ok);

        // To validate linkInternalWrenches, we use those forces to verify that Newton-Euler
        // equations holds true for each link In particular, we build the three terms of
        // Equation 4.17 in
        // https://traversaro.github.io/traversaro-phd-thesis/traversaro-phd-thesis.pdf:
        //
        // \f$ {}_C \phi_L = {}_C \mathrm{f}^{x}_L + \sum_{D  \in \aleph{L}} {}_C \mathrm{f}_{D,L}
        // \f$
        //
        // inertial "force/torque": \f$ {}_C \phi_L =  \f$ : the term that depend on inertial
        // parameters, including the gravitational force. external force/torque: \f$ {}_C
        // \mathrm{f}^{x}_L \f$ : the net external force/torque that the external environment
        // excerts on the link L. internal force/torque: \f$ \sum_{D  \in \aleph{L}} {}_C
        // \mathrm{f}_{D,L} \f$ : the net (i.e. sum of all) internal force/torque
        //                                           that the other neighbor links (contained in the
        //                                           set $\aleph{L}$) excert on link L via the
        //                                           joints.
        // The main difference is that in Equation 4.17 the quantities are computed in the frame L
        // of the link, while in this test for consistency with the quantities contained in
        // linkInternalWrenches and the rest of results of KinDynComputations, we will use a
        // different frame depending on the getFrameVelocityRepresentation used:
        //
        // |`FrameVelocityRepresentation`     |       C      |
        // |:--------------------------------:|:------------:|
        // | `MIXED_REPRESENTATION` (default) | \f$ L[A] \f$ |
        // | `BODY_FIXED_REPRESENTATION`      | \f$   L  \f$ |
        // | `INERTIAL_FIXED_REPRESENTATION`  | \f$   A \f$  |

        // The inertial term is a complicated to compute in mixed representation, so for now we
        // test only BODY_FIXED_REPRESENTATION or INERTIAL_FIXED_REPRESENTATION
        if (dynComp.getFrameVelocityRepresentation() == MIXED_REPRESENTATION)
        {
            return;
        }

        iDynTree::LinkWrenches inertialWrenches(dynComp.model());
        iDynTree::LinkWrenches externalWrenches(dynComp.model());
        iDynTree::LinkWrenches internalWrenches(dynComp.model());

        iDynTree::VectorDynSize dummyJointPos, dummyJointVel;
        iDynTree::Vector3 gravityInAbsoluteFrame;
        dummyJointPos.resize(dynComp.model().getNrOfPosCoords());
        dummyJointVel.resize(dynComp.model().getNrOfDOFs());
        dynComp.getRobotState(dummyJointPos, dummyJointVel, gravityInAbsoluteFrame);

        LinkIndex baseLnkIdx = dynComp.getRobotModel().getLinkIndex(dynComp.getFloatingBase());

        for (LinkIndex lnkIdx = 0; lnkIdx < dynComp.getNrOfLinks(); lnkIdx++)
        {
            // Compute inertialWrenches
            // Inertia is always expressed in L, let's convert it in A if we are in
            // INERTIAL_FIXED_REPRESENTATION
            iDynTree::SpatialInertia rawInertia
                = dynComp.getRobotModel().getLink(lnkIdx)->getInertia();
            iDynTree::SpatialInertia I;
            if (dynComp.getFrameVelocityRepresentation() == INERTIAL_FIXED_REPRESENTATION)
            {
                I = dynComp.getWorldTransform(lnkIdx) * rawInertia;
            } else
            {
                // BODY_FIXED_REPRESENTATION
                I = rawInertia;
            }

            // Not really efficient, but ok as we are in a test
            iDynTree::SpatialAcc a
                = Vector6ToSpatialAcc(dynComp.getFrameAcc(lnkIdx, baseAcc, shapeAccs));
            iDynTree::Twist v = dynComp.getFrameVel(lnkIdx);
            // Add "gravitational" force
            iDynTree::Wrench gravitationalWrench;
            if (dynComp.getFrameVelocityRepresentation() == INERTIAL_FIXED_REPRESENTATION)
            {
                gravitationalWrench = (I * Vector3ToSpatialAcc(gravityInAbsoluteFrame));
            } else
            {
                // BODY_FIXED_REPRESENTATION
                gravitationalWrench = (I
                                       * (dynComp.getWorldTransform(lnkIdx).inverse()
                                          * Vector3ToSpatialAcc(gravityInAbsoluteFrame)));
            }
            inertialWrenches(lnkIdx) = I * a + v * (I * v) - gravitationalWrench;

            // Compute external wrenches (it was already an input of inverseDynamics)
            externalWrenches(lnkIdx) = netExternalWrenches(lnkIdx);

            // Compute internal wrenches
            // We start from the wrench applied by the parent link on the link lnkIdx
            // {}_{C} \mathrm{f}_{\lambda(L), L}
            internalWrenches(lnkIdx) = linkInternalWrenches(lnkIdx);

            // Then we add the link that the link applies on its children
            // rotating them appropriately and inverting their direction
            const iDynTree::Model& model = dynComp.getRobotModel();
            iDynTree::Traversal traversal;
            model.computeFullTreeTraversal(traversal,
                                           model.getLinkIndex(dynComp.getFloatingBase()));
            const Link* parentLink = traversal.getParentLinkFromLinkIndex(lnkIdx);
            for (unsigned int neigh_i = 0; neigh_i < model.getNrOfNeighbors(lnkIdx); neigh_i++)
            {
                LinkIndex neighborIndex = model.getNeighbor(lnkIdx, neigh_i).neighborLink;
                // The child links are the neighbor links that are not parent
                if (!parentLink || neighborIndex != parentLink->getIndex())
                {
                    LinkIndex childIndex = neighborIndex;
                    Transform link_X_child;
                    if (dynComp.getFrameVelocityRepresentation() == INERTIAL_FIXED_REPRESENTATION)
                    {
                        // In inertial fixed representation, all joint internal wrenches are
                        // expressed in A
                        link_X_child = iDynTree::Transform::Identity();
                    } else
                    {
                        // BODY_FIXED_REPRESENTATION
                        link_X_child = dynComp.getRelativeTransform(lnkIdx, childIndex);
                    }
                    iDynTree::Wrench wrenchAppliedByChildOnLinkInLinkFrame
                        = -(link_X_child * linkInternalWrenches(childIndex));
                    internalWrenches(lnkIdx)
                        = internalWrenches(lnkIdx) + wrenchAppliedByChildOnLinkInLinkFrame;
                }
            }

            // If the link is the base link, we need to add to the external wrenches the baseWrench
            // of the generalized torques This happens because the random-generated external forces
            // and acceleration may not be "consistent" according to definition 4.1 of
            // https://traversaro.github.io/traversaro-phd-thesis/traversaro-phd-thesis.pdf
            if (baseLnkIdx == lnkIdx)
            {
                externalWrenches(lnkIdx)
                    = externalWrenches(lnkIdx) + invDynForcesWithInternal.baseWrench();
            }

            // Validate Netwon-Euler Equations
            // inertial = external + internal
            ASSERT_EQUAL_SPATIAL_FORCE(inertialWrenches(lnkIdx),
                                       externalWrenches(lnkIdx) + internalWrenches(lnkIdx));
        }
    }
}

void testRelativeJacobians(KinDynComputations& dynComp)
{
    if (dynComp.getNrOfLinks() < 2)
        return;
    FrameIndex frame = -1;
    FrameIndex refFrame = -1;

    if (dynComp.getNrOfLinks() == 2)
    {
        frame = 0;
        refFrame = 1;
    } else
    {
        // Pick two frames at random
        frame = real_random_int(0, dynComp.getNrOfFrames());
        // be sure to pick two different frames
        do
        {
            refFrame = real_random_int(0, dynComp.getNrOfFrames());
        } while (refFrame == frame && frame >= 0);
    }

    FrameVelocityRepresentation representation = dynComp.getFrameVelocityRepresentation();
    dynComp.setFrameVelocityRepresentation(MIXED_REPRESENTATION);

    // Compute the relative Jacobian
    iDynTree::MatrixDynSize relativeJacobian(6, dynComp.getNrOfDegreesOfFreedom());
    dynComp.getRelativeJacobian(refFrame, frame, relativeJacobian);

    iDynTree::VectorDynSize qj(dynComp.getNrOfDegreesOfFreedom()),
        dqj(dynComp.getNrOfDegreesOfFreedom());
    Vector3 gravity;
    dynComp.getRobotState(qj, dqj, gravity);

    Twist relativeVel;
    Eigen::Matrix<double, 6, 1> relativeVelTemp;
    relativeVelTemp = toEigen(relativeJacobian) * toEigen(dqj);
    fromEigen(relativeVel, relativeVelTemp);

    // this velocity depends on where the Jacobian is expressed

    Twist frameVel = dynComp.getFrameVel(frame);
    Twist refFrameVel = dynComp.getFrameVel(refFrame);

    if (dynComp.getFrameVelocityRepresentation() == INERTIAL_FIXED_REPRESENTATION)
    {
        // Inertial = right trivialized.
        // frameVel is written wrt A
        // refFrameVel is written wrt A
        // relativeJacobian is written wrt refFrame
        Eigen::Matrix<double, 6, 1> temp
            = toEigen(dynComp.getWorldTransform(refFrame).asAdjointTransform())
              * toEigen(relativeVel);
        fromEigen(relativeVel, temp);
    } else if (dynComp.getFrameVelocityRepresentation() == BODY_FIXED_REPRESENTATION)
    {
        // BODY = left trivialized.
        // frameVel is written wrt frame
        // refFrameVel is written wrt refFrame
        // relativeJacobian is written wrt frame
        // convert refFrameVel to frame
        Eigen::Matrix<double, 6, 1> temp
            = toEigen(dynComp.getRelativeTransform(frame, refFrame).asAdjointTransform())
              * toEigen(refFrameVel);
        fromEigen(refFrameVel, temp);
    } else if (dynComp.getFrameVelocityRepresentation() == MIXED_REPRESENTATION)
    {
        // MIXED
        // frameVel is written wrt frame, [A]
        // refFrameVel is written wrt refFrame, [A]
        // relativeJacobian is written wrt frame, [refFrame]
        // convert refFrameVel to frame, [A]
        Transform frame_A_H_frame(dynComp.getWorldTransform(frame).getRotation(), Position::Zero());

        // refFrameVel = ref_[A]_v_ref, I want frame_[A]_v_ref. As I do not have an explicit A frame
        // I do the following:
        //  frame_[A]_H_frame_[frame] * frame_[frame]_H_ref_[frame] * ref_[frame]_H_ref_[A] *
        //  ref_[A]_v_ref
        Eigen::Matrix<double, 6, 1> temp
            = toEigen((frame_A_H_frame
                       * dynComp.getRelativeTransformExplicit(frame, frame, refFrame, frame)
                       * frame_A_H_frame.inverse())
                          .asAdjointTransform())
              * toEigen(refFrameVel);
        fromEigen(refFrameVel, temp);
        // and relativeVel to frame [A]
        temp = toEigen((frame_A_H_frame
                        * dynComp.getRelativeTransformExplicit(frame, frame, frame, refFrame))
                           .asAdjointTransform())
               * toEigen(relativeVel);
        fromEigen(relativeVel, temp);
    }

    // now compute the error velocity
    // now they should be expressed in the same frame
    Twist velDifference = frameVel - refFrameVel;
    ASSERT_EQUAL_VECTOR(velDifference, relativeVel);

    dynComp.setFrameVelocityRepresentation(representation);
}

// Dummy test: for now it just prints the frameBiasAcc, to check there is no
// usage of not initialized memory
void testAbsoluteJacobiansAndFrameBiasAcc(KinDynComputations& dynComp)
{
    FrameIndex frame = real_random_int(0, dynComp.getNrOfFrames());

    // Test Jacobian consistency

    // Get robot velocity
    iDynTree::VectorDynSize nu(6 + dynComp.getNrOfDegreesOfFreedom());
    dynComp.getModelVel(nu);

    // Compute frame velocity and jacobian
    Twist frameVel = dynComp.getFrameVel(frame);
    Vector6 frameVelJac;
    FrameFreeFloatingJacobian jac(dynComp.model());
    dynComp.getFrameFreeFloatingJacobian(frame, jac);
    toEigen(frameVelJac) = toEigen(jac) * toEigen(nu);

    ASSERT_EQUAL_VECTOR(frameVel.asVector(), frameVelJac);

    std::cerr << "nu: " << nu.toString() << std::endl;

    // Compute frame acceleration, and bias acceleration
    Vector6 baseAcc;
    baseAcc.zero();
    baseAcc(0) = 1;
    baseAcc(1) = 0;
    baseAcc(2) = 0;
    baseAcc(3) = 0;
    baseAcc(4) = 0;
    baseAcc(5) = 0;

    VectorDynSize sddot(dynComp.model().getNrOfDOFs());
    for (int i = 0; i < sddot.size(); i++)
    {
        sddot(i) = i * 0.1;
    }

    iDynTree::VectorDynSize nuDot(6 + dynComp.getNrOfDegreesOfFreedom());
    toEigen(nuDot) = toEigen(baseAcc, sddot);

    Vector6 frameAcc = dynComp.getFrameAcc(frame, baseAcc, sddot);
    Vector6 biasAcc = dynComp.getFrameBiasAcc(frame);
    Vector6 frameAccJac;
    toEigen(frameAccJac) = toEigen(jac) * toEigen(nuDot) + toEigen(biasAcc);

    ASSERT_EQUAL_VECTOR(frameAcc, frameAccJac);
}

void testInverseDynamicsWithInternalJointForceTorques(KinDynComputations& dynComp)
{
    // Comute inverseDynamicsWithInternalJointForceTorques
    iDynTree::LinkInternalWrenches linkInternalWrenches(dynComp.model());
}

bool isStringInVector(const std::string& str, const std::vector<std::string>& vec)
{
    return std::find(vec.begin(), vec.end(), str) != vec.end();
}

void getRandomSubsetOfJoints(const Model& model,
                             size_t nrOfJointsInSubset,
                             std::vector<std::string>& subsetOfJoints)
{
    while (subsetOfJoints.size() < nrOfJointsInSubset)
    {
        JointIndex randomJoint = (JointIndex)getRandomInteger(0, model.getNrOfJoints() - 1);
        std::string randomJointName = model.getJointName(randomJoint);

        // If the random added joint is not in the vector, add it
        if (!isStringInVector(randomJointName, subsetOfJoints))
        {
            subsetOfJoints.push_back(randomJointName);
        }
    }
}

void testSubModelConsistency(std::string modelFilePath,
                             const FrameVelocityRepresentation frameVelRepr)
{
    iDynTree::ModelLoader mdlLoader;
    bool ok = mdlLoader.loadModelFromFile(modelFilePath);
    ASSERT_IS_TRUE(ok);

    iDynTree::KinDynComputations dynCompFullModel;
    ok = dynCompFullModel.loadRobotModel(mdlLoader.model());
    ASSERT_IS_TRUE(ok);

    ok = dynCompFullModel.setFrameVelocityRepresentation(frameVelRepr);
    ASSERT_IS_TRUE(ok);

    setRandomState(dynCompFullModel);

    Model fullModel = mdlLoader.model();

    int dofsFullModel = fullModel.getNrOfDOFs();

    for (size_t jnts = 1; jnts < fullModel.getNrOfJoints(); jnts += 5)
    {
        Traversal traversal;
        ok = fullModel.computeFullTreeTraversal(traversal);

        ASSERT_IS_TRUE(ok);

        // Define set of joints which split the model in submodels
        std::vector<std::string> jointInReducedModel;
        getRandomSubsetOfJoints(fullModel, jnts, jointInReducedModel);

        iDynTree::SubModelDecomposition subModels;
        ok = subModels.splitModelAlongJoints(fullModel, traversal, jointInReducedModel);
        ASSERT_IS_TRUE(ok);

        for (int subModelIdx = 0; subModelIdx < subModels.getNrOfSubModels(); subModelIdx++)
        {
            const Traversal& subModelTraversal = subModels.getTraversal(subModelIdx);

            Model reducedModel;
            ok = extractSubModel(fullModel, subModelTraversal, reducedModel);
            ASSERT_IS_TRUE(ok);

            iDynTree::KinDynComputations dynCompReducedModel;
            ok = dynCompReducedModel.loadRobotModel(reducedModel);
            ASSERT_IS_TRUE(ok);

            ok = dynCompReducedModel.setFrameVelocityRepresentation(frameVelRepr);
            ASSERT_IS_TRUE(ok);

            // Get robot state from kinDynFullModel
            iDynTree::VectorDynSize qjFullModel(dofsFullModel), dqjFullModel(dofsFullModel);
            Vector3 gravity;
            Transform worldTbaseFullModel;
            Twist baseVelFullModel;
            dynCompFullModel.getRobotState(worldTbaseFullModel,
                                           qjFullModel,
                                           baseVelFullModel,
                                           dqjFullModel,
                                           gravity);

            // Get transform between base of full model and base of reduced model
            Transform baseFullTbaseReduced;
            baseFullTbaseReduced
                = dynCompFullModel.getWorldTransform(subModelTraversal.getLink(0)->getIndex());
            Twist baseVelReducedModel;
            baseVelReducedModel
                = dynCompFullModel.getFrameVel(subModelTraversal.getLink(0)->getIndex());

            // Find indeces of fullModel joints corresponding to subModel joints
            VectorDynSize idxJntReducedModelInFullModel(reducedModel.getNrOfJoints());

            for (JointIndex jntIdx = 0; jntIdx < reducedModel.getNrOfJoints(); jntIdx++)
            {
                std::string jntName = reducedModel.getJointName(jntIdx);

                idxJntReducedModelInFullModel(jntIdx) = fullModel.getJointIndex(jntName);
            }

            // Get q and dq from kinDynFullModel of joints contained in subModel
            VectorDynSize qReducedModel(reducedModel.getNrOfDOFs()),
                dqReducedModel(reducedModel.getNrOfDOFs());

            for (JointIndex jntIdx = 0; jntIdx < reducedModel.getNrOfJoints(); jntIdx++)
            {
                size_t posCoordsOffsetReducedModel
                    = reducedModel.getJoint(jntIdx)->getPosCoordsOffset();
                JointIndex jntIdxFullModel = idxJntReducedModelInFullModel(jntIdx);
                size_t posCoordsOffsetFullModel
                    = fullModel.getJoint(jntIdxFullModel)->getPosCoordsOffset();
                for (size_t localPosCoords = 0;
                     localPosCoords < reducedModel.getJoint(jntIdx)->getNrOfPosCoords();
                     localPosCoords++)
                {
                    qReducedModel(posCoordsOffsetReducedModel + localPosCoords)
                        = qjFullModel(posCoordsOffsetFullModel + localPosCoords);
                }
            }

            for (JointIndex jntIdx = 0; jntIdx < reducedModel.getNrOfJoints(); jntIdx++)
            {
                size_t dofsOffsetReducedModel = reducedModel.getJoint(jntIdx)->getDOFsOffset();
                JointIndex jntIdxFullModel = idxJntReducedModelInFullModel(jntIdx);
                size_t dofsOffsetFullModel = fullModel.getJoint(jntIdxFullModel)->getDOFsOffset();
                for (size_t localDofs = 0; localDofs < reducedModel.getJoint(jntIdx)->getNrOfDOFs();
                     localDofs++)
                {
                    dqReducedModel(dofsOffsetReducedModel + localDofs)
                        = dqjFullModel(dofsOffsetFullModel + localDofs);
                }
            }

            ok = dynCompReducedModel.setRobotState(baseFullTbaseReduced,
                                                   qReducedModel,
                                                   baseVelReducedModel,
                                                   dqReducedModel,
                                                   gravity);
            ASSERT_IS_TRUE(ok);

            // Compare pose and velocity per each link from full model and from reduced model
            for (FrameIndex frameIdxReducedModel = 0;
                 frameIdxReducedModel < reducedModel.getNrOfFrames();
                 frameIdxReducedModel++)
            {
                std::string frameName = reducedModel.getFrameName(frameIdxReducedModel);

                int frameIdxFullModel = fullModel.getFrameIndex(frameName);

                // Frame pose from full model
                Transform framePoseFullModel
                    = dynCompFullModel.getWorldTransform(frameIdxFullModel);

                // Frame pose from reduced model
                Transform framePoseReducedModel
                    = dynCompReducedModel.getWorldTransform(frameIdxReducedModel);

                ASSERT_EQUAL_TRANSFORM(framePoseFullModel, framePoseReducedModel);

                // Frame velicity from full model
                Twist frameVelFullModel = dynCompFullModel.getFrameVel(frameName);

                // Frame velocity from reduced model
                Twist frameVelReducedModel = dynCompReducedModel.getFrameVel(frameName);

                ASSERT_EQUAL_VECTOR(frameVelFullModel, frameVelReducedModel);
            }
        }
    }
}

void testModelConsistency(std::string modelFilePath, const FrameVelocityRepresentation frameVelRepr)
{
    iDynTree::KinDynComputations dynComp;
    iDynTree::ModelLoader mdlLoader;
    bool ok = mdlLoader.loadModelFromFile(modelFilePath);
    ok = ok && dynComp.loadRobotModel(mdlLoader.model());
    ASSERT_IS_TRUE(ok);

    ok = dynComp.setFrameVelocityRepresentation(frameVelRepr);
    ASSERT_IS_TRUE(ok);

    for (int i = 0; i < 1; i++)
    {
        setRandomState(dynComp);
        testRelativeTransform(dynComp);
        testAverageVelocityAndTotalMomentumJacobian(dynComp);
        testInverseDynamics(dynComp);
        testRelativeJacobians(dynComp);
        testAbsoluteJacobiansAndFrameBiasAcc(dynComp);
    }
}

void testSubModelConsistencyAllRepresentations(std::string modelName)
{
    std::string urdfFileName = getAbsModelPath(modelName);
    std::cout << "Testing file " << urdfFileName << std::endl;
    std::cout << "Testing MIXED_REPRESENTATION " << urdfFileName << std::endl;
    testSubModelConsistency(urdfFileName, iDynTree::MIXED_REPRESENTATION);
    std::cout << "Testing BODY_FIXED_REPRESENTATION " << urdfFileName << std::endl;
    testSubModelConsistency(urdfFileName, iDynTree::BODY_FIXED_REPRESENTATION);
    std::cout << "Testing INERTIAL_FIXED_REPRESENTATION " << urdfFileName << std::endl;
    testSubModelConsistency(urdfFileName, iDynTree::INERTIAL_FIXED_REPRESENTATION);
}

/**
 * Test that verifies the mass matrix via kinetic energy computation.
 *
 * The kinetic energy is computed in two ways:
 * 1. Via body 6D velocities: KE = 0.5 * sum_l v_l^T * I_l * v_l
 *    where v_l is the body-fixed velocity of link l, and I_l is the spatial inertia of link l
 * 2. Via the mass matrix: KE = 0.5 * nu^T * M(q) * nu
 *    where nu is the generalized velocity vector and M(q) is the mass matrix
 *
 * Both methods should yield the same kinetic energy value.
 */
void testKineticEnergyConsistency(KinDynComputations& dynComp)
{
    const Model& model = dynComp.model();
    size_t nrOfLinks = model.getNrOfLinks();
    size_t nrOfDofs = dynComp.getNrOfDegreesOfFreedom();

    // Store the original frame velocity representation
    FrameVelocityRepresentation origRepr = dynComp.getFrameVelocityRepresentation();

    // Use BODY_FIXED representation for computing link velocities
    // This is because the spatial inertia I_l is expressed in the link frame,
    // and we need the velocity to be expressed in the same frame for v^T * I * v
    dynComp.setFrameVelocityRepresentation(BODY_FIXED_REPRESENTATION);

    // Method 1: Compute kinetic energy via body velocities
    // KE = 0.5 * sum_l v_l^T * I_l * v_l
    double kineticEnergyFromBodyVelocities = 0.0;

    for (LinkIndex linkIdx = 0; linkIdx < static_cast<LinkIndex>(nrOfLinks); linkIdx++)
    {
        // Get the body-fixed velocity of the link
        // In BODY_FIXED representation, getFrameVel returns the velocity expressed in the link
        // frame
        Twist linkVel = dynComp.getFrameVel(linkIdx);

        // Get the spatial inertia of the link (expressed in the link frame)
        const SpatialInertia& linkInertia = model.getLink(linkIdx)->getInertia();

        // Compute v^T * I * v using the 6x6 matrix representation
        // The spatial inertia matrix M is such that momentum h = M * v
        // So kinetic energy for this link is 0.5 * v^T * M * v
        Eigen::Matrix<double, 6, 6> inertiaMatrix = toEigen(linkInertia.asMatrix());
        Eigen::Matrix<double, 6, 1> velEigen = toEigen(linkVel);

        double linkKE = 0.5 * velEigen.transpose() * inertiaMatrix * velEigen;
        kineticEnergyFromBodyVelocities += linkKE;
    }

    // Method 2: Compute kinetic energy via mass matrix
    // KE = 0.5 * nu^T * M(q) * nu
    MatrixDynSize massMatrix(6 + nrOfDofs, 6 + nrOfDofs);
    bool ok = dynComp.getFreeFloatingMassMatrix(massMatrix);
    ASSERT_IS_TRUE(ok);

    // Get the generalized velocity nu = [baseVel; jointVel]
    VectorDynSize nu(6 + nrOfDofs);
    ok = dynComp.getModelVel(nu);
    ASSERT_IS_TRUE(ok);

    // Compute kinetic energy: 0.5 * nu^T * M * nu
    double kineticEnergyFromMassMatrix
        = 0.5 * toEigen(nu).transpose() * toEigen(massMatrix) * toEigen(nu);

    // Compare the two kinetic energies
    // Use a relative tolerance since the energies can be large
    double tolerance = 1e-8 * std::max(1.0, std::abs(kineticEnergyFromMassMatrix));
    ASSERT_EQUAL_DOUBLE_TOL(kineticEnergyFromBodyVelocities,
                            kineticEnergyFromMassMatrix,
                            tolerance);

    // Restore the original frame velocity representation
    dynComp.setFrameVelocityRepresentation(origRepr);
}

/**
 * Test kinetic energy consistency on a random model with various joint types.
 */
void testKineticEnergyOnRandomModels()
{
    std::cout << "Testing kinetic energy consistency on random models..." << std::endl;

    // Test with different model sizes
    for (unsigned int nrOfJoints = 1; nrOfJoints <= 20; nrOfJoints += 5)
    {
        std::cout << "  Testing random model with " << nrOfJoints << " joints..." << std::endl;

        // Create a random model with revolute, spherical, and other joint types
        Model randomModel
            = getRandomModel(nrOfJoints,
                             5,
                             SIMPLE_JOINT_TYPES | JOINT_REVOLUTE_SO2 | JOINT_SPHERICAL);

        KinDynComputations dynComp;
        bool ok = dynComp.loadRobotModel(randomModel);
        ASSERT_IS_TRUE(ok);

        // Test multiple random configurations
        for (int iter = 0; iter < 5; iter++)
        {
            setRandomState(dynComp);
            testKineticEnergyConsistency(dynComp);
        }
    }
}

/**
 * Test kinetic energy consistency on URDF models loaded from files.
 */
void testKineticEnergyOnURDFModel(std::string modelFilePath)
{
    iDynTree::ModelLoader mdlLoader;
    bool ok = mdlLoader.loadModelFromFile(modelFilePath);
    ASSERT_IS_TRUE(ok);

    KinDynComputations dynComp;
    ok = dynComp.loadRobotModel(mdlLoader.model());
    ASSERT_IS_TRUE(ok);

    // Test multiple random configurations
    for (int iter = 0; iter < 5; iter++)
    {
        setRandomState(dynComp);
        testKineticEnergyConsistency(dynComp);
    }
}

void testModelConsistencyAllRepresentations(std::string modelName)
{
    std::string urdfFileName = getAbsModelPath(modelName);
    std::cout << "Testing file " << urdfFileName << std::endl;
    std::cout << "Testing MIXED_REPRESENTATION " << urdfFileName << std::endl;
    testModelConsistency(urdfFileName, iDynTree::MIXED_REPRESENTATION);
    std::cout << "Testing BODY_FIXED_REPRESENTATION " << urdfFileName << std::endl;
    testModelConsistency(urdfFileName, iDynTree::BODY_FIXED_REPRESENTATION);
    std::cout << "Testing INERTIAL_FIXED_REPRESENTATION " << urdfFileName << std::endl;
    testModelConsistency(urdfFileName, iDynTree::INERTIAL_FIXED_REPRESENTATION);
}

void testRelativeJacobianSparsity(KinDynComputations& dynComp)
{
    // take two frames
    if (dynComp.getNrOfLinks() < 2)
        return;
    FrameIndex frame = -1;
    FrameIndex refFrame = -1;

    if (dynComp.getNrOfLinks() == 2)
    {
        frame = 0;
        refFrame = 1;
    } else
    {
        // Pick two frames at random
        frame = real_random_int(0, dynComp.getNrOfFrames());
        // be sure to pick two different frames
        do
        {
            refFrame = real_random_int(0, dynComp.getNrOfFrames());
        } while (refFrame == frame && frame >= 0);
    }

    // get sparsity pattern
    MatrixDynSize jacobianPattern(6, dynComp.getNrOfDegreesOfFreedom());
    bool ok = dynComp.getRelativeJacobianSparsityPattern(refFrame, frame, jacobianPattern);
    ASSERT_IS_TRUE(ok);

    // Now computes the actual jacobian
    MatrixDynSize jacobian(6, dynComp.getNrOfDegreesOfFreedom());
    ok = dynComp.getRelativeJacobian(refFrame, frame, jacobian);
    ASSERT_IS_TRUE(ok);

    ASSERT_IS_TRUE(jacobian.rows() == jacobianPattern.rows() && jacobian.cols() == jacobian.cols());

    for (size_t row = 0; row < jacobian.rows(); ++row)
    {
        for (size_t col = 0; col < jacobian.cols(); ++col)
        {
            // if jacobian has value != 0, pattern should have 1
            if (std::abs(jacobian(row, col)) > iDynTree::DEFAULT_TOL)
            {
                ASSERT_EQUAL_DOUBLE(jacobianPattern(row, col), 1.0);
            }
        }
    }
}

void testAbsoluteJacobianSparsity(KinDynComputations& dynComp)
{
    // take one frames
    if (dynComp.getNrOfLinks() < 1)
        return;
    FrameIndex frame = -1;

    if (dynComp.getNrOfLinks() == 1)
    {
        frame = 0;
    } else
    {
        // Pick one frames at random
        frame = real_random_int(0, dynComp.getNrOfFrames());
    }

    // get sparsity pattern
    MatrixDynSize jacobianPattern(6, 6 + dynComp.getNrOfDegreesOfFreedom());
    bool ok = dynComp.getFrameFreeFloatingJacobianSparsityPattern(frame, jacobianPattern);
    ASSERT_IS_TRUE(ok);

    // Now computes the actual jacobian
    MatrixDynSize jacobian(6, 6 + dynComp.getNrOfDegreesOfFreedom());
    ok = dynComp.getFrameFreeFloatingJacobian(frame, jacobian);
    ASSERT_IS_TRUE(ok);

    ASSERT_IS_TRUE(jacobian.rows() == jacobianPattern.rows() && jacobian.cols() == jacobian.cols());

    for (size_t row = 0; row < jacobian.rows(); ++row)
    {
        for (size_t col = 0; col < jacobian.cols(); ++col)
        {
            // if jacobian has value != 0, pattern should have 1
            if (std::abs(jacobian(row, col)) > iDynTree::DEFAULT_TOL)
            {
                ASSERT_EQUAL_DOUBLE(jacobianPattern(row, col), 1.0);
            }
        }
    }
}

void testSparsityPattern(std::string modelFilePath, const FrameVelocityRepresentation frameVelRepr)
{
    iDynTree::KinDynComputations dynComp;
    iDynTree::ModelLoader mdlLoader;
    bool ok = mdlLoader.loadModelFromFile(modelFilePath);
    ok = ok && dynComp.loadRobotModel(mdlLoader.model());
    ASSERT_IS_TRUE(ok);

    ok = dynComp.setFrameVelocityRepresentation(frameVelRepr);
    ASSERT_IS_TRUE(ok);

    for (int i = 0; i < 10; i++)
    {
        setRandomState(dynComp);
        testRelativeJacobianSparsity(dynComp);
        testAbsoluteJacobianSparsity(dynComp);
    }
}

void testSparsityPatternAllRepresentations(std::string modelName)
{
    std::string urdfFileName = getAbsModelPath(modelName);
    std::cout << "Testing file " << urdfFileName << std::endl;
    testSparsityPattern(urdfFileName, iDynTree::MIXED_REPRESENTATION);
    testSparsityPattern(urdfFileName, iDynTree::BODY_FIXED_REPRESENTATION);
    testSparsityPattern(urdfFileName, iDynTree::INERTIAL_FIXED_REPRESENTATION);
}

void testFloatingBaseFrameConsistency(std::string modelFilePath,
                                      const FrameVelocityRepresentation frameVelRepr)
{
    iDynTree::KinDynComputations dynComp;
    iDynTree::ModelLoader mdlLoader;
    bool ok = mdlLoader.loadModelFromFile(modelFilePath);
    ok = ok && dynComp.loadRobotModel(mdlLoader.model());
    ASSERT_IS_TRUE(ok);

    ok = dynComp.setFrameVelocityRepresentation(frameVelRepr);
    ASSERT_IS_TRUE(ok);

    // Test with iCub model using l_foot as base link vs l_sole as base frame
    std::string baseLinkName = "l_foot";
    std::string baseFrameName = "l_sole";
    std::string rootLinkName = "root_link";

    // Check if both frames exist in the model
    int baseLinkIndex = dynComp.model().getFrameIndex(baseLinkName);
    int baseFrameIndex = dynComp.model().getFrameIndex(baseFrameName);
    int rootLinkIndex = dynComp.model().getFrameIndex(rootLinkName);

    if (baseLinkIndex < 0 || baseFrameIndex < 0 || rootLinkIndex < 0)
    {
        // if frames are not present, test should fail
        ASSERT_IS_TRUE(false);
    }

    // Set floating base to l_foot and create a state with non-zero velocities
    ok = dynComp.setFloatingBase(baseLinkName);
    ASSERT_IS_TRUE(ok);

    // Create a non-trivial state with non-zero velocities for both base and joints
    Transform world_H_l_foot = getRandomTransform();
    VectorDynSize q_joint(dynComp.model().getNrOfPosCoords());
    getRandomJointPositions(q_joint, dynComp.model());

    // Generate non-zero base velocity
    Twist base_vel_l_foot = getRandomTwist();

    // Generate non-zero base acceleration
    iDynTree::Vector6 base_acc_l_foot = getRandomTwist().asVector();

    VectorDynSize dq_joint(dynComp.model().getNrOfDOFs());
    getRandomVector(dq_joint);
    VectorDynSize ddq_joint(dynComp.model().getNrOfDOFs());
    getRandomVector(ddq_joint);

    LinkNetExternalWrenches externalWrenches(dynComp.model());
    for (LinkIndex lnkIdx = 0; lnkIdx < dynComp.model().getNrOfLinks(); lnkIdx++)
    {
        // Generate random external wrenches
        externalWrenches(lnkIdx) = getRandomWrench();
    }

    Vector3 gravity;
    gravity(0) = 0.0;
    gravity(1) = 0.0;
    gravity(2) = -9.81;

    // Set state with l_foot as floating base
    ok = dynComp.setRobotState(world_H_l_foot, q_joint, base_vel_l_foot, dq_joint, gravity);
    ASSERT_IS_TRUE(ok);

    // Compute quantities with l_foot as base
    Transform world_H_l_sole_with_l_foot_base = dynComp.getWorldTransform(baseFrameName);
    Transform world_H_l_foot_with_l_foot_base = dynComp.getWorldTransform(baseLinkName);
    Position com_pos_with_l_foot_base = dynComp.getCenterOfMassPosition();
    Twist l_sole_velocity_with_l_foot_base = dynComp.getFrameVel(baseFrameName);
    Twist l_foot_velocity_with_l_foot_base = dynComp.getFrameVel(baseLinkName);
    Transform l_foot_H_l_sole = world_H_l_foot.inverse() * world_H_l_sole_with_l_foot_base;
    iDynTree::Vector6 l_sole_acceleration_with_l_foot_base
        = dynComp.getFrameAcc(baseFrameName, base_acc_l_foot, ddq_joint);
    iDynTree::Vector6 rootLinkAcceleration_with_l_foot_base
        = dynComp.getFrameAcc(rootLinkName, base_acc_l_foot, ddq_joint);
    iDynTree::Vector6 rootLink_bias_acc_with_l_foot_base = dynComp.getFrameBiasAcc(rootLinkName);
    MatrixDynSize MassMatrix_with_l_foot_base(6 + dynComp.getNrOfDegreesOfFreedom(),
                                              6 + dynComp.getNrOfDegreesOfFreedom());
    dynComp.getFreeFloatingMassMatrix(MassMatrix_with_l_foot_base);
    MatrixDynSize Jacobian_rootLink_with_l_foot_base(6, 6 + dynComp.getNrOfDegreesOfFreedom());
    dynComp.getFrameFreeFloatingJacobian(rootLinkName, Jacobian_rootLink_with_l_foot_base);
    FreeFloatingGeneralizedTorques baseForceAndJointTorques_with_l_foot_base(dynComp.model());
    ASSERT_IS_TRUE(dynComp.inverseDynamics(base_acc_l_foot,
                                           ddq_joint,
                                           externalWrenches,
                                           baseForceAndJointTorques_with_l_foot_base));
    FreeFloatingGeneralizedTorques biasForces_with_l_foot_base(dynComp.model());
    ASSERT_IS_TRUE(dynComp.generalizedBiasForces(biasForces_with_l_foot_base));
    FreeFloatingGeneralizedTorques gravityForces_with_l_foot_base(dynComp.model());
    ASSERT_IS_TRUE(dynComp.generalizedGravityForces(gravityForces_with_l_foot_base));

    // Now set floating base to l_sole
    ok = dynComp.setFloatingBase(baseFrameName);
    ASSERT_IS_TRUE(ok);

    // Set state with l_sole as base - using the computed transform and velocity from previous state
    ok = dynComp.setRobotState(world_H_l_sole_with_l_foot_base,
                               q_joint,
                               l_sole_velocity_with_l_foot_base,
                               dq_joint,
                               gravity);
    ASSERT_IS_TRUE(ok);

    // Compute quantities with l_sole as base
    Transform world_H_l_sole_with_l_sole_base = dynComp.getWorldTransform(baseFrameName);
    Transform world_H_l_foot_with_l_sole_base = dynComp.getWorldTransform(baseLinkName);
    Position com_pos_with_l_sole_base = dynComp.getCenterOfMassPosition();
    Twist l_sole_velocity_with_l_sole_base = dynComp.getFrameVel(baseFrameName);
    Twist l_foot_velocity_with_l_sole_base = dynComp.getFrameVel(baseLinkName);
    iDynTree::Vector6 rootLinkAcceleration_with_l_sole_base
        = dynComp.getFrameAcc(rootLinkName, l_sole_acceleration_with_l_foot_base, ddq_joint);
    iDynTree::Vector6 rootLink_bias_acc_with_l_sole_base = dynComp.getFrameBiasAcc(rootLinkIndex);
    MatrixDynSize MassMatrix_with_l_sole_base(6 + dynComp.getNrOfDegreesOfFreedom(),
                                              6 + dynComp.getNrOfDegreesOfFreedom());
    dynComp.getFreeFloatingMassMatrix(MassMatrix_with_l_sole_base);
    MatrixDynSize Jacobian_rootLink_with_l_sole_base(6, 6 + dynComp.getNrOfDegreesOfFreedom());
    dynComp.getFrameFreeFloatingJacobian(rootLinkName, Jacobian_rootLink_with_l_sole_base);
    FreeFloatingGeneralizedTorques baseForceAndJointTorques_with_l_sole_base(dynComp.model());
    ASSERT_IS_TRUE(dynComp.inverseDynamics(l_sole_acceleration_with_l_foot_base,
                                           ddq_joint,
                                           externalWrenches,
                                           baseForceAndJointTorques_with_l_sole_base));
    FreeFloatingGeneralizedTorques biasForces_with_l_sole_base(dynComp.model());
    ASSERT_IS_TRUE(dynComp.generalizedBiasForces(biasForces_with_l_sole_base));
    FreeFloatingGeneralizedTorques gravityForces_with_l_sole_base(dynComp.model());
    ASSERT_IS_TRUE(dynComp.generalizedGravityForces(gravityForces_with_l_sole_base));

    // Test that computed quantities are consistent regardless of floating base choice
    ASSERT_EQUAL_TRANSFORM(world_H_l_sole_with_l_foot_base, world_H_l_sole_with_l_sole_base);
    ASSERT_EQUAL_TRANSFORM(world_H_l_foot_with_l_foot_base, world_H_l_foot_with_l_sole_base);
    ASSERT_EQUAL_VECTOR(com_pos_with_l_foot_base, com_pos_with_l_sole_base);
    ASSERT_EQUAL_VECTOR(l_sole_velocity_with_l_foot_base.asVector(),
                        l_sole_velocity_with_l_sole_base.asVector());
    ASSERT_EQUAL_VECTOR(l_foot_velocity_with_l_foot_base.asVector(),
                        l_foot_velocity_with_l_sole_base.asVector());
    ASSERT_EQUAL_VECTOR(dynComp.getBaseTwist().asVector(),
                        l_sole_velocity_with_l_sole_base.asVector());
    ASSERT_EQUAL_VECTOR(rootLink_bias_acc_with_l_foot_base, rootLink_bias_acc_with_l_sole_base);
    ASSERT_EQUAL_VECTOR(rootLinkAcceleration_with_l_foot_base,
                        rootLinkAcceleration_with_l_sole_base);
    ASSERT_EQUAL_VECTOR(baseForceAndJointTorques_with_l_foot_base.jointTorques(),
                        baseForceAndJointTorques_with_l_sole_base.jointTorques());
    ASSERT_EQUAL_VECTOR(biasForces_with_l_foot_base.jointTorques(),
                        biasForces_with_l_sole_base.jointTorques());
    ASSERT_EQUAL_VECTOR(gravityForces_with_l_foot_base.jointTorques(),
                        gravityForces_with_l_sole_base.jointTorques());
    iDynTree::Transform toFrame_H_fromFrame;
    if (dynComp.getFrameVelocityRepresentation() == iDynTree::BODY_FIXED_REPRESENTATION)
    {
        toFrame_H_fromFrame = l_foot_H_l_sole;
    } else if (dynComp.getFrameVelocityRepresentation() == iDynTree::INERTIAL_FIXED_REPRESENTATION)
    {
        toFrame_H_fromFrame = Transform::Identity();
    } else if (dynComp.getFrameVelocityRepresentation() == iDynTree::MIXED_REPRESENTATION)
    {
        // l_foot[W]_H_l_sole[W]
        toFrame_H_fromFrame = Transform(Rotation::Identity(),
                                        world_H_l_sole_with_l_sole_base.getPosition()
                                            - world_H_l_foot_with_l_foot_base.getPosition());
    }
    ASSERT_EQUAL_VECTOR(baseForceAndJointTorques_with_l_foot_base.baseWrench(),
                        toFrame_H_fromFrame
                            * baseForceAndJointTorques_with_l_sole_base.baseWrench());
    ASSERT_EQUAL_VECTOR(biasForces_with_l_foot_base.baseWrench(),
                        toFrame_H_fromFrame * biasForces_with_l_sole_base.baseWrench());
    ASSERT_EQUAL_VECTOR(gravityForces_with_l_foot_base.baseWrench(),
                        toFrame_H_fromFrame * gravityForces_with_l_sole_base.baseWrench());

    // recompute some quantities with getRobotState
    dynComp.getRobotState(world_H_l_sole_with_l_sole_base,
                          q_joint,
                          l_sole_velocity_with_l_sole_base,
                          dq_joint,
                          gravity);

    // check getRobotState consistency
    ASSERT_EQUAL_TRANSFORM(world_H_l_sole_with_l_foot_base, world_H_l_sole_with_l_sole_base);
    ASSERT_EQUAL_VECTOR(l_sole_velocity_with_l_foot_base.asVector(),
                        l_sole_velocity_with_l_sole_base.asVector());

    // Mass matrix and consistency check via kinetic energy
    VectorDynSize nu_with_l_foot_base(6 + dynComp.getNrOfDegreesOfFreedom());
    toEigen(nu_with_l_foot_base) = toEigen(l_foot_velocity_with_l_foot_base.asVector(), dq_joint);
    VectorDynSize nu_with_l_sole_base(6 + dynComp.getNrOfDegreesOfFreedom());
    toEigen(nu_with_l_sole_base) = toEigen(l_sole_velocity_with_l_sole_base.asVector(), dq_joint);

    double kinetic_energy_with_l_foot_base = toEigen(nu_with_l_foot_base).transpose()
                                             * toEigen(MassMatrix_with_l_foot_base)
                                             * toEigen(nu_with_l_foot_base);
    double kinetic_energy_with_l_sole_base = toEigen(nu_with_l_sole_base).transpose()
                                             * toEigen(MassMatrix_with_l_sole_base)
                                             * toEigen(nu_with_l_sole_base);
    // kinetic energy should be the same regardless of floating base choice
    ASSERT_EQUAL_DOUBLE(kinetic_energy_with_l_foot_base, kinetic_energy_with_l_sole_base);

    // Check Jacobian consistency
    VectorDynSize rootLink_vel_with_l_foot_base(6);
    toEigen(rootLink_vel_with_l_foot_base)
        = toEigen(Jacobian_rootLink_with_l_foot_base) * toEigen(nu_with_l_foot_base);

    VectorDynSize rootLink_vel_with_l_sole_base(6);
    toEigen(rootLink_vel_with_l_sole_base)
        = toEigen(Jacobian_rootLink_with_l_sole_base) * toEigen(nu_with_l_sole_base);

    ASSERT_EQUAL_VECTOR(rootLink_vel_with_l_foot_base, rootLink_vel_with_l_sole_base);
}

void testFloatingBaseFrameConsistencyAllRepresentations(std::string modelName)
{
    std::string urdfFileName = getAbsModelPath(modelName);
    std::cout << "Testing floating base frame consistency for file " << urdfFileName << std::endl;
    testFloatingBaseFrameConsistency(urdfFileName, iDynTree::INERTIAL_FIXED_REPRESENTATION);
    testFloatingBaseFrameConsistency(urdfFileName, iDynTree::BODY_FIXED_REPRESENTATION);
    testFloatingBaseFrameConsistency(urdfFileName, iDynTree::MIXED_REPRESENTATION);
}

void testCoriolisAndMassMatricesConsistency(KinDynComputations& dynComp)
{
    const size_t nrOfDofs = dynComp.getNrOfDegreesOfFreedom();
    const size_t matSize = 6 + nrOfDofs;

    // Get all three matrices from function being tested (i.e., getCoriolisAndMassMatrices)
    MatrixDynSize coriolisMatrix(matSize, matSize);
    MatrixDynSize massMatrix(matSize, matSize);
    MatrixDynSize massMatrixDerivative(matSize, matSize);
    bool ok = dynComp.getCoriolisAndMassMatrices(coriolisMatrix, massMatrix, massMatrixDerivative);
    ASSERT_IS_TRUE(ok);

    // Get the mass matrix from getFreeFloatingMassMatrix for comparison
    MatrixDynSize massMatrixRef(matSize, matSize);
    ok = dynComp.getFreeFloatingMassMatrix(massMatrixRef);
    ASSERT_IS_TRUE(ok);

    // Test 1: Mass matrix from getCoriolisAndMassMatrices should match getFreeFloatingMassMatrix
    ASSERT_EQUAL_MATRIX_TOL(massMatrix, massMatrixRef, 1e-8);

    // Test 2: Mdot = C + C^T
    MatrixDynSize C_plus_CT(matSize, matSize);
    toEigen(C_plus_CT) = toEigen(coriolisMatrix) + toEigen(coriolisMatrix).transpose();
    ASSERT_EQUAL_MATRIX_TOL(massMatrixDerivative, C_plus_CT, 1e-8);

    // Test 3: C*nu = bias forces - gravity forces
    FreeFloatingGeneralizedTorques biasForces(dynComp.model());
    ok = dynComp.generalizedBiasForces(biasForces);
    ASSERT_IS_TRUE(ok);

    FreeFloatingGeneralizedTorques gravityForces(dynComp.model());
    ok = ok && dynComp.generalizedGravityForces(gravityForces);
    ASSERT_IS_TRUE(ok);

    VectorDynSize nu(6 + nrOfDofs);
    ok = dynComp.getModelVel(nu);
    ASSERT_IS_TRUE(ok);

    VectorDynSize C_nu(6 + nrOfDofs);
    toEigen(C_nu) = toEigen(coriolisMatrix) * toEigen(nu);

    ASSERT_EQUAL_VECTOR_TOL(toEigen(C_nu).tail(nrOfDofs),
                            toEigen(biasForces.jointTorques())
                                - toEigen(gravityForces.jointTorques()),
                            1e-8);

    ASSERT_EQUAL_VECTOR_TOL(toEigen(C_nu).head(6),
                            toEigen(biasForces.baseWrench()) - toEigen(gravityForces.baseWrench()),
                            1e-8);
}

void testCoriolisAndMassMatricesOnModel(std::string modelFilePath)
{
    iDynTree::ModelLoader mdlLoader;
    bool ok = mdlLoader.loadModelFromFile(modelFilePath);
    ASSERT_IS_TRUE(ok);

    KinDynComputations dynComp;
    ok = dynComp.loadRobotModel(mdlLoader.model());
    ASSERT_IS_TRUE(ok);

    std::cout << "Testing getCoriolisAndMassMatrices for model: " << modelFilePath << std::endl;

    // Test getCoriolisAndMassMatrices on a KinDynComputations object for all frame velocity
    // representations.
    for (auto repr :
         {BODY_FIXED_REPRESENTATION, MIXED_REPRESENTATION, INERTIAL_FIXED_REPRESENTATION})
    {
        ok = dynComp.setFrameVelocityRepresentation(repr);
        ASSERT_IS_TRUE(ok);

        // Test multiple random configurations
        for (int iter = 0; iter < 5; iter++)
        {
            setRandomState(dynComp);
            testCoriolisAndMassMatricesConsistency(dynComp);
        }
    }
}

void testCoriolisAndMassMatricesOnRandomModels()
{
    std::cout << "Testing getCoriolisAndMassMatrices consistency on random models..." << std::endl;

    // Test with different model sizes
    for (unsigned int nrOfJoints = 1; nrOfJoints <= 10; nrOfJoints += 3)
    {
        std::cout << "  Testing random model with " << nrOfJoints << " joints..." << std::endl;

        // Create a random model with revolute joints
        Model randomModel = getRandomModel(nrOfJoints, 5, ALL_JOINT_TYPES);

        KinDynComputations dynComp;
        bool ok = dynComp.loadRobotModel(randomModel);
        ASSERT_IS_TRUE(ok);

        // Test for each frame velocity representation
        for (auto repr :
             {BODY_FIXED_REPRESENTATION, MIXED_REPRESENTATION, INERTIAL_FIXED_REPRESENTATION})
        {
            ok = dynComp.setFrameVelocityRepresentation(repr);
            ASSERT_IS_TRUE(ok);

            // Test multiple random configurations
            for (int iter = 0; iter < 3; iter++)
            {
                setRandomState(dynComp);
                testCoriolisAndMassMatricesConsistency(dynComp);
            }
        }
    }
}

void testCoriolisMatrixAlgorithmOnFloatingBaseModel(std::string modelFilePath,
                                                    const FrameVelocityRepresentation frameVelRepr)
{
    // It tests getCoriolisAndMassMatrices on a floating base model with
    // different frames used as floating base.

    iDynTree::KinDynComputations dynComp;
    iDynTree::ModelLoader mdlLoader;
    bool ok = mdlLoader.loadModelFromFile(modelFilePath);
    ok = ok && dynComp.loadRobotModel(mdlLoader.model());
    ASSERT_IS_TRUE(ok);

    ok = dynComp.setFrameVelocityRepresentation(frameVelRepr);
    ASSERT_IS_TRUE(ok);

    const int nrOfDofs = dynComp.getNrOfDegreesOfFreedom();

    // Test with iCub model using l_foot as base link vs l_sole as base frame
    std::string baseLinkName = "l_foot";
    std::string baseFrameName = "l_sole";

    // Check if both frames exist in the model
    int baseLinkIndex = dynComp.model().getFrameIndex(baseLinkName);
    int baseFrameIndex = dynComp.model().getFrameIndex(baseFrameName);

    if (baseLinkIndex < 0 || baseFrameIndex < 0)
    {
        // if frames are not present, test should fail
        ASSERT_IS_TRUE(false);
    }

    // Set floating base to l_foot and create a state with non-zero velocities
    ok = dynComp.setFloatingBase(baseLinkName);
    ASSERT_IS_TRUE(ok);

    // Create a non-trivial state with non-zero velocities for both base and joints
    Transform world_H_l_foot = getRandomTransform();
    VectorDynSize q_joint(dynComp.model().getNrOfPosCoords());
    getRandomJointPositions(q_joint, dynComp.model());

    // Generate non-zero base velocity
    Twist base_vel_l_foot = getRandomTwist();

    // Set random joint velocities
    VectorDynSize qdot_joint(dynComp.model().getNrOfDOFs());
    getRandomVector(qdot_joint);

    // Gravity
    Vector3 gravity;
    gravity(0) = 0.0;
    gravity(1) = 0.0;
    gravity(2) = -9.81;

    // Set state with l_foot as floating base
    ok = dynComp.setRobotState(world_H_l_foot, q_joint, base_vel_l_foot, qdot_joint, gravity);
    ASSERT_IS_TRUE(ok);

    VectorDynSize nu_l_foot(6 + nrOfDofs);
    ok = dynComp.getModelVel(nu_l_foot);
    ASSERT_IS_TRUE(ok);

    // Get coriolis matrix, mass matrix, and mass matrix derivative with l_foot as base
    MatrixDynSize coriolisMatrix_l_foot(6 + nrOfDofs, 6 + nrOfDofs);
    MatrixDynSize massMatrix_l_foot(6 + nrOfDofs, 6 + nrOfDofs);
    MatrixDynSize massMatrixDerivative_l_foot(6 + nrOfDofs, 6 + nrOfDofs);
    ok = dynComp.getCoriolisAndMassMatrices(coriolisMatrix_l_foot,
                                            massMatrix_l_foot,
                                            massMatrixDerivative_l_foot);
    ASSERT_IS_TRUE(ok);

    // Check mass Mdot = C + C^T
    ASSERT_EQUAL_MATRIX_TOL(toEigen(massMatrixDerivative_l_foot),
                            toEigen(coriolisMatrix_l_foot)
                                + toEigen(coriolisMatrix_l_foot).transpose(),
                            1e-8);

    // Check mass matrices consistency
    MatrixDynSize massMatrix_l_foot_reference(6 + nrOfDofs, 6 + nrOfDofs);
    ok = dynComp.getFreeFloatingMassMatrix(massMatrix_l_foot_reference);
    ASSERT_IS_TRUE(ok);
    ASSERT_EQUAL_MATRIX_TOL(toEigen(massMatrix_l_foot), toEigen(massMatrix_l_foot_reference), 1e-8);

    // Check C_nu = bias_forces - gravity_forces
    FreeFloatingGeneralizedTorques biasForces_l_foot(dynComp.model());
    ok = dynComp.generalizedBiasForces(biasForces_l_foot);
    ASSERT_IS_TRUE(ok);
    FreeFloatingGeneralizedTorques gravityForces_l_foot(dynComp.model());
    ok = ok && dynComp.generalizedGravityForces(gravityForces_l_foot);
    ASSERT_IS_TRUE(ok);

    VectorDynSize C_nu_l_foot(6 + nrOfDofs);
    toEigen(C_nu_l_foot) = toEigen(coriolisMatrix_l_foot) * toEigen(nu_l_foot);

    ASSERT_EQUAL_VECTOR_TOL(toEigen(C_nu_l_foot).tail(nrOfDofs),
                            toEigen(biasForces_l_foot.jointTorques())
                                - toEigen(gravityForces_l_foot.jointTorques()),
                            1e-8);

    ASSERT_EQUAL_VECTOR_TOL(toEigen(C_nu_l_foot).head(6),
                            toEigen(biasForces_l_foot.baseWrench())
                                - toEigen(gravityForces_l_foot.baseWrench()),
                            1e-8);

    // Now set floating base to l_sole
    ok = dynComp.setFloatingBase(baseFrameName);
    ASSERT_IS_TRUE(ok);

    // Update state with l_sole as floating base
    Transform world_H_l_sole = dynComp.getWorldTransform(baseFrameName);
    Twist base_vel_l_sole = dynComp.getFrameVel(baseFrameName);
    ok = dynComp.setRobotState(world_H_l_sole, q_joint, base_vel_l_sole, qdot_joint, gravity);
    ASSERT_IS_TRUE(ok);

    VectorDynSize nu_l_sole(6 + nrOfDofs);
    ok = dynComp.getModelVel(nu_l_sole);
    ASSERT_IS_TRUE(ok);

    // Get coriolis matrix, mass matrix, and mass matrix derivative with l_sole as base
    MatrixDynSize coriolisMatrix_l_sole(6 + nrOfDofs, 6 + nrOfDofs);
    MatrixDynSize massMatrix_l_sole(6 + nrOfDofs, 6 + nrOfDofs);
    MatrixDynSize massMatrixDerivative_l_sole(6 + nrOfDofs, 6 + nrOfDofs);
    ok = dynComp.getCoriolisAndMassMatrices(coriolisMatrix_l_sole,
                                            massMatrix_l_sole,
                                            massMatrixDerivative_l_sole);
    ASSERT_IS_TRUE(ok);

    // Check mass Mdot = C + C^T
    ASSERT_EQUAL_MATRIX_TOL(toEigen(massMatrixDerivative_l_sole),
                            toEigen(coriolisMatrix_l_sole)
                                + toEigen(coriolisMatrix_l_sole).transpose(),
                            1e-8);

    // Check mass matrices consistency
    MatrixDynSize massMatrix_l_sole_reference(6 + nrOfDofs, 6 + nrOfDofs);
    ok = dynComp.getFreeFloatingMassMatrix(massMatrix_l_sole_reference);
    ASSERT_IS_TRUE(ok);
    ASSERT_EQUAL_MATRIX_TOL(toEigen(massMatrix_l_sole), toEigen(massMatrix_l_sole_reference), 1e-8);

    // Check C_nu = bias_forces - gravity_forces
    FreeFloatingGeneralizedTorques biasForces_l_sole(dynComp.model());
    ok = dynComp.generalizedBiasForces(biasForces_l_sole);
    ASSERT_IS_TRUE(ok);
    FreeFloatingGeneralizedTorques gravityForces_l_sole(dynComp.model());
    ok = ok && dynComp.generalizedGravityForces(gravityForces_l_sole);
    ASSERT_IS_TRUE(ok);

    VectorDynSize C_nu_l_sole(6 + nrOfDofs);
    toEigen(C_nu_l_sole) = toEigen(coriolisMatrix_l_sole) * toEigen(nu_l_sole);

    ASSERT_EQUAL_VECTOR_TOL(toEigen(C_nu_l_sole).tail(nrOfDofs),
                            toEigen(biasForces_l_sole.jointTorques())
                                - toEigen(gravityForces_l_sole.jointTorques()),
                            1e-8);

    ASSERT_EQUAL_VECTOR_TOL(toEigen(C_nu_l_sole).head(6),
                            toEigen(biasForces_l_sole.baseWrench())
                                - toEigen(gravityForces_l_sole.baseWrench()),
                            1e-8);

    // check kinetic energy consistency
    double kineticEnergy_l_foot
        = 0.5 * toEigen(nu_l_foot).transpose() * toEigen(massMatrix_l_foot) * toEigen(nu_l_foot);
    double kineticEnergy_l_sole
        = 0.5 * toEigen(nu_l_sole).transpose() * toEigen(massMatrix_l_sole) * toEigen(nu_l_sole);

    ASSERT_EQUAL_DOUBLE_TOL(kineticEnergy_l_foot, kineticEnergy_l_sole, 1e-9);
}

void testCoriolisMatrixAlgorithmOnFloatingBaseModelAllRepresentations(std::string modelFilePath)
{

    for (auto repr :
         {BODY_FIXED_REPRESENTATION, MIXED_REPRESENTATION, INERTIAL_FIXED_REPRESENTATION})
    {
        std::cout << "Testing getCoriolisAndMassMatrices using different floating bases for "
                     "representation "
                  << repr << " on model " << modelFilePath << std::endl;
        testCoriolisMatrixAlgorithmOnFloatingBaseModel(modelFilePath, repr);
    }
}

int main()
{
    // Just run the tests on a handful of models to avoid
    // a long testing time under valgrind
    testModelConsistencyAllRepresentations("oneLink.urdf");
    testModelConsistencyAllRepresentations("twoLinks.urdf");
    testModelConsistencyAllRepresentations("threeLinks.urdf");
    testModelConsistencyAllRepresentations("bigman.urdf");
    testModelConsistencyAllRepresentations("icub_skin_frames.urdf");
    testModelConsistencyAllRepresentations("iCubGenova02.urdf");
    testModelConsistencyAllRepresentations("icalibrate.urdf");

    testSubModelConsistencyAllRepresentations("oneLink.urdf");
    testSubModelConsistencyAllRepresentations("twoLinks.urdf");
    testSubModelConsistencyAllRepresentations("threeLinks.urdf");
    testSubModelConsistencyAllRepresentations("bigman.urdf");
    testSubModelConsistencyAllRepresentations("icub_skin_frames.urdf");
    testSubModelConsistencyAllRepresentations("iCubGenova02.urdf");
    testSubModelConsistencyAllRepresentations("icalibrate.urdf");

    testSparsityPatternAllRepresentations("oneLink.urdf");
    testSparsityPatternAllRepresentations("twoLinks.urdf");
    testSparsityPatternAllRepresentations("threeLinks.urdf");
    testSparsityPatternAllRepresentations("bigman.urdf");
    testSparsityPatternAllRepresentations("icub_skin_frames.urdf");

    // Test kinetic energy consistency on random models with various joint types
    // (including revolute, spherical, and others)
    testKineticEnergyOnRandomModels();

    // Test kinetic energy consistency on URDF models
    std::cout << "Testing kinetic energy consistency on URDF models..." << std::endl;
    testKineticEnergyOnURDFModel(getAbsModelPath("oneLink.urdf"));
    testKineticEnergyOnURDFModel(getAbsModelPath("twoLinks.urdf"));
    testKineticEnergyOnURDFModel(getAbsModelPath("threeLinks.urdf"));
    testKineticEnergyOnURDFModel(getAbsModelPath("bigman.urdf"));
    testKineticEnergyOnURDFModel(getAbsModelPath("icub_skin_frames.urdf"));
    testKineticEnergyOnURDFModel(getAbsModelPath("iCubGenova02.urdf"));

    testFloatingBaseFrameConsistencyAllRepresentations("iCubGenova02.urdf");

    // Test getCoriolisAndMassMatrices consistency (mass matrix matches getFreeFloatingMassMatrix,
    // Hdot = C + C^T)
    testCoriolisAndMassMatricesOnModel(getAbsModelPath("oneLink.urdf"));
    testCoriolisAndMassMatricesOnModel(getAbsModelPath("twoLinks.urdf"));
    testCoriolisAndMassMatricesOnModel(getAbsModelPath("threeLinks.urdf"));
    testCoriolisAndMassMatricesOnModel(getAbsModelPath("bigman.urdf"));
    testCoriolisAndMassMatricesOnModel(getAbsModelPath("iCubGenova02.urdf"));

    // Test getCoriolisAndMassMatrices on random models
    testCoriolisAndMassMatricesOnRandomModels();

    // Test getCoriolisAndMassMatrices on a Floating Base Model using different floating bases
    testCoriolisMatrixAlgorithmOnFloatingBaseModelAllRepresentations(getAbsModelPath("iCubGenova02."
                                                                                     "urdf"));

    return EXIT_SUCCESS;
}
