// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/Model.h>
#include <iDynTree/Traversal.h>

#include <iDynTree/FreeFloatingMatrices.h>
#include <iDynTree/FreeFloatingState.h>
#include <iDynTree/JointState.h>
#include <iDynTree/LinkState.h>

#include <iDynTree/ArticulatedBodyInertia.h>
#include <iDynTree/EigenHelpers.h>
#include <iDynTree/MatrixDynSize.h>
#include <iDynTree/SpatialInertia.h>
#include <iDynTree/SpatialMomentum.h>
#include <iDynTree/VectorDynSize.h>

#include <iDynTree/Dynamics.h>

#include <Eigen/Core>

namespace iDynTree
{

bool ComputeLinearAndAngularMomentum(const Model& model,
                                     const LinkPositions& linkPositions,
                                     const LinkVelArray& linkVels,
                                     SpatialMomentum& totalMomentum)
{
    totalMomentum.zero();

    for (LinkIndex lnkIdx = 0; lnkIdx < static_cast<LinkIndex>(model.getNrOfLinks()); lnkIdx++)
    {
        const Transform& commonFrame_X_link = linkPositions(lnkIdx);
        const Twist& v = linkVels(lnkIdx);
        const SpatialInertia& I = model.getLink(lnkIdx)->getInertia();
        totalMomentum = totalMomentum + commonFrame_X_link * (I * v);
    }

    return true;
}

bool ComputeLinearAndAngularMomentumDerivativeBias(const Model& model,
                                                   const LinkPositions& linkPositions,
                                                   const LinkVelArray& linkVel,
                                                   const LinkAccArray& linkBiasAcc,
                                                   Wrench& totalMomentumBias)
{
    totalMomentumBias.zero();

    for (LinkIndex lnkIdx = 0; lnkIdx < static_cast<LinkIndex>(model.getNrOfLinks()); lnkIdx++)
    {
        const Transform& commonFrame_X_link = linkPositions(lnkIdx);
        const Twist& v = linkVel(lnkIdx);
        const SpatialAcc& a_bias = linkBiasAcc(lnkIdx);
        const SpatialInertia& I = model.getLink(lnkIdx)->getInertia();
        totalMomentumBias = totalMomentumBias + commonFrame_X_link * (I * a_bias + v * (I * v));
    }

    return true;
}

bool RNEADynamicPhase(const Model& model,
                      const Traversal& traversal,
                      const JointPosDoubleArray& jointPos,
                      const LinkVelArray& linksVels,
                      const LinkAccArray& linksAccs,
                      const LinkNetExternalWrenches& fext,
                      LinkInternalWrenches& f,
                      FreeFloatingGeneralizedTorques& baseWrenchJntTorques)
{
    bool retValue = true;

    for (int traversalEl = traversal.getNrOfVisitedLinks() - 1; traversalEl >= 0; traversalEl--)
    {
        LinkConstPtr visitedLink = traversal.getLink(traversalEl);
        LinkIndex visitedLinkIndex = visitedLink->getIndex();
        LinkConstPtr parentLink = traversal.getParentLink(traversalEl);
        IJointConstPtr toParentJoint = traversal.getParentJoint(traversalEl);

        // We sum in the link internal wrench the inertial wrenches,
        // the external wrenches and the child link wrenches .
        // It is Equation 5.20 in Featherstone 2008 , with the
        // only difference that we assume the external forces
        // are expressed in the link reference frame (both orientation
        // and point).
        const iDynTree::SpatialInertia& I = visitedLink->getInertia();
        const iDynTree::SpatialAcc& a = linksAccs(visitedLinkIndex);
        const iDynTree::Twist& v = linksVels(visitedLinkIndex);
        f(visitedLinkIndex) = I * a + v * (I * v) - fext(visitedLinkIndex);

        // Iterate on childs of visitedLink
        // We obtain all the children as all the neighbors of the link, except
        // for its parent
        // \todo TODO this point is definitly Tree-specific
        // \todo TODO this "get child" for is duplicated in the code, we
        //            should try to consolidate it
        for (unsigned int neigh_i = 0; neigh_i < model.getNrOfNeighbors(visitedLinkIndex);
             neigh_i++)
        {
            LinkIndex neighborIndex = model.getNeighbor(visitedLinkIndex, neigh_i).neighborLink;
            if (!parentLink || neighborIndex != parentLink->getIndex())
            {
                LinkIndex childIndex = neighborIndex;
                IJointConstPtr neighborJoint
                    = model.getJoint(model.getNeighbor(visitedLinkIndex, neigh_i).neighborJoint);

                const Transform& visitedLink_X_child
                    = neighborJoint->getTransform(jointPos, visitedLinkIndex, childIndex);

                // One term of the sum in Equation 5.20 in Featherstone 2008
                f(visitedLinkIndex) = f(visitedLinkIndex) + visitedLink_X_child * f(childIndex);
            }
        }

        if (parentLink == 0)
        {
            // If the visited link is the base, the base has no parent, and hence no
            // joint torque to compute.
            // In this case the base wrench is simply saved in the output generalized
            // torques vector (without a minus because they both express the wrench applied on the
            // base). Notice that this force, if the model, the accelerations and the external
            // wrenches are coherent, should be zero. This because any external wrench on the base
            // should be present also in the fExt vector .
            baseWrenchJntTorques.baseWrench() = f(visitedLinkIndex);

            // As the the base link has no parent link and the residual force/torque is reported as
            // the base wrench in the generalized torques, set the related internal joint
            // force/torque to zero
            f(visitedLinkIndex) = iDynTree::Wrench::Zero();
        } else
        {
            // If the visited link is not the base and it is connected to a parent link
            // at this point we can compute the torque of the joint connecting the visited link and
            // its parent This is Equation 5.13 in Featherstone 2008. It is offloaded to the joint
            // to be able to deal with different kind of joints.
            toParentJoint->computeJointTorque(jointPos,
                                              f(visitedLinkIndex),
                                              parentLink->getIndex(),
                                              visitedLinkIndex,
                                              baseWrenchJntTorques.jointTorques());
        }
    }

    return retValue;
}

bool CompositeRigidBodyAlgorithm(const Model& model,
                                 const Traversal& traversal,
                                 const JointPosDoubleArray& jointPos,
                                 LinkCompositeRigidBodyInertias& linkCRBs,
                                 FreeFloatingMassMatrix& massMatrix)
{
    // Map the massMatrix to an Eigen matrix
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        massMatrixEigen(massMatrix.data(), massMatrix.rows(), massMatrix.cols());

    /**
     * Forward pass: initialize the CRBI
     * of each link to its own inertia.
     */
    for (unsigned int traversalEl = 0; traversalEl < traversal.getNrOfVisitedLinks(); traversalEl++)
    {
        LinkConstPtr visitedLink = traversal.getLink(traversalEl);
        LinkIndex visitedLinkIndex = visitedLink->getIndex();
        linkCRBs(visitedLinkIndex) = visitedLink->getInertia();
    }

    /**
     * Backward pass: for each link compute the
     * CRB of the link (given the traversal).
     */
    for (int traversalEl = traversal.getNrOfVisitedLinks() - 1; traversalEl >= 0; traversalEl--)
    {
        LinkConstPtr visitedLink = traversal.getLink(traversalEl);
        LinkIndex visitedLinkIndex = visitedLink->getIndex();
        LinkConstPtr parentLink = traversal.getParentLink(traversalEl);
        IJointConstPtr toParentJoint = traversal.getParentJoint(traversalEl);

        // If the visited link is not the base one, add its
        // CRBI to the CRBI of the parent
        // \todo TODO streamline the check "is Link the floating base"
        // given a traversal
        if (parentLink)
        {
            LinkIndex parentLinkIndex = parentLink->getIndex();

            linkCRBs(parentLinkIndex)
                = linkCRBs(parentLinkIndex)
                  + (toParentJoint->getTransform(jointPos, parentLinkIndex, visitedLinkIndex))
                        * linkCRBs(visitedLinkIndex);

            // If the visited link is attached to its parent with a fixed joint,
            // we don't need to do anything else for this link.
            // Otherwise we need to compute the rows and columns of the mass matrix
            // related to the dof(s) connecting the visited link to its parent
            if (toParentJoint->getNrOfDOFs() >= 1)
            {
                // Handle multiple DOFs (e.g., spherical joints with 3 DOFs)
                unsigned int nrOfDOFs = toParentJoint->getNrOfDOFs();

                for (unsigned int dofIdx = 0; dofIdx < nrOfDOFs; dofIdx++)
                {
                    // In this loop we follow the algorithm as described in
                    // in Featherstone 2008 , Table 6.2 . In particular S_visitedDof (S_i in the
                    // book) is the motion subspace vector connected to the degree of freedom
                    // connecting the link to its parent, while S_ancestorDof (S_j in the book) is
                    // the motion subspace vector of its ancestor considered in the inner loop
                    SpatialMotionVector S_visitedDof
                        = toParentJoint->getMotionSubspaceVector(dofIdx,
                                                                 visitedLink->getIndex(),
                                                                 parentLinkIndex);
                    SpatialForceVector F = linkCRBs(visitedLinkIndex) * S_visitedDof;

                    // We compute the term of the mass matrix on the diagonal
                    // (in the book: H_ii = S_i^\top F
                    size_t dofIndex = toParentJoint->getDOFsOffset() + dofIdx;
                    massMatrix(6 + dofIndex, 6 + dofIndex) = S_visitedDof.dot(F);

                    // For multi-DOF joints (e.g., spherical joints with 3 DOFs),
                    // compute the intra-joint off-diagonal terms: M[dof_i, dof_j] = S_j^T * F
                    // where F = Ic * S_i, for all other DOFs j of the same joint.
                    // These represent the coupling between different DOFs of the same joint.
                    for (unsigned int otherDofIdx = dofIdx + 1; otherDofIdx < nrOfDOFs;
                         otherDofIdx++)
                    {
                        SpatialMotionVector S_otherDof
                            = toParentJoint->getMotionSubspaceVector(otherDofIdx,
                                                                     visitedLink->getIndex(),
                                                                     parentLinkIndex);
                        size_t otherDofIndex = toParentJoint->getDOFsOffset() + otherDofIdx;

                        // M[i,j] = S_j^T * (Ic * S_i) = S_j^T * F
                        double coupling = S_otherDof.dot(F);
                        massMatrix(6 + dofIndex, 6 + otherDofIndex) = coupling;
                        massMatrix(6 + otherDofIndex, 6 + dofIndex) = coupling; // Symmetric
                    }

                    // Then we compute all the off-diagonal terms relative to
                    // the ancestors of the currently visited link

                    // j = i
                    LinkConstPtr ancestor = visitedLink;

                    // while lambda(j) != 0
                    while (traversal.getParentLinkFromLinkIndex(
                        traversal.getParentLinkFromLinkIndex(ancestor->getIndex())->getIndex()))
                    {
                        {
                            IJointConstPtr ancestorToParentJoint
                                = traversal.getParentJointFromLinkIndex(ancestor->getIndex());
                            LinkIndex ancestorParent
                                = traversal.getParentLinkFromLinkIndex(ancestor->getIndex())
                                      ->getIndex();
                            Transform ancestorParent_X_ancestor
                                = ancestorToParentJoint->getTransform(jointPos,
                                                                      ancestorParent,
                                                                      ancestor->getIndex());
                            F = ancestorParent_X_ancestor * F;
                        }

                        // j = \lambda(j)

                        ancestor = traversal.getParentLinkFromLinkIndex(ancestor->getIndex());

                        IJointConstPtr ancestorToParentJoint
                            = traversal.getParentJointFromLinkIndex(ancestor->getIndex());
                        LinkIndex ancestorParentIndex
                            = traversal.getParentLinkFromLinkIndex(ancestor->getIndex())->getIndex();

                        unsigned int ancestorNrOfDOFs = ancestorToParentJoint->getNrOfDOFs();
                        if (ancestorNrOfDOFs >= 1)
                        {
                            for (unsigned int ancestorDofIdx = 0; ancestorDofIdx < ancestorNrOfDOFs;
                                 ancestorDofIdx++)
                            {
                                SpatialMotionVector S_ancestorDof
                                    = ancestorToParentJoint
                                          ->getMotionSubspaceVector(ancestorDofIdx,
                                                                    ancestor->getIndex(),
                                                                    ancestorParentIndex);
                                size_t ancestorDofIndex
                                    = ancestorToParentJoint->getDOFsOffset() + ancestorDofIdx;

                                // H_ij = F^\top S_j
                                // H_ji = H_ij^\top
                                massMatrix(6 + dofIndex, 6 + ancestorDofIndex)
                                    = S_ancestorDof.dot(F);
                                massMatrix(6 + ancestorDofIndex, 6 + dofIndex)
                                    = massMatrix(6 + dofIndex, 6 + ancestorDofIndex);
                            }
                        }
                    }

                    // We need to write F in the base link for the off-diagonal term of the mass
                    // matrix
                    {
                        IJointConstPtr ancestorToParentJoint
                            = traversal.getParentJointFromLinkIndex(ancestor->getIndex());
                        LinkIndex ancestorParent
                            = traversal.getParentLinkFromLinkIndex(ancestor->getIndex())->getIndex();
                        Transform ancestorParent_X_ancestor
                            = ancestorToParentJoint->getTransform(jointPos,
                                                                  ancestorParent,
                                                                  ancestor->getIndex());
                        F = ancestorParent_X_ancestor * F;
                    }

                    // Fill the 6 \times nDof right top submatrix of the mass matrix
                    // (i.e. the jacobian of the momentum)
                    Eigen::Matrix<double, 6, 1> FEigen = toEigen(F);
                    massMatrixEigen.block<6, 1>(0, 6 + dofIndex) = FEigen;
                    massMatrixEigen.block<1, 6>(6 + dofIndex, 0) = FEigen;
                } // end of DOF loop
            }
        }
    }

    // Fill the top left 6x6 matrix: it is just the composite rigid body inertia of all the body
    Matrix6x6 lockedInertia = linkCRBs(traversal.getLink(0)->getIndex()).asMatrix();
    massMatrixEigen.block<6, 6>(0, 0) = toEigen(lockedInertia);

    return true;
}

bool CoriolisMatrixAlgorithm(const Model& model,
                             const Traversal& traversal,
                             const iDynTree::LinkPositions& linkPos,
                             const iDynTree::LinkVelArray& linkVels,
                             LinkCompositeRigidBodyInertias& linkCRBIs,
                             FreeFloatingCoriolisMatrix& coriolisMatrix,
                             FreeFloatingMassMatrix& massMatrix,
                             FreeFloatingMassMatrixDerivative& massMatrixDerivative)
{

    // Map the coriolisMatrix, massMatrix, massMatrixDerivative to an Eigen matrix
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        coriolisMatrixEigen(coriolisMatrix.data(), coriolisMatrix.rows(), coriolisMatrix.cols());
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        massMatrixEigen(massMatrix.data(), massMatrix.rows(), massMatrix.cols());
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        massMatrixDerivativeEigen(massMatrixDerivative.data(),
                                  massMatrixDerivative.rows(),
                                  massMatrixDerivative.cols());

    LinkIndex baseLinkIndex = traversal.getBaseLink()->getIndex();

    // Forward pass to initialize the CRBI
    for (int i = 0; i < traversal.getNrOfVisitedLinks(); i++)
    {
        LinkConstPtr link = traversal.getLink(i);
        LinkIndex linkIndex = link->getIndex();
        linkCRBIs(linkIndex) = link->getInertia();
    }

    // create a buffer to store bilinear factors (6x6 matrices)
    std::vector<Matrix6x6> B(traversal.getNrOfVisitedLinks());
    // initialize with zeros
    for (int i = 0; i < traversal.getNrOfVisitedLinks(); i++)
    {
        B[i].zero();
    }

    // Backward pass through traversal to compute bilinear factors and motion subspace vectors
    for (int traversalEl = traversal.getNrOfVisitedLinks() - 1; traversalEl >= 0; traversalEl--)
    {
        // get link and parent link
        LinkConstPtr visitedLink = traversal.getLink(traversalEl);
        LinkIndex visitedLinkIndex = visitedLink->getIndex();
        // get joint between parent and child
        IJointConstPtr toParentJoint = traversal.getParentJointFromLinkIndex(visitedLinkIndex);
        // get link velocity
        const Twist& linkVel = linkVels(visitedLinkIndex);
        // get link inertia
        SpatialInertia inertia = linkCRBIs(visitedLinkIndex);

        if (visitedLinkIndex != baseLinkIndex)
        { // the visited link is NOT the base link

            if (toParentJoint->getNrOfDOFs() == 0)
            {
                // fixed joint, adding its CRBI to the CRBI of the parent. This is needed to compute
                // the bilinear factors of the parent links.
                LinkIndex parentLinkIndex
                    = traversal.getParentLinkFromLinkIndex(visitedLinkIndex)->getIndex();
                Transform link_X_parentLink
                    = linkPos(visitedLinkIndex).inverse() * linkPos(parentLinkIndex);
                linkCRBIs(parentLinkIndex)
                    = linkCRBIs(parentLinkIndex)
                      + link_X_parentLink.inverse() * linkCRBIs(visitedLinkIndex);
                // nothing else to do for fixed joints
                continue;
            }
        }
        // compute bilinear factor
        // bilinearFactor = 2 * [ (v_i x*) I_i + (I_i v_i) x̄* - I_i (v_i x) ]
        Matrix6x6 bilinearFactor, term1, term2, term3;
        bilinearFactor.zero();
        // term1 = (v_i x*) I_i
        toEigen(term1)
            = toEigen(linkVel.asCrossProductMatrixWrench()) * toEigen(inertia.asMatrix());
        // term2 = (I_i v_i) x̄*
        SpatialForceVector momentum = inertia * linkVel;
        toEigen(term2) = toEigen(momentum.asCrossProductMatrix());
        // term3 = I_i (v_i x)
        toEigen(term3) = toEigen(inertia.asMatrix()) * toEigen(linkVel.asCrossProductMatrix());
        // sum terms
        toEigen(bilinearFactor) = 0.5 * (toEigen(term1) + toEigen(term2) - toEigen(term3));
        B[visitedLinkIndex] = bilinearFactor;
    }

    // Forward pass to restore the CRBI to the initial value. They were only needed to compute the
    // bilinear factors in the previous backward pass. We will compose them again in the next
    // backward pass of the algorithm.
    for (int i = 0; i < traversal.getNrOfVisitedLinks(); i++)
    {
        LinkConstPtr link = traversal.getLink(i);
        LinkIndex linkIndex = link->getIndex();
        linkCRBIs(linkIndex) = link->getInertia();
    }

    // This is the main algorithm!
    // Backward pass through traversal to compute coriolis, mass matrix, and mass matrix derivative
    for (int traversalEl = traversal.getNrOfVisitedLinks() - 1; traversalEl >= 0; traversalEl--)
    {
        // get link and parent link
        LinkConstPtr visitedLink = traversal.getLink(traversalEl);
        LinkIndex visitedLinkIndex = visitedLink->getIndex();
        LinkConstPtr parentLink = traversal.getParentLinkFromLinkIndex(visitedLinkIndex);

        if (visitedLinkIndex != baseLinkIndex)
        {
            // the visited link is NOT the base link
            IJointConstPtr toParentJoint = traversal.getParentJointFromLinkIndex(visitedLinkIndex);
            if (toParentJoint->getNrOfDOFs() > 0)
            { // is NOT a fixed joint

                // Handle multiple DOFs (e.g., spherical joints with 3 DOFs)

                // IMPORTANT: here we assume that the motion subspace vector ring derivatives
                // are zero! Check chapter 3.5 of Featherstone's "Rigid Body Dynamics Algorithms"
                // for details. For the iDynTree joints of type Revolute, Prismatic, Spherical, and
                // RevoluteSO2, this assumption holds.
                unsigned int nrOfDOFs = toParentJoint->getNrOfDOFs();

                for (unsigned int dofIdx = 0; dofIdx < nrOfDOFs; dofIdx++)
                {
                    // Compute motion subspace vector and its derivative
                    Vector6 S_visitedDof, Sdot_visitedDof;
                    toEigen(S_visitedDof)
                        = toEigen(toParentJoint->getMotionSubspaceVector(dofIdx,
                                                                         visitedLinkIndex,
                                                                         parentLink->getIndex()));
                    toEigen(Sdot_visitedDof)
                        = toEigen(linkVels(visitedLinkIndex).asCrossProductMatrix())
                          * toEigen(S_visitedDof);

                    // compute F1, F2, F3
                    Vector6 F1, F2, F3;
                    toEigen(F1)
                        = toEigen(linkCRBIs(visitedLinkIndex).asMatrix()) * toEigen(Sdot_visitedDof)
                          + toEigen(B[visitedLinkIndex]) * toEigen(S_visitedDof);
                    toEigen(F2)
                        = toEigen(linkCRBIs(visitedLinkIndex).asMatrix()) * toEigen(S_visitedDof);
                    toEigen(F3) = toEigen(B[visitedLinkIndex]).transpose() * toEigen(S_visitedDof);

                    // insert contributions to coriolis, mass matrix, and mass matrix derivative of
                    // visited link
                    const int dofIndex = toParentJoint->getDOFsOffset() + dofIdx;

                    coriolisMatrix(dofIndex + 6, dofIndex + 6)
                        = toEigen(S_visitedDof).transpose() * toEigen(F1);
                    massMatrix(dofIndex + 6, dofIndex + 6)
                        = toEigen(S_visitedDof).transpose() * toEigen(F2);
                    auto Mdot_jj
                        = toEigen(Sdot_visitedDof).transpose() * toEigen(F2)
                          + toEigen(S_visitedDof).transpose() * (toEigen(F1) + toEigen(F3));
                    assert(Mdot_jj.rows() == 1 && Mdot_jj.cols() == 1);
                    massMatrixDerivative(dofIndex + 6, dofIndex + 6) = Mdot_jj(0, 0);

                    // For multi-DOF joints (e.g., spherical joints with 3 DOFs),
                    // compute the intra-joint off-diagonal terms: C[dof_i, dof_j], etc.
                    // where for all other DOFs j of the same joint.
                    // These represent the coupling between different DOFs of the same joint.
                    for (unsigned int otherDofIdx = dofIdx + 1; otherDofIdx < nrOfDOFs;
                         otherDofIdx++)
                    {
                        Vector6 S_otherDof, Sdot_otherDof;
                        toEigen(S_otherDof) = toEigen(
                            toParentJoint->getMotionSubspaceVector(otherDofIdx,
                                                                   visitedLinkIndex,
                                                                   parentLink->getIndex()));
                        toEigen(Sdot_otherDof)
                            = toEigen(linkVels(visitedLinkIndex).asCrossProductMatrix())
                              * toEigen(S_otherDof);

                        // Compute F for the other DOF as well
                        Vector6 F1_other, F2_other, F3_other;
                        toEigen(F1_other) = toEigen(linkCRBIs(visitedLinkIndex).asMatrix())
                                                * toEigen(Sdot_otherDof)
                                            + toEigen(B[visitedLinkIndex]) * toEigen(S_otherDof);
                        toEigen(F2_other)
                            = toEigen(linkCRBIs(visitedLinkIndex).asMatrix()) * toEigen(S_otherDof);
                        toEigen(F3_other)
                            = toEigen(B[visitedLinkIndex]).transpose() * toEigen(S_otherDof);

                        size_t otherDofIndex = toParentJoint->getDOFsOffset() + otherDofIdx;

                        // Coriolis coupling: C[i,j] = S_j^T * F1_i and C[j,i] = S_i^T * F1_j
                        double coriolis_ij = toEigen(S_otherDof).transpose() * toEigen(F1);
                        double coriolis_ji = toEigen(S_visitedDof).transpose() * toEigen(F1_other);
                        coriolisMatrix(otherDofIndex + 6, dofIndex + 6) = coriolis_ij;
                        coriolisMatrix(dofIndex + 6, otherDofIndex + 6) = coriolis_ji;

                        // Mass matrix coupling: M[i,j] = S_j^T * F2_i = M[j,i]
                        // Note: M is symmetric
                        double mass_coupling = toEigen(S_otherDof).transpose() * toEigen(F2);
                        massMatrix(otherDofIndex + 6, dofIndex + 6) = mass_coupling;
                        massMatrix(dofIndex + 6, otherDofIndex + 6) = mass_coupling; // Symmetric

                        // Mass matrix derivative coupling
                        // Mdot[i,j] = Sdot_j^T * F2_i + S_j^T * (F1_i + F3_i)
                        // Mdot[j,i] = Sdot_i^T * F2_j + S_i^T * (F1_j + F3_j)
                        auto Mdot_ij
                            = toEigen(Sdot_otherDof).transpose() * toEigen(F2)
                              + toEigen(S_otherDof).transpose() * (toEigen(F1) + toEigen(F3));
                        auto Mdot_ji = toEigen(Sdot_visitedDof).transpose() * toEigen(F2_other)
                                       + toEigen(S_visitedDof).transpose()
                                             * (toEigen(F1_other) + toEigen(F3_other));
                        assert(Mdot_ij.rows() == 1 && Mdot_ij.cols() == 1);
                        assert(Mdot_ji.rows() == 1 && Mdot_ji.cols() == 1);
                        massMatrixDerivative(otherDofIndex + 6, dofIndex + 6) = Mdot_ij(0, 0);
                        massMatrixDerivative(dofIndex + 6, otherDofIndex + 6) = Mdot_ji(0, 0);
                    }

                    // Compute off-diagonal terms of the coriolis, mass matrix, and mass matrix
                    // derivative related to ancestor links (excluding the base link)
                    LinkIndex ancestorLinkIndex = visitedLinkIndex;
                    Vector6 F1_ancestor = F1;
                    Vector6 F2_ancestor = F2;
                    Vector6 F3_ancestor = F3;

                    while (traversal.getParentLinkFromLinkIndex(
                        traversal.getParentLinkFromLinkIndex(ancestorLinkIndex)->getIndex()))
                    {
                        LinkConstPtr ancestorParentLink
                            = traversal.getParentLinkFromLinkIndex(ancestorLinkIndex);
                        LinkIndex ancestorParentIndex = ancestorParentLink->getIndex();

                        Transform ancestorLink_X_parentLink
                            = linkPos(ancestorLinkIndex).inverse() * linkPos(ancestorParentIndex);
                        toEigen(F1_ancestor)
                            = toEigen(ancestorLink_X_parentLink.asAdjointTransform()).transpose()
                              * toEigen(F1_ancestor);
                        toEigen(F2_ancestor)
                            = toEigen(ancestorLink_X_parentLink.asAdjointTransform()).transpose()
                              * toEigen(F2_ancestor);
                        toEigen(F3_ancestor)
                            = toEigen(ancestorLink_X_parentLink.asAdjointTransform()).transpose()
                              * toEigen(F3_ancestor);

                        Vector6 S_ancestorParentDof, Sdot_ancestorParentDof;

                        // check if parent link is connected to its parent through a fixed joint
                        IJointConstPtr toGrandParentJoint
                            = traversal.getParentJointFromLinkIndex(ancestorParentIndex);
                        if (toGrandParentJoint->getNrOfDOFs() == 0)
                        {
                            // the parent link is connected to its parent through a fixed joint
                            // nothing left to do, continue upwards
                            ancestorLinkIndex = ancestorParentIndex;
                            continue;
                        } else
                        {
                            // Handle multiple DOFs in ancestor joint
                            unsigned int ancestorNrOfDOFs = toGrandParentJoint->getNrOfDOFs();
                            for (unsigned int ancestorDofIdx = 0; ancestorDofIdx < ancestorNrOfDOFs;
                                 ancestorDofIdx++)
                            {
                                // compute ancestor parent motion subspace vector and its derivative
                                toEigen(S_ancestorParentDof) = toEigen(
                                    toGrandParentJoint
                                        ->getMotionSubspaceVector(ancestorDofIdx,
                                                                  ancestorParentIndex,
                                                                  traversal
                                                                      .getParentLinkFromLinkIndex(
                                                                          ancestorParentIndex)
                                                                      ->getIndex()));
                                toEigen(Sdot_ancestorParentDof)
                                    = toEigen(linkVels(ancestorParentIndex).asCrossProductMatrix())
                                      * toEigen(S_ancestorParentDof);

                                // insert contributions to coriolis, mass matrix, and mass matrix
                                // derivative of ancestor parent link
                                const int ancestorDofIndex
                                    = toGrandParentJoint->getDOFsOffset() + ancestorDofIdx;

                                coriolisMatrix(ancestorDofIndex + 6, dofIndex + 6)
                                    = toEigen(S_ancestorParentDof).transpose()
                                      * toEigen(F1_ancestor);
                                coriolisMatrix(dofIndex + 6, ancestorDofIndex + 6)
                                    = (toEigen(F2_ancestor).transpose()
                                           * toEigen(Sdot_ancestorParentDof)
                                       + toEigen(F3_ancestor).transpose()
                                             * toEigen(S_ancestorParentDof))(0, 0);
                                auto M_ij = toEigen(S_ancestorParentDof).transpose()
                                            * toEigen(F2_ancestor);
                                assert(M_ij.rows() == 1 && M_ij.cols() == 1);
                                massMatrix(ancestorDofIndex + 6, dofIndex + 6) = M_ij(0, 0);
                                massMatrix(dofIndex + 6, ancestorDofIndex + 6) = M_ij(0, 0);
                                auto Mdot_ij
                                    = toEigen(Sdot_ancestorParentDof).transpose()
                                          * toEigen(F2_ancestor)
                                      + toEigen(S_ancestorParentDof).transpose()
                                            * (toEigen(F1_ancestor) + toEigen(F3_ancestor));
                                assert(Mdot_ij.rows() == 1 && Mdot_ij.cols() == 1);
                                massMatrixDerivative(ancestorDofIndex + 6, dofIndex + 6)
                                    = Mdot_ij(0, 0);
                                massMatrixDerivative(dofIndex + 6, ancestorDofIndex + 6)
                                    = Mdot_ij(0, 0);
                            }
                        }

                        ancestorLinkIndex = ancestorParentIndex;
                    }

                    // Fill the 6 \times nDof right top submatrix of the coriolis, mass matrix, and
                    // mass matrix derivative related to the base link
                    LinkConstPtr ancestorParentLink
                        = traversal.getParentLinkFromLinkIndex(ancestorLinkIndex);
                    LinkIndex ancestorParentIndex = ancestorParentLink->getIndex();
                    assert(ancestorParentIndex == baseLinkIndex);

                    Transform otherLink_X_parentLink
                        = linkPos(ancestorLinkIndex).inverse() * linkPos(ancestorParentIndex);
                    toEigen(F1_ancestor)
                        = toEigen(otherLink_X_parentLink.asAdjointTransform()).transpose()
                          * toEigen(F1_ancestor);
                    toEigen(F2_ancestor)
                        = toEigen(otherLink_X_parentLink.asAdjointTransform()).transpose()
                          * toEigen(F2_ancestor);
                    toEigen(F3_ancestor)
                        = toEigen(otherLink_X_parentLink.asAdjointTransform()).transpose()
                          * toEigen(F3_ancestor);

                    coriolisMatrixEigen.block<6, 1>(0, 6 + dofIndex) = toEigen(F1_ancestor);
                    coriolisMatrixEigen.block<1, 6>(6 + dofIndex, 0)
                        = toEigen(F3_ancestor).transpose();
                    massMatrixEigen.block<6, 1>(0, 6 + dofIndex) = toEigen(F2_ancestor);
                    massMatrixEigen.block<1, 6>(6 + dofIndex, 0) = toEigen(F2_ancestor).transpose();
                    massMatrixDerivativeEigen.block<6, 1>(0, 6 + dofIndex)
                        = toEigen(F1_ancestor) + toEigen(F3_ancestor);
                    massMatrixDerivativeEigen.block<1, 6>(6 + dofIndex, 0)
                        = (toEigen(F1_ancestor) + toEigen(F3_ancestor)).transpose();
                } // end of DOF loop
            } // end if not fixed joint

            // update CRBIs and bilinear factors of parent link to include contribution of visited
            // link
            LinkIndex parentLinkIndex = parentLink->getIndex();
            Transform link_X_parentLink
                = linkPos(visitedLinkIndex).inverse() * linkPos(parentLinkIndex);
            // update CRBIs inertias
            linkCRBIs(parentLinkIndex)
                = linkCRBIs(parentLinkIndex)
                  + link_X_parentLink.inverse() * linkCRBIs(visitedLinkIndex);

            // update bilinear factors
            toEigen(B[parentLinkIndex])
                += toEigen(link_X_parentLink.asAdjointTransform()).transpose()
                   * toEigen(B[visitedLinkIndex]) * toEigen(link_X_parentLink.asAdjointTransform());
        } // end if not base link
        else
        {
            // the visited link is the base link
            // fill in the top-left 6x6 blocks of coriolis, mass matrix, and mass matrix derivative
            coriolisMatrixEigen.block<6, 6>(0, 0) = toEigen(B[visitedLinkIndex]);
            massMatrixEigen.block<6, 6>(0, 0) = toEigen(linkCRBIs(visitedLinkIndex).asMatrix());
            massMatrixDerivativeEigen.block<6, 6>(0, 0)
                = toEigen(B[visitedLinkIndex]) + toEigen(B[visitedLinkIndex]).transpose();
        }
    }
    return true;
}

ArticulatedBodyAlgorithmInternalBuffers::ArticulatedBodyAlgorithmInternalBuffers(const Model& model)
{
    resize(model);
}

void ArticulatedBodyAlgorithmInternalBuffers::resize(const Model& model)
{
    S.resize(model);
    U.resize(model);
    D.resize(model);
    u.resize(model);
    linksVel.resize(model);
    linksBiasAcceleration.resize(model);
    linksAccelerations.resize(model);
    linkABIs.resize(model);
    linksBiasWrench.resize(model);
    // debug
    // pa.resize(model);
}

bool ArticulatedBodyAlgorithmInternalBuffers::isConsistent(const Model& model)
{
    bool ok = true;

    ok = ok && S.isConsistent(model);
    ok = ok && U.isConsistent(model);
    ok = ok && D.isConsistent(model);
    ok = ok && u.isConsistent(model);
    ok = ok && linksVel.isConsistent(model);
    ok = ok && linksBiasAcceleration.isConsistent(model);
    ok = ok && linkABIs.isConsistent(model);
    ok = ok && linksBiasWrench.isConsistent(model);

    return ok;
}

bool ArticulatedBodyAlgorithm(const Model& model,
                              const Traversal& traversal,
                              const FreeFloatingPos& robotPos,
                              const FreeFloatingVel& robotVel,
                              const LinkNetExternalWrenches& linkExtWrenches,
                              const JointDOFsDoubleArray& jointTorques,
                              ArticulatedBodyAlgorithmInternalBuffers& bufs,
                              FreeFloatingAcc& robotAcc)
{
    /**
     * Forward pass: compute the link velocities and the link bias accelerations
     * and initialize the Articulated Body Inertia and the articulated bias wrench.
     */
    for (unsigned int traversalEl = 0; traversalEl < traversal.getNrOfVisitedLinks(); traversalEl++)
    {
        LinkConstPtr visitedLink = traversal.getLink(traversalEl);
        LinkIndex visitedLinkIndex = visitedLink->getIndex();
        LinkConstPtr parentLink = traversal.getParentLink(traversalEl);
        IJointConstPtr toParentJoint = traversal.getParentJoint(traversalEl);

        // Propagate velocities and bias accelerations

        // If the visitedLink is the base one, initialize the velocity with the input velocity
        if (parentLink == 0)
        {
            assert(visitedLinkIndex >= 0 && visitedLinkIndex < (LinkIndex)model.getNrOfLinks());
            bufs.linksVel(visitedLinkIndex) = robotVel.baseVel();
            bufs.linksBiasAcceleration(visitedLinkIndex) = SpatialAcc::Zero();
        } else
        {
            LinkIndex parentLinkIndex = parentLink->getIndex();
            // Otherwise we propagate velocity in the usual way
            // Uniform handling for any number of DOFs (including 0)
            Twist vj = Twist::Zero();
            const unsigned int nrOfDOFs = toParentJoint->getNrOfDOFs();
            for (unsigned int dofIdx = 0; dofIdx < nrOfDOFs; dofIdx++)
            {
                size_t globalDofIndex = toParentJoint->getDOFsOffset() + dofIdx;
                bufs.S(globalDofIndex) = toParentJoint->getMotionSubspaceVector(dofIdx,
                                                                                visitedLinkIndex,
                                                                                parentLinkIndex);

                // Accumulate joint velocity contribution
                Twist vj_dof;
                toEigen(vj_dof.getLinearVec3()) = robotVel.jointVel()(globalDofIndex)
                                                  * toEigen(bufs.S(globalDofIndex).getLinearVec3());
                toEigen(vj_dof.getAngularVec3())
                    = robotVel.jointVel()(globalDofIndex)
                      * toEigen(bufs.S(globalDofIndex).getAngularVec3());
                vj = vj + vj_dof;
            }

            bufs.linksVel(visitedLinkIndex) = toParentJoint->getTransform(robotPos.jointPos(),
                                                                          visitedLinkIndex,
                                                                          parentLinkIndex)
                                                  * bufs.linksVel(parentLinkIndex)
                                              + vj;

            // Zero when nrOfDOFs == 0
            bufs.linksBiasAcceleration(visitedLinkIndex) = bufs.linksVel(visitedLinkIndex) * vj;
        }

        // Initialize Articulated Body Inertia
        bufs.linkABIs(visitedLinkIndex) = visitedLink->getInertia();

        // Initialize bias force (note that we assume that the external frames are
        // expressed in a local frame, differently from Featherstone 2008
        bufs.linksBiasWrench(visitedLinkIndex)
            = bufs.linksVel(visitedLinkIndex)
                  * (visitedLink->getInertia() * bufs.linksVel(visitedLinkIndex))
              - linkExtWrenches(visitedLinkIndex);
    }

    /*
     * Backward pass: recursivly compute
     * the articulated body inertia and the articulated body bias wrench.
     *
     */
    for (int traversalEl = traversal.getNrOfVisitedLinks() - 1; traversalEl >= 0; traversalEl--)
    {
        LinkConstPtr visitedLink = traversal.getLink(traversalEl);
        LinkIndex visitedLinkIndex = visitedLink->getIndex();
        LinkConstPtr parentLink = traversal.getParentLink(traversalEl);
        IJointConstPtr toParentJoint = traversal.getParentJoint(traversalEl);

        if (parentLink)
        {
            ArticulatedBodyInertia Ia;
            Wrench pa;

            // For now we support only 0 and 1 dof joints
            if (toParentJoint->getNrOfDOFs() > 1)
            {
                return false;
            }

            // for 1 dof joints, the articulated inertia
            // need to be propagated to the parent considering
            // the joints
            if (toParentJoint->getNrOfDOFs() > 0)
            {
                size_t dofIndex = toParentJoint->getDOFsOffset();
                bufs.U(dofIndex) = bufs.linkABIs(visitedLinkIndex) * bufs.S(dofIndex);
                bufs.D(dofIndex) = bufs.S(dofIndex).dot(bufs.U(dofIndex));
                bufs.u(dofIndex) = jointTorques(dofIndex)
                                   - bufs.S(dofIndex).dot(bufs.linksBiasWrench(visitedLinkIndex));

                Ia = bufs.linkABIs(visitedLinkIndex)
                     - ArticulatedBodyInertia::ABADyadHelper(bufs.U(dofIndex), bufs.D(dofIndex));

                pa = bufs.linksBiasWrench(visitedLinkIndex)
                     + Ia * bufs.linksBiasAcceleration(visitedLinkIndex)
                     + bufs.U(dofIndex) * (bufs.u(dofIndex) / bufs.D(dofIndex));
            }

            // For fixed joints, we just need to propate
            // the articulated quantities without considering the
            // joint
            if (toParentJoint->getNrOfDOFs() == 0)
            {
                Ia = bufs.linkABIs(visitedLinkIndex);
                pa = bufs.linksBiasWrench(visitedLinkIndex)
                     + Ia * bufs.linksBiasAcceleration(visitedLinkIndex);
            }

            // bufs.pa(visitedLinkIndex) = pa;

            // Propagate
            LinkIndex parentLinkIndex = parentLink->getIndex();
            Transform parent_X_visited = toParentJoint->getTransform(robotPos.jointPos(),
                                                                     parentLinkIndex,
                                                                     visitedLinkIndex);
            bufs.linkABIs(parentLinkIndex) += parent_X_visited * Ia;
            bufs.linksBiasWrench(parentLinkIndex)
                = bufs.linksBiasWrench(parentLinkIndex) + parent_X_visited * pa;
        }
    }

    /**
     * Second forward pass: find robot accelerations
     *
     */
    // \todo TODO Handle gravity
    for (unsigned int traversalEl = 0; traversalEl < traversal.getNrOfVisitedLinks(); traversalEl++)
    {
        LinkConstPtr visitedLink = traversal.getLink(traversalEl);
        LinkIndex visitedLinkIndex = visitedLink->getIndex();
        LinkConstPtr parentLink = traversal.getParentLink(traversalEl);
        IJointConstPtr toParentJoint = traversal.getParentJoint(traversalEl);

        if (parentLink == 0)
        {
            // Preliminary step: find base acceleration
            bufs.linksAccelerations(visitedLinkIndex)
                = -(bufs.linkABIs(visitedLinkIndex)
                        .applyInverse(bufs.linksBiasWrench(visitedLinkIndex)));

            robotAcc.baseAcc() = bufs.linksAccelerations(visitedLinkIndex);
        } else
        {
            LinkIndex parentLinkIndex = parentLink->getIndex();
            if (toParentJoint->getNrOfDOFs() > 0)
            {
                size_t dofIndex = toParentJoint->getDOFsOffset();
                assert(toParentJoint->getNrOfDOFs() == 1);
                bufs.linksAccelerations(visitedLinkIndex)
                    = toParentJoint->getTransform(robotPos.jointPos(),
                                                  visitedLinkIndex,
                                                  parentLinkIndex)
                          * bufs.linksAccelerations(parentLinkIndex)
                      + bufs.linksBiasAcceleration(visitedLinkIndex);
                robotAcc.jointAcc()(dofIndex)
                    = (bufs.u(dofIndex)
                       - bufs.U(dofIndex).dot(bufs.linksAccelerations(visitedLinkIndex)))
                      / bufs.D(dofIndex);
                bufs.linksAccelerations(visitedLinkIndex)
                    = bufs.linksAccelerations(visitedLinkIndex)
                      + bufs.S(dofIndex) * robotAcc.jointAcc()(dofIndex);
            } else
            {
                // for fixed joints we just propagate the acceleration
                bufs.linksAccelerations(visitedLinkIndex)
                    = toParentJoint->getTransform(robotPos.jointPos(),
                                                  visitedLinkIndex,
                                                  parentLinkIndex)
                          * bufs.linksAccelerations(parentLinkIndex)
                      + bufs.linksBiasAcceleration(visitedLinkIndex);
            }
        }
    }

    return true;
}

bool InverseDynamicsInertialParametersRegressor(
    const iDynTree::Model& model,
    const iDynTree::Traversal& traversal,
    const iDynTree::LinkPositions& referenceFrame_H_link,
    const iDynTree::LinkVelArray& linksVel,
    const iDynTree::LinkAccArray& linksProperAcc,
    iDynTree::MatrixDynSize& baseForceAndJointTorquesRegressor)
{
    baseForceAndJointTorquesRegressor.resize(6 + model.getNrOfDOFs(), 10 * model.getNrOfLinks());
    baseForceAndJointTorquesRegressor.zero();

    // Eigen map of the input matrix
    iDynTreeEigenMatrixMap regressor = iDynTree::toEigen(baseForceAndJointTorquesRegressor);

    Matrix6x10 netForceTorqueRegressor_i;

    for (TraversalIndex l = (TraversalIndex)traversal.getNrOfVisitedLinks() - 1; l >= 0; l--)
    {

        // In this cycle, we compute the contribution of the inertial parameters of link to the
        // inverse dynamics results
        LinkConstPtr link = traversal.getLink(l);
        LinkIndex lnkIdx = link->getIndex();

        // Each link affects the dynamics of the joints from itself to the base
        netForceTorqueRegressor_i
            = SpatialInertia::momentumDerivativeRegressor(linksVel(lnkIdx), linksProperAcc(lnkIdx));

        iDynTreeEigenMatrix linkRegressor = iDynTree::toEigen(netForceTorqueRegressor_i);

        // Base dynamics
        // The base dynamics is expressed with the orientation of the base and with respect to the
        // base origin
        regressor.block(0, (int)(10 * lnkIdx), 6, 10)
            = toEigen(referenceFrame_H_link(lnkIdx).asAdjointTransformWrench()) * linkRegressor;

        LinkIndex visitedLinkIdx = lnkIdx;

        while (visitedLinkIdx != traversal.getBaseLink()->getIndex())
        {
            LinkIndex parentLinkIdx
                = traversal.getParentLinkFromLinkIndex(visitedLinkIdx)->getIndex();
            IJointConstPtr joint = traversal.getParentJointFromLinkIndex(visitedLinkIdx);

            size_t dofOffset = joint->getDOFsOffset();
            for (int i = 0; i < joint->getNrOfDOFs(); i++)
            {
                iDynTree::SpatialMotionVector S
                    = joint->getMotionSubspaceVector(i, visitedLinkIdx, parentLinkIdx);
                Transform visitedLink_H_link = referenceFrame_H_link(visitedLinkIdx).inverse()
                                               * referenceFrame_H_link(lnkIdx);
                regressor.block(6 + dofOffset + i, 10 * lnkIdx, 1, 10)
                    = toEigen(S).transpose()
                      * (toEigen(visitedLink_H_link.asAdjointTransformWrench()) * linkRegressor);
            }

            visitedLinkIdx = parentLinkIdx;
        }
    }

    return true;
}

} // namespace iDynTree
