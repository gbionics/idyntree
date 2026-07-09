
/* File : iDynTree.i */
%module iDynTree
#ifdef SWIGPYTHON
%implicitconv;
#endif

%include "std_string.i"
%include "std_unordered_map.i"
%include "std_vector.i"

// std::shared_ptr holder is currently supported only for python bindings
#ifdef SWIGPYTHON
%include "std_shared_ptr.i"
#endif

// Wrap the std::vector<std::string> params
%template(StringVector) std::vector<std::string>;

// Wrap the std::vector<std::int> params
%template(IntVector) std::vector<int>;

// Wrap the std::unordered_map<std::string, double>
%template(StringToDoubleUnorderedMap) std::unordered_map<std::string, double>;

// Wrap the std::unordered_map<std::string, std::vector<double>>
%template(StringToDoubleVectorUnorderedMap) std::unordered_map<std::string, std::vector<double>>;

// Ignore some methods to avoid warnings
%include "./ignore.i"


// Python
#ifdef SWIGPYTHON
%include "./python/python.i"
#endif

// Matlab
#ifdef SWIGMATLAB
%include "./matlab/matlab.i"
#endif

%{
/* Note : always include headers following the inheritance order */
#include <cmath>
#include <cstddef>

//Utils
#include "iDynTree/Utils.h"
#include "iDynTree/URIUtils.h"

// Basic math classes
#include "iDynTree/MatrixDynSize.h"
#include "iDynTree/MatrixFixSize.h"
#include "iDynTree/SparseMatrix.h"

#include "iDynTree/VectorDynSize.h"
#include "iDynTree/VectorFixSize.h"

// Basic Vectors: Point Vectors and Spatial Vectors
#include "iDynTree/Position.h"
#include "iDynTree/SpatialForceVector.h"
#include "iDynTree/SpatialMotionVector.h"
#include "iDynTree/Twist.h"
#include "iDynTree/Wrench.h"
#include "iDynTree/SpatialMomentum.h"
#include "iDynTree/SpatialAcc.h"
#include "iDynTree/ClassicalAcc.h"
#include "iDynTree/Direction.h"
#include "iDynTree/Axis.h"

// Inertias
#include "iDynTree/RotationalInertia.h"
#include "iDynTree/SpatialInertia.h"
#include "iDynTree/SpatialInertia.h"
#include "iDynTree/ArticulatedBodyInertia.h"
#include "iDynTree/InertiaNonLinearParametrization.h"

// Transformations: Rotation and Transform
#include "iDynTree/Rotation.h"
#include "iDynTree/Transform.h"
#include "iDynTree/TransformDerivative.h"
#include "iDynTree/Span.h"

// Model related data structures
#include "iDynTree/Indices.h"
#include "iDynTree/LinkState.h"
#include "iDynTree/Link.h"
#include "iDynTree/IJoint.h"
#include "iDynTree/FixedJoint.h"
#include "iDynTree/MovableJointImpl.h"
#include "iDynTree/RevoluteJoint.h"
#include "iDynTree/SphericalJoint.h"
#include "iDynTree/PrismaticJoint.h"
#include "iDynTree/Traversal.h"
#include "iDynTree/SolidShapes.h"
#include "iDynTree/Sensors.h"
#include "iDynTree/Model.h"
#include "iDynTree/JointState.h"
#include "iDynTree/FreeFloatingMatrices.h"
#include "iDynTree/FreeFloatingState.h"
#include "iDynTree/ContactWrench.h"
#include "iDynTree/ModelTestUtils.h"
#include "iDynTree/ModelTransformers.h"
#include "iDynTree/SubModel.h"
#include "iDynTree/SixAxisForceTorqueSensor.h"
#include "iDynTree/AccelerometerSensor.h"
#include "iDynTree/GyroscopeSensor.h"
#include "iDynTree/ThreeAxisAngularAccelerometerSensor.h"
#include "iDynTree/ThreeAxisForceTorqueContactSensor.h"
#include "iDynTree/PredictSensorsMeasurements.h"

// Kinematics & Dynamics related functions
#include "iDynTree/ForwardKinematics.h"
#include "iDynTree/Dynamics.h"
#include "iDynTree/DenavitHartenberg.h"

// Model loading from external formats
#include "iDynTree/URDFDofsImport.h"
#include "iDynTree/ModelLoader.h"
#include "iDynTree/ModelExporter.h"
#include "iDynTree/ModelCalibrationHelper.h"


// Estimation related classes
#include "iDynTree/ExternalWrenchesEstimation.h"
#include "iDynTree/ExtWrenchesAndJointTorquesEstimator.h"
#include "iDynTree/SimpleLeggedOdometry.h"
#include "iDynTree/BerdyHelper.h"
#include "iDynTree/BerdySparseMAPSolver.h"
#include "iDynTree/AttitudeEstimator.h"
#include "iDynTree/AttitudeMahonyFilter.h"
#include "iDynTree/ExtendedKalmanFilter.h"
#include "iDynTree/AttitudeQuaternionEKF.h"

// SolidShapes related classes
#include "iDynTree/InertialParametersSolidShapesHelpers.h"

// High level interfaces
#include "iDynTree/KinDynComputations.h"

// Visualization
#include "iDynTree/Visualizer.h"

// Inverse Kinematics
#include "iDynTree/ConvexHullHelpers.h"
#include "iDynTree/InverseKinematics.h"

%}

// Wrap the std::vector<iDynTree::MatrixDynSize> params
%template(MatrixDynSizeVector) std::vector<iDynTree::MatrixDynSize>;

// Wrap the std::vector<iDynTree::VectorDynSize> params
%template(VectorDynSizeVector) std::vector<iDynTree::VectorDynSize>;

// Wrap the:
// * std::vector<std::ptrdiff_t>,
// * std::vector<iDynTree::LinkIndex>,
// * std::vector<iDynTree::JointIndex>,
// * std::vector<iDynTree::DOFIndex>,
// * std::vector<iDynTree::FrameIndex>,
// * std::vector<iDynTree::TraversalIndex>,
// params
namespace std {
    typedef ::ptrdiff_t ptrdiff_t;
}
%template(IndexVector) std::vector<std::ptrdiff_t>;

//Wrap std::vector<BerdySensors>
namespace std {
    %template(BerdySensors) vector<iDynTree::BerdySensor>;
    %template(BerdyDynamicVariables) vector<iDynTree::BerdyDynamicVariable>;
}

//Utils
%include "iDynTree/Utils.h"
%include "iDynTree/URIUtils.h"

/* Note : always include headers following the inheritance order */
// Basic math classes
%include "iDynTree/MatrixDynSize.h"
%include "iDynTree/MatrixFixSize.h"
%include "iDynTree/SparseMatrix.h"
%template(SparseMatrixRowMajor) iDynTree::SparseMatrix<iDynTree::RowMajor>;
%template(SparseMatrixColMajor) iDynTree::SparseMatrix<iDynTree::ColumnMajor>;


%include "iDynTree/VectorDynSize.h"
%include "iDynTree/VectorFixSize.h"

#ifdef SWIGMATLAB
%include "./matlab/matlab_matvec.i"
#endif

#ifdef SWIGOCTAVE
%include "./octave/octave_matvec.i"
#endif

%template(Matrix1x6) iDynTree::MatrixFixSize<1,6>;
%template(Matrix2x3) iDynTree::MatrixFixSize<2,3>;
%template(Matrix3x3) iDynTree::MatrixFixSize<3,3>;
%template(Matrix4x4) iDynTree::MatrixFixSize<4,4>;
%template(Matrix6x6) iDynTree::MatrixFixSize<6,6>;
%template(Matrix6x10) iDynTree::MatrixFixSize<6,10>;
%template(Matrix10x16) iDynTree::MatrixFixSize<10,16>;

%template(Vector3) iDynTree::VectorFixSize<3>;
%template(Vector4) iDynTree::VectorFixSize<4>;
%template(Vector6) iDynTree::VectorFixSize<6>;
%template(Vector10) iDynTree::VectorFixSize<10>;
%template(Vector16) iDynTree::VectorFixSize<16>;

// Basic Vectors: Point Vectors and Spatial Vectors
%include "iDynTree/Position.h"

%include "iDynTree/GeomVector3.h"

%include "iDynTree/SpatialVector.h"

%template() iDynTree::DualSpace<iDynTree::SpatialMotionVector>;
%template() iDynTree::DualSpace<iDynTree::SpatialForceVector>;

#ifdef SWIGMATLAB
%include "./matlab/matlab_spatialvec.i"
#endif

#ifdef SWIGOCTAVE
%include "./octave/octave_spatialvec.i"
#endif

%template(SpatialMotionVectorBase) iDynTree::SpatialVector<iDynTree::SpatialMotionVector>;
%template(SpatialForceVectorBase) iDynTree::SpatialVector<iDynTree::SpatialForceVector>;

%include "iDynTree/SpatialMotionVector.h"
%include "iDynTree/SpatialForceVector.h"
%include "iDynTree/Twist.h"
%include "iDynTree/Wrench.h"
%include "iDynTree/SpatialMomentum.h"
%include "iDynTree/SpatialAcc.h"
%include "iDynTree/ClassicalAcc.h"
%include "iDynTree/Direction.h"
%include "iDynTree/Axis.h"

// Inertias
%include "iDynTree/RotationalInertia.h"
%include "iDynTree/SpatialInertia.h"
%include "iDynTree/ArticulatedBodyInertia.h"
%include "iDynTree/InertiaNonLinearParametrization.h"

// Transformations: Rotation and Transform
%include "iDynTree/Rotation.h"

// getRotation()/getPosition() return a reference into this Transform, not a copy.
// Chained on a temporary (e.g. getWorldTransform(f).getPosition().toNumPy()), the
// Transform gets GC'd before the child is used, leaving it pointing at freed memory
// (https://github.com/gbionics/idyntree/issues/1263).
// In this way we keep the parent alive by attaching it to the child below.
#ifdef SWIGPYTHON
%feature("pythonappend") iDynTree::Transform::getRotation() const %{
    val.__idyntree_owner_keepalive = self
%}
%feature("pythonappend") iDynTree::Transform::getPosition() const %{
    val.__idyntree_owner_keepalive = self
%}
#endif
%include "iDynTree/Transform.h"
%include "iDynTree/TransformDerivative.h"
%include "iDynTree/Span.h"
%include "iDynTree/MatrixView.h"

%template(DynamicSpan) iDynTree::Span<double, iDynTree::dynamic_extent>;
%template(DynamicMatrixView) iDynTree::MatrixView<double>;

// Model related data structures
%include "iDynTree/Indices.h"
%include "iDynTree/LinkState.h"
%include "iDynTree/Link.h"
%include "iDynTree/IJoint.h"
%include "iDynTree/FixedJoint.h"
%include "iDynTree/MovableJointImpl.h"

%template(MovableJointImpl1) iDynTree::MovableJointImpl<1,1>;
%template(MovableJointImpl2) iDynTree::MovableJointImpl<2,2>;
%template(MovableJointImpl3) iDynTree::MovableJointImpl<3,3>;
%template(MovableJointImpl4) iDynTree::MovableJointImpl<4,4>;
%template(MovableJointImpl4_3) iDynTree::MovableJointImpl<4,3>;
%template(MovableJointImpl5) iDynTree::MovableJointImpl<5,5>;
%template(MovableJointImpl6) iDynTree::MovableJointImpl<6,6>;

%include "iDynTree/RevoluteJoint.h"
%include "iDynTree/SphericalJoint.h"
%include "iDynTree/PrismaticJoint.h"
%include "iDynTree/Traversal.h"
%include "iDynTree/SolidShapes.h"
%include "iDynTree/Sensors.h"
%include "iDynTree/Model.h"
%include "iDynTree/JointState.h"
%include "iDynTree/FreeFloatingMatrices.h"
%include "iDynTree/FreeFloatingState.h"
%include "iDynTree/ContactWrench.h"
%include "iDynTree/ModelTestUtils.h"
%include "iDynTree/ModelTransformers.h"
%include "iDynTree/SubModel.h"
%include "iDynTree/SixAxisForceTorqueSensor.h"
%include "iDynTree/AccelerometerSensor.h"
%include "iDynTree/GyroscopeSensor.h"
%include "iDynTree/ThreeAxisAngularAccelerometerSensor.h"
%include "iDynTree/ThreeAxisForceTorqueContactSensor.h"
%include "iDynTree/PredictSensorsMeasurements.h"

%include "sensors.i"
%include "joints.i"

%template(SolidShapesVector) std::vector<iDynTree::SolidShape*>;
%template(LinksSolidShapesVector) std::vector< std::vector<iDynTree::SolidShape *>>;


// Kinematics & Dynamics related functions
%include "iDynTree/ForwardKinematics.h"
%include "iDynTree/Dynamics.h"
%include "iDynTree/DenavitHartenberg.h"

// Model loading from external formats
%include "iDynTree/URDFDofsImport.h"
%include "iDynTree/ModelLoader.h"
%include "iDynTree/ModelExporter.h"
%include "iDynTree/ModelCalibrationHelper.h"

// Estimation related classes
%include "iDynTree/ExternalWrenchesEstimation.h"
%include "iDynTree/ExtWrenchesAndJointTorquesEstimator.h"
%include "iDynTree/SimpleLeggedOdometry.h"
%include "iDynTree/BerdyHelper.h"
%include "iDynTree/BerdySparseMAPSolver.h"
%include "iDynTree/AttitudeEstimator.h"
%include "iDynTree/AttitudeMahonyFilter.h"
%include "iDynTree/ExtendedKalmanFilter.h"
%include "iDynTree/AttitudeQuaternionEKF.h"

// SolidShapes related classes
%include "iDynTree/InertialParametersSolidShapesHelpers.h"

// High level interfaces
#ifdef SWIGPYTHON
%shared_ptr(iDynTree::KinDynComputations)
#endif
%include "iDynTree/KinDynComputations.h"

#ifdef SWIGMATLAB
%include "./matlab/matlab_mat4x4vec.i"
#endif

%template(Matrix4x4Vector) std::vector<iDynTree::Matrix4x4>;


// Visualization
%include "iDynTree/Visualizer.h"

// Inverse Kinematics
%include "iDynTree/ConvexHullHelpers.h"
%include "iDynTree/InverseKinematics.h"
