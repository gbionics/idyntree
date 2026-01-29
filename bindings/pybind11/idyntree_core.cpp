#include "idyntree_core.h"

#include <iDynTree/Axis.h>
#include <iDynTree/Direction.h>
#include <iDynTree/MatrixDynSize.h>
#include <iDynTree/MatrixFixSize.h>
#include <iDynTree/Position.h>
#include <iDynTree/Rotation.h>
#include <iDynTree/RotationalInertia.h>
#include <iDynTree/SpatialInertia.h>
#include <iDynTree/Transform.h>
#include <iDynTree/Twist.h>
#include <iDynTree/URIUtils.h>
#include <iDynTree/VectorDynSize.h>
#include <iDynTree/VectorFixSize.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <array>
#include <cstddef>

namespace iDynTree
{
namespace bindings
{
namespace
{
namespace py = ::pybind11;

void vectorDynSizeClassDefinition(py::class_<VectorDynSize>& vector)
{
    vector.def(py::init())
        .def(py::init<unsigned>())
        .def("__getitem__",
             py::overload_cast<const std::size_t>(&VectorDynSize::operator(), py::const_))
        .def("__setitem__",
             [](VectorDynSize& the_vector, std::size_t index, double new_value) {
                 the_vector(index) = new_value;
             })
        .def(
            "__iter__",
            [](const VectorDynSize& s) { return py::make_iterator(s.begin(), s.end()); },
            py::keep_alive<0, 1>() /* Essential: keep object alive while iterator exists */)
        .def("__len__", &VectorDynSize::size)
        .def("set_zero", &VectorDynSize::zero)
        .def("resize", &VectorDynSize::resize)
        .def("__repr__", &VectorDynSize::toString)
        .def_buffer([](VectorDynSize& v) -> py::buffer_info {
            return py::buffer_info(v.data(), /* Pointer to buffer */
                                   sizeof(double), /* Size of one scalar */
                                   py::format_descriptor<double>::format(), /* Python struct-style
                                                                               format descriptor */
                                   1, /* Number of dimensions */
                                   {v.size()}, /* Buffer dimensions */
                                   {sizeof(double)} /* Strides (in bytes) for each index */
            );
        });
}

template <unsigned size>
void createFixSizeVector(pybind11::module& module, const std::string& class_name)
{
    py::class_<VectorFixSize<size>>(module, class_name.c_str(), py::buffer_protocol())
        .def(py::init())
        .def("__getitem__",
             py::overload_cast<const std::size_t>(&VectorFixSize<size>::operator(), py::const_))
        .def("__setitem__",
             [](VectorFixSize<size>& the_vector, std::size_t index, double new_value) {
                 the_vector(index) = new_value;
             })
        .def("__len__", &VectorFixSize<size>::size)
        .def(
            "__iter__",
            [](const VectorFixSize<size>& s) { return py::make_iterator(s.begin(), s.end()); },
            py::keep_alive<0, 1>() /* Essential: keep object alive while iterator exists */)
        .def("set_zero", &VectorFixSize<size>::zero)
        .def("__repr__", &VectorFixSize<size>::toString)
        .def_buffer([](VectorFixSize<size>& v) -> py::buffer_info {
            return py::buffer_info(v.data(), /* Pointer to buffer */
                                   sizeof(double), /* Size of one scalar */
                                   py::format_descriptor<double>::format(), /* Python struct-style
                                                                               format descriptor */
                                   1, /* Number of dimensions */
                                   {v.size()}, /* Buffer dimensions */
                                   {sizeof(double)} /* Strides (in bytes) for each index */
            );
        });
}

void matrixDynSizeClassDefinition(py::class_<MatrixDynSize>& matrix)
{
    matrix.def(py::init())
        .def(py::init<unsigned, unsigned>())
        .def("__getitem__",
             [](MatrixDynSize& matrix, std::pair<std::size_t, std::size_t> indices) {
                 return matrix.getVal(indices.first, indices.second);
             })
        .def("__setitem__",
             [](MatrixDynSize& matrix, std::pair<std::size_t, std::size_t> indices, double item) {
                 return matrix.setVal(indices.first, indices.second, item);
             })
        .def("rows", &MatrixDynSize::rows)
        .def("cols", &MatrixDynSize::cols)
        .def("set_zero", &MatrixDynSize::zero)
        .def("resize", &MatrixDynSize::resize)
        .def("__repr__", &MatrixDynSize::toString)
        .def_buffer([](MatrixDynSize& m) -> py::buffer_info {
            return py::buffer_info(m.data(), /* Pointer to buffer */
                                   sizeof(double), /* Size of one scalar */
                                   py::format_descriptor<double>::format(), /* Python struct-style
                                                                               format descriptor */
                                   2, /* Number of dimensions */
                                   {m.rows(), m.cols()}, /* Buffer dimensions */
                                   {sizeof(double) * m.cols(), /* Strides (in bytes) for each index
                                                                */
                                    sizeof(double)});
        });
}

template <unsigned rows, unsigned cols>
void createFixSizeMatrix(pybind11::module& module, const std::string& class_name)
{
    py::class_<MatrixFixSize<rows, cols>>(module, class_name.c_str(), py::buffer_protocol())
        .def(py::init())
        .def("__getitem__",
             [](MatrixFixSize<rows, cols>& matrix, std::pair<std::size_t, std::size_t> indices) {
                 return matrix.getVal(indices.first, indices.second);
             })
        .def("__setitem__",
             [](MatrixFixSize<rows, cols>& matrix,
                std::pair<std::size_t, std::size_t> indices,
                double item) { return matrix.setVal(indices.first, indices.second, item); })
        .def("rows", &MatrixFixSize<rows, cols>::rows)
        .def("cols", &MatrixFixSize<rows, cols>::cols)
        .def("set_zero", &MatrixFixSize<rows, cols>::zero)
        .def("__repr__", &MatrixFixSize<rows, cols>::toString)
        .def_buffer([](MatrixFixSize<rows, cols>& m) -> py::buffer_info {
            return py::buffer_info(m.data(), /* Pointer to buffer */
                                   sizeof(double), /* Size of one scalar */
                                   py::format_descriptor<double>::format(), /* Python struct-style
                                                                               format descriptor */
                                   2, /* Number of dimensions */
                                   {m.rows(), m.cols()}, /* Buffer dimensions */
                                   {sizeof(double) * m.cols(), /* Strides (in bytes) for each index
                                                                */
                                    sizeof(double)});
        });
}

void transformClassDefinition(py::class_<Transform>& transform)
{
    transform.def(py::init())
        .def(py::init<const Rotation&, const Position&>())
        .def(py::init<const MatrixFixSize<4, 4>&>())
        .def_static("Identity", &Transform::Identity)
        .def_property_readonly("rotation", &Transform::getRotation)
        .def_property_readonly("position", &Transform::getPosition)
        .def("inverse", &Transform::inverse)
        .def(py::self * py::self)
        .def(
            "__mul__",
            [](const Transform& self, const Position& pos) -> Position { return self * pos; },
            py::is_operator())
        .def(
            "__mul__",
            [](const Transform& self, const SpatialInertia& inertia) -> SpatialInertia {
                return self * inertia;
            },
            py::is_operator());
}
} // namespace

void iDynTreeCoreBindings(pybind11::module& module)
{
    // Vectors and matrices.
    py::class_<VectorDynSize> vector_dyn(module, "VectorDynSize", py::buffer_protocol());
    vectorDynSizeClassDefinition(vector_dyn);
    createFixSizeVector<3>(module, "Vector3");
    createFixSizeVector<4>(module, "Vector4");
    createFixSizeVector<6>(module, "Vector6");

    py::class_<MatrixDynSize> matrix_dyn(module, "MatrixDynSize", py::buffer_protocol());
    matrixDynSizeClassDefinition(matrix_dyn);
    createFixSizeMatrix<3, 3>(module, "Matrix3x3");
    createFixSizeMatrix<4, 4>(module, "Matrix4x4");
    createFixSizeMatrix<6, 6>(module, "Matrix6x6");

    // Positions, Rotations and Transforms.
    py::class_<Position, VectorFixSize<3>>(module, "Position")
        .def(py::init())
        .def(py::init<double, double, double>())
        .def(py::self + py::self)
        .def(py::self - py::self)
        .def(-py::self)
        .def("__repr__", &Position::toString)
        .def_static("Zero", &Position::Zero);

    py::class_<Rotation, MatrixFixSize<3, 3>>(module, "Rotation")
        .def(py::init())
        .def(py::init<double,
                      double,
                      double, //
                      double,
                      double,
                      double, //
                      double,
                      double,
                      double>())
        .def("inverse", &Rotation::inverse)
        .def(py::self * py::self)
        .def(
            "__mul__",
            [](const Rotation& r, const Position& p) -> Position { return r * p; },
            py::is_operator())
        .def("as_rpy", &Rotation::asRPY)
        .def("as_quaternion", &Rotation::asQuaternion)
        .def("__repr__", &Rotation::toString)
        .def_static("Identity", &Rotation::Identity)
        .def_static("RotX", &Rotation::RotX)
        .def_static("RotY", &Rotation::RotY)
        .def_static("RotZ", &Rotation::RotZ)
        .def_static("RPY", &Rotation::RPY)
        .def_static("Quaternion", &Rotation::RotationFromQuaternion);

    py::class_<Transform> transform(module, "Transform");
    transformClassDefinition(transform);

    // Other classes.
    py::class_<Direction, VectorFixSize<3>>(module, "Direction")
        .def(py::init<double, double, double>());

    py::class_<Axis>(module, "Axis")
        .def(py::init<const Direction&, const Position&>())
        .def_property("direction", &Axis::getDirection, &Axis::setDirection)
        .def_property("origin", &Axis::getOrigin, &Axis::setOrigin)
        .def("__repr__", &Axis::toString);

    py::class_<RotationalInertia, MatrixFixSize<3, 3>>(module, "RotationalInertia").def(py::init());

    py::class_<SpatialInertia>(module, "SpatialInertia")
        .def(py::init())
        .def(py::init<double, const Position&, const RotationalInertia&>())
        .def("from_rotational_inertia_wrt_center_of_mass",
             &SpatialInertia::fromRotationalInertiaWrtCenterOfMass)
        .def("get_mass", &SpatialInertia::getMass)
        .def("get_center_of_mass", &SpatialInertia::getCenterOfMass)
        .def("get_rotational_inertia_wrt_frame_origin",
             &SpatialInertia::getRotationalInertiaWrtFrameOrigin)
        .def("get_rotational_inertia_wrt_center_of_mass",
             &SpatialInertia::getRotationalInertiaWrtCenterOfMass)
        .def_static("Zero", &SpatialInertia::Zero)
        .def("as_matrix", &SpatialInertia::asMatrix)
        .def(py::self + py::self)
        .def("__repr__",
             [](const SpatialInertia& inertia) { return inertia.asMatrix().toString(); });

    // Basic twist interface.
    py::class_<Twist>(module, "Twist")
        .def(py::init([](const std::array<double, 3>& lin_velocity,
                         const std::array<double, 3>& ang_velocity) {
            LinVelocity lin_vel(lin_velocity.data(), 3);
            AngVelocity ang_vel(ang_velocity.data(), 3);
            return Twist(lin_vel, ang_vel);
        }));

    // URI utility functions
    module.def("resolve_uri",
               &resolveURI,
               py::arg("uri"),
               py::arg("package_dirs") = std::vector<std::string>(),
               "Resolve a URI (such as package:// or model://) to an absolute path on the local "
               "file system.");
}
} // namespace bindings
} // namespace iDynTree
