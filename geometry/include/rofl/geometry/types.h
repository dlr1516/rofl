/**
 * ROFL - RIMLab Open Factotum Library
 * Copyright (C) 2021 Dario Lodi Rizzini, Ernesto Fontana
 *
 * ROFL is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ROFL is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with ROFL.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef ROFL_GEOMETRY_TYPES_H
#define ROFL_GEOMETRY_TYPES_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <deque>
#include <vector>

//#include <boost/geometry.hpp>
#include <boost/geometry/geometries/adapted/boost_polygon.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/register/linestring.hpp>
#include <boost/geometry/geometries/register/multi_polygon.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/ring.hpp>
#include <boost/geometry/geometries/segment.hpp>

namespace rofl {

using Scalar = float;

// From C++-20 substitute with: constexpr K_PI = std::numbers::pi_v<float>;
const Scalar K_PI = 3.141592653589793238463;
const Scalar K_2PI = 6.28318530718;

using AngleInterval = std::pair<Scalar, Scalar>;

/**
 * ROFL geometry types are taken from library Eigen.
 * STL containers and classes containing Eigen linear algebra types
 * require aligned allocator before C++17.
 * Thus, there are different definitions according to the version.
 *
 * From gcc 4.7.0 the values of __cplusplus are associated to a C++ version:
 * - pre-standard: __cplusplus == 1;
 * - C++98: __cplusplus == 199711L;
 * - C++11: __cplusplus == 201103L;
 * - C++14: __cplusplus == 201402L;
 * - C++17: __cplusplus == 201703L;
 *
 * TODO: Other compilers like MSVC may require additional specifications.
 * We will never be able to fix everything....
 */

#if __cplusplus < 201703L

static_assert(__cplusplus >= 201703L, "Versions before C++-17 are deprecated: add compiling option \"-DCMAKE_CXX_STANDARD=17 -DCMAKE_CXX_FLAGS='-std=c++17'\"");

// --------------------------------------------------------------
// LINEAR ALGEBRA AND GEOMETRY: ELEMENTARY TYPES
// --------------------------------------------------------------

using Vector2 = Eigen::Matrix<Scalar, 2, 1>;
using VectorVector2 = std::vector<Vector2, Eigen::aligned_allocator<Vector2> >;
using DequeVector2 = std::deque<Vector2, Eigen::aligned_allocator<Vector2> >;
using Transform2 = Eigen::Transform<Scalar, 2, Eigen::Affine>;
using VectorTransform2 = std::vector<Transform2, Eigen::aligned_allocator<Transform2> >;

using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
using VectorVector3 = std::vector<Vector3, Eigen::aligned_allocator<Vector3> >;
using DequeVector3 = std::deque<Vector3, Eigen::aligned_allocator<Vector3> >;
using Transform3 = Eigen::Transform<Scalar, 3, Eigen::Affine>;
using VectorTransform3 = std::vector<Transform3, Eigen::aligned_allocator<Transform3> >;

using Vector4 = Eigen::Matrix<Scalar, 4, 1>;
using Vector5 = Eigen::Matrix<Scalar, 5, 1>;
using Vector6 = Eigen::Matrix<Scalar, 5, 1>;
using Vector7 = Eigen::Matrix<Scalar, 5, 1>;

using Matrix2 = Eigen::Matrix<Scalar, 2, 2>;
using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
using Matrix4 = Eigen::Matrix<Scalar, 4, 4>;
using Matrix5 = Eigen::Matrix<Scalar, 5, 5>;
using Matrix6 = Eigen::Matrix<Scalar, 6, 6>;

// --------------------------------------------------------------
// GEOMETRY 2D: BOOST POLYGON TYPES
// --------------------------------------------------------------

using Point2 = Vector2;
using Segment2 = boost::geometry::model::segment<Point2>;
using Polygon2 = boost::geometry::model::polygon<Point2,
                                                 false,  // outer ring of polygons is CCW, inner rings (holes) are CW
                                                 false,  // true for closed polygons; false for sparse points
                                                 std::vector,
                                                 std::vector,
                                                 Eigen::aligned_allocator,
                                                 Eigen::aligned_allocator>;
using Box2 = boost::geometry::model::box<Point2>;
using VectorSegment2 = std::vector<Segment2, Eigen::aligned_allocator<Segment2> >;
using VectorPolygon2 = std::vector<Polygon2, Eigen::aligned_allocator<Polygon2> >;
using VectorBox2 = std::vector<Box2, Eigen::aligned_allocator<Box2> >;

#else

// --------------------------------------------------------------
// LINEAR ALGEBRA AND GEOMETRY: ELEMENTARY TYPES
// --------------------------------------------------------------

using Vector2 = Eigen::Matrix<Scalar, 2, 1>;
using VectorVector2 = std::vector<Vector2>;
using DequeVector2 = std::deque<Vector2>;
using Transform2 = Eigen::Transform<Scalar, 2, Eigen::Affine>;
using VectorTransform2 = std::vector<Transform2>;

using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
using VectorVector3 = std::vector<Vector3>;
using DequeVector3 = std::deque<Vector3>;
using Transform3 = Eigen::Transform<Scalar, 3, Eigen::Affine>;
using VectorTransform3 = std::vector<Transform3>;

using Vector4 = Eigen::Matrix<Scalar, 4, 1>;
using Vector5 = Eigen::Matrix<Scalar, 5, 1>;
using Vector6 = Eigen::Matrix<Scalar, 5, 1>;
using Vector7 = Eigen::Matrix<Scalar, 5, 1>;

using Matrix2 = Eigen::Matrix<Scalar, 2, 2>;
using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
using Matrix4 = Eigen::Matrix<Scalar, 4, 4>;
using Matrix5 = Eigen::Matrix<Scalar, 5, 5>;
using Matrix6 = Eigen::Matrix<Scalar, 6, 6>;

// --------------------------------------------------------------
// GEOMETRY 2D: BOOST POLYGON TYPES
// --------------------------------------------------------------

using Point2 = Vector2;
using Segment2 = boost::geometry::model::segment<Point2>;
using Polygon2 = boost::geometry::model::polygon<Point2,
                                                 false,  // outer ring of polygons is CCW, inner rings (holes) are CW
                                                 false,  // true for closed polygons; false for sparse points
                                                 std::vector,
                                                 std::vector,
                                                 Eigen::aligned_allocator,
                                                 Eigen::aligned_allocator>;
using Box2 = boost::geometry::model::box<Point2>;
using VectorSegment2 = std::vector<Segment2>;
using VectorPolygon2 = std::vector<Polygon2>;
using VectorBox2 = std::vector<Box2>;

#endif

}  // namespace rofl

BOOST_GEOMETRY_REGISTER_POINT_2D(rofl::Vector2, rofl::Scalar, boost::geometry::cs::cartesian, x(), y());
BOOST_GEOMETRY_REGISTER_RING(rofl::VectorVector2);
BOOST_GEOMETRY_REGISTER_MULTI_POLYGON(rofl::VectorPolygon2);

#endif
