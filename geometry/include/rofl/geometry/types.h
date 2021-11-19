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
#ifndef ROFL_TYPES_H
#define ROFL_TYPES_H

#include <vector>
#include <deque>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/adapted/boost_polygon.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/linestring.hpp>
#include <boost/geometry/geometries/register/ring.hpp>
#include <boost/geometry/geometries/register/multi_polygon.hpp>

namespace rofl {

    using Scalar = float;

    // --------------------------------------------------------------
    // GEOMETRY 2D: ELEMENTARY TYPES
    // --------------------------------------------------------------
    
    using Point2 = Eigen::Matrix<Scalar, 2, 1>; //Eigen::Vector2f;
    using VectorPoint2 = std::vector<Point2, Eigen::aligned_allocator<Point2> >;
    using DequePoint2 = std::deque<Point2, Eigen::aligned_allocator<Point2> >;
    using Transform2 = Eigen::Transform<Scalar, 2, Eigen::Affine>; //Eigen::Affine2f;
    using VectorTransform2 = std::vector<Transform2, Eigen::aligned_allocator<Transform2> >;
    
    // --------------------------------------------------------------
    // GEOMETRY 2D: BOOST POLYGON TYPES
    // --------------------------------------------------------------
    
    using Segment2 = boost::geometry::model::segment<Point2>;
    using Polygon2 = boost::geometry::model::polygon<Point2,
            false, //outer ring of polygons is CCW, inner rings (holes) are CW
            false, //true for closed polygons; false for sparse points
            std::vector,
            std::vector,
            Eigen::aligned_allocator,
            Eigen::aligned_allocator>;
    using Box2 = boost::geometry::model::box<Point2>;
    using VectorSegment2 = std::vector<Segment2, Eigen::aligned_allocator<Segment2> >;
    using VectorPolygon2 = std::vector<Polygon2, Eigen::aligned_allocator<Polygon2> >;
    using VectorBox2 = std::vector<Box2, Eigen::aligned_allocator<Box2> >;
    

    // --------------------------------------------------------------
    // GEOMETRY 3D: ELEMENTARY TYPES
    // --------------------------------------------------------------
    
    using Point3 = Eigen::Matrix<Scalar, 2, 1>; //Eigen::Vector3f;
    using VectorPoint3 = std::vector<Point3, Eigen::aligned_allocator<Point3> >;
    using DequePoint3 = std::deque<Point3, Eigen::aligned_allocator<Point3> >;
    using Transform3 = Eigen::Transform<Scalar, 3, Eigen::Affine>; //Eigen::Affine3f;
    using VectorTransform3 = std::vector<Transform3, Eigen::aligned_allocator<Transform3> >;
    
    using VectorAffine3f = std::vector<Eigen::Affine3f, Eigen::aligned_allocator<Eigen::Affine3f> >;

}

BOOST_GEOMETRY_REGISTER_POINT_2D(rofl::Point2, rofl::Scalar, boost::geometry::cs::cartesian, x(), y());
BOOST_GEOMETRY_REGISTER_RING(rofl::VectorPoint2);
BOOST_GEOMETRY_REGISTER_MULTI_POLYGON(rofl::VectorPolygon2);

#endif 

