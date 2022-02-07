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
#ifndef ROFL_GEOMETRY_BASIC_GEOMETRY2_H_
#define ROFL_GEOMETRY_BASIC_GEOMETRY2_H_

#include <rofl/geometry/types.h>

namespace rofl {

    // --------------------------------------------------------------
    // TYPE CONVERSION
    // --------------------------------------------------------------

    /**
     * Converts pose components to 2D transform. 
     * @param x coordinate x of pose
     * @param y coordinate y of pose
     * @param theta orientation of pose
     * @param transf the computed transformation 
     */
    void poseToIsometry2(Scalar x, Scalar y, Scalar theta, Transform2& transf);

    /**
     * Converts a pose vector [x, y, theta] to 2D transform. 
     * @param pose the vector with components [x, y, theta]
     * @param transf the computed transformation 
     */
    void poseToIsometry2(const Vector3& pose, Transform2& transf);

    // --------------------------------------------------------------
    // BASIC PRIMITIVES
    // --------------------------------------------------------------

    /**
     * Returns the angle in the interval [0.0, 2*M_PI[
     * @param angle input angle in radians
     */
    Scalar normalizeAngle2Pi(Scalar angle);

    /**
     * Says if a given angle falls inside an angle interval with limits
     * [intervBeg, intervEnd] (where intervEnd is reached in CCW turn from 
     * intervEnd).
     * @param angle
     * @param intervBeg
     * @param intervEnd
     * @return 
     */
    bool insideAngleInterval(Scalar angle, Scalar intervBeg, Scalar intervEnd);

    /**
     * Says if a given angle falls inside an angle interval interv. 
     * @param angle
     * @param interv
     * @return 
     */
    bool insideAngleInterval(Scalar angle, const AngleInterval& interv);

    /**
     * Says if a given angle falls inside an angle interval with limits
     * [intervBeg, intervEnd] (where intervEnd is reached in CCW turn from 
     * intervEnd).
     * Version with equality!
     * @param angle
     * @param intervBeg
     * @param intervEnd
     * @return 
     */
    bool insideAngleIntervalEq(Scalar angle, Scalar intervBeg, Scalar intervEnd);

    /**
     * Returns the signed area of triangle p1-p2-p3, which is positive if the 
     * three points are CCW ordered.
     */
    Scalar ccw(const Vector2& p1, const Vector2& p2, const Vector2& p3);

} // end of namespace

#endif 

