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
#include <rofl/geometry/base_geometry2.h>

namespace rofl {

    // --------------------------------------------------------------
    // TYPE CONVERSION
    // --------------------------------------------------------------

    void poseToIsometry2(Scalar x, Scalar y, Scalar theta, Transform2& transf) {
        using Rotation2DLocal = Eigen::Rotation2D<Scalar>;
        transf = Transform2::Identity();
        //transf.prerotate(Eigen::Rotation2Dd(theta));
        transf.prerotate(Rotation2DLocal(theta));
        transf.pretranslate(Vector2(x, y));
    }

    void poseToIsometry2(const Vector3& pose, Transform2& transf) {
        poseToIsometry2(pose(0), pose(1), pose(2), transf);
    }

    // --------------------------------------------------------------
    // BASIC PRIMITIVES
    // --------------------------------------------------------------

    Scalar normalizeAngle2Pi(Scalar angle) {
        return (angle - K_2PI * floor(angle / K_2PI));
    }

    bool insideAngleInterval(Scalar x, Scalar y, Vector2 pBeg, Vector2 pEnd) {
        Scalar scalarE = -pEnd(0) * y + pEnd(1) * x;
        Scalar scalarB = pBeg(0) * y - pBeg(1) * x;
        int reverseCone = 0;
        if (pBeg(0) * pEnd(1) - pEnd(0) * pBeg(1) < 0) {
            reverseCone = 1;
            scalarE = -scalarE;
            scalarB = -scalarB;
        }

        if (!reverseCone) {
            return ((scalarE > 0)&&(scalarB > 0));
        } else {//reverse cone
            return (!((scalarE > 0)&&(scalarB > 0)));
        }
        assert(0);
    }

    bool insideAngleInterval(Scalar angle, Scalar intervBeg, Scalar intervEnd) {
        return (normalizeAngle2Pi(angle - intervBeg) < normalizeAngle2Pi(intervEnd - intervBeg));
    }

    bool insideAngleInterval(Scalar angle, const AngleInterval& interv) {
        return insideAngleInterval(angle, interv.first, interv.second);
    }

    bool insideAngleIntervalEq(Scalar angle, Scalar intervBeg, Scalar intervEnd) {
        return (normalizeAngle2Pi(angle - intervBeg) <= normalizeAngle2Pi(intervEnd - intervBeg));
    }

    Scalar ccw(const Vector2& p1, const Vector2& p2, const Vector2& p3) {
        return ((p2(0) - p1(0)) * (p3(1) - p1(1)) - (p2(1) - p1(1)) * (p3(0) - p1(0)));
    }

} // end of namespace
