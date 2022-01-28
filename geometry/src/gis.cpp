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
#include <rofl/geometry/gis.h>
#include <cmath>

namespace rofl {

    void convertLatLonAltToECEF(Scalar lat, Scalar lon, Scalar alt, Scalar& x, Scalar& y, Scalar& z) {
        Scalar latRad = lat / 180.0 * M_PI; // latitude in radians
        Scalar lonRad = lon / 180.0 * M_PI; // longituede in radians
        Scalar Nphi = ROFL_WGS84_SEMI_AXIS_A / sqrt(1.0 - ROFL_WGS84_ECCENTRICITY_2 * sin(latRad) * sin(latRad));
        x = (Nphi + alt) * cos(latRad) * cos(lonRad);
        y = (Nphi + alt) * cos(latRad) * sin(lonRad);
        z = (ROFL_WGS84_AXIS_RATIO * ROFL_WGS84_AXIS_RATIO * Nphi + alt) * sin(latRad);
    }

    Scalar meterPerDegLat(Scalar lat) {
        Scalar latRad, dist;
        latRad = lat / 180.0 * M_PI;
        dist = 111132.09 - 566.05 * cos(2.0 * latRad) + 1.20 * cos(4.0 * latRad) - 0.002 * cos(6.0 * latRad);
        return dist;
    }

    Scalar meterPerDegLon(Scalar lon) {
        Scalar lonRad, dist;
        lonRad = lon / 180.0 * M_PI;
        dist = 111415.13 * cos(lonRad) - 94.55 * cos(3.0 * lonRad) + 0.12 * cos(5.0 * lonRad);
        return dist;
    }

    void convertLatLonToLocalXY(Scalar latOrg, Scalar lonOrg, Scalar lat, Scalar lon, Scalar& x, Scalar& y) {
        x = (lon - lonOrg) * meterPerDegLon(lonOrg);
        y = (lat - latOrg) * meterPerDegLat(latOrg);
    }

    void convertLocalXYToLatLon(Scalar latOrg, Scalar lonOrg, Scalar x, Scalar y, Scalar& lat, Scalar& lon) {
        lon = x / meterPerDegLon(lonOrg) + lonOrg;
        lat = y / meterPerDegLat(latOrg) + latOrg;
    }
}
//