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
#ifndef ROFL_GEOMETRY_GIS_H_
#define ROFL_GEOMETRY_GIS_H_

#include <rofl/geometry/types.h>

namespace rofl {

	/**
	 * Constants are taken from document NIMA TR8350.2 about WGS84
	 * (World Geodetic System 1984).
	 * Distances are in meters [m].
	 */

	const Scalar ROFL_WGS84_SEMI_AXIS_A = 6378137.0;

	const Scalar ROFL_WGS84_SEMI_AXIS_B = 6356752.3142;

	const Scalar ROFL_WGS84_AXIS_RATIO = 0.996647189335;        // ROFL_WGS84_SEMI_AXIS_B / ROFL_WGS84_SEMI_AXIS_A

	const Scalar ROFL_WGS84_ECCENTRICITY = 0.082094437949696;   // sqrt(1.0 - ROFL_WGS84_AXIS_RATIO^2)

	const Scalar ROFL_WGS84_ECCENTRICITY_2 = 0.00669437999014;  // 1.0 - ROFL_WGS84_AXIS_RATIO^2


	/**
	 * Converts the latitude, longitude and altitude into ECEF (Earth-centered, Earth-fixed)
	 * coordinate system.
	 * The conversion procedure assumes that the Earth is modeled with a ellipse section as in
	 * https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_geodetic_to_ECEF_coordinates
	 *
	 * Note: more complex representation is suggested by NIMA TR8350.2
	 *
	 * @param lat latitude in decimal degrees
	 * @param lon longitude in decimal degrees
	 * @param alt altitude in [m]
	 * @param x coordinate x in ECEF
	 * @param y coordinate y in ECEF
	 * @param z coordinate z in ECEF
	 */
	void convertLatLonAltToECEF(Scalar lat, Scalar lon, Scalar alt, Scalar& x, Scalar& y, Scalar& z);

	/**
	 * Returns the meters-per-degree of corresponding to longitude at zero altitude.
	 * The implementation follows mdeglon() from the Python library AlvinXY,
	 * which uses spherical harmonics representation.
	 * @param lat latitude in decimal degrees
	 * @return the corresponding distance in meters [m]
	 */
	Scalar meterPerDegLat(Scalar lat);

	/**
	 * Returns the meters-per-degree of corresponding to latitude at zero altitude.
	 * The implementation follows mdeglat() from the Python library AlvinXY,
	 * which uses spherical harmonics representation.
	 * @param lon longitude in decimal degrees
	 * @return the corresponding distance in meters [m]
	 */
	Scalar meterPerDegLon(Scalar lon);

	/**
	 * Converts latitude and longitude coordinates into "flattened" local Cartesian X and Y coordinates
	 * w.r.t. an origin point specified by its latitute and longitude.
	 *
	 * The implementation follows the Python library AlvinXY.
	 *
	 * @param latOrg latitude of origin point in decimal degrees
	 * @param lonOrg longitude of origin in decimal degrees
	 * @param lat latitude of current point in decimal degrees
	 * @param lon longitude of current point in decimal degrees
	 * @param x the computed coordinate x of current point
	 * @param y the computed coordinate y of current point
	 */
	void convertLatLonToLocalXY(Scalar latOrg, Scalar lonOrg, Scalar lat, Scalar lon, Scalar& x, Scalar& y);

	void convertLocalXYToLatLon(Scalar latOrg, Scalar lonOrg, Scalar x, Scalar y, Scalar& lat, Scalar& lon);
}

#endif
