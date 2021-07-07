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

#ifndef PCL_NO_PRECOMPILE
#include <rofl/pcl/point_cloud_plane_aligner.h>
#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>

// Instantiations of specific point types
PCL_INSTANTIATE(PointCloudPlaneAligner, PCL_XYZ_POINT_TYPES)

//namespace rofl {
//    template class PointCloudPlaneAligner<pcl::PointXYZ>;
//    template class PointCloudPlaneAligner<pcl::PointXYZI>;
//}
        
#endif    // PCL_NO_PRECOMPILE
