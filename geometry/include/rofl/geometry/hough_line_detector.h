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
#ifndef ROFL_GEOMETRY_HOUGH_LINE_DETECTOR_H_
#define ROFL_GEOMETRY_HOUGH_LINE_DETECTOR_H_

#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <rofl/common/macros.h>
#include <rofl/geometry/types.h>
#include <rofl/common/grid.h>
#include <rofl/common/peak_finder_d.h>

namespace rofl {

    class HoughLineDetector {
    public:
        using Counter = size_t;
        using Index = int;
        using HoughTransformGrid = Grid<2, Counter, Index, rofl::detail::RasterIndexer<2, Index>, std::vector, std::allocator>;
        using HoughSpectrumGrid = std::vector<Counter>;
        using NormalLutGrid = VectorVector2;
        using Indices2 = typename HoughTransformGrid::Indices;
        using PeakFinder1 = rofl::PeakFinderD<1, Counter, Index, std::greater<Counter> >;
        using Indices1 = typename PeakFinder1::Indices;
        using LineParam = Vector3; //Eigen::Matrix<Scalar, 4, 1>;
        using VectorLineParam = std::vector<LineParam, Eigen::aligned_allocator<LineParam> >;

        /**
         * Default constructor. It creates an empty class.
         */
        HoughLineDetector();

        /**
         * Constructor that creates the Hough transform and spectrum space with the given
         * dimensions.
         * @param thetaNum the number of partitioning bins of theta
         * @param rhoNum the number of partitioning bins of distance from origin rho
         * @param rhoRes the resolution of rho cells
         */
        HoughLineDetector(int thetaNum, int rhoNum, Scalar rhoRes);

        /**
         * Destructor.
         */
        virtual ~HoughLineDetector();

        /**
         * Initializes the Hough transform and spectrum space with the given dimensions.
         * It uses the whole interval: thetaMin = 0.0, rhoMin = 0.0.
         * Moreover, the resolution of angular parameter theta is
         *    thetaRes = PI / thetaNum
         * @param thetaNum the number of partitioning bins of latitude theta
         * @param rhoNum the number of partitioning bins of distance from origin rho
         * @param rhoRes the resolution of rho cells
         */
        void init(int thetaNum, int rhoNum, Scalar rhoRes);

        /**
         * Initializes the Hough transform and spectrum space with the given dimensions.
         * It allows to set a different interval for Hough parameter ranges.
         */
        void init(const Indices2& isize, const Vector2& paramMin, const Vector2& paramRes);

        /**
         * Returns the number of partitioning bins of latitude theta.
         */
        int getThetaNum() const;

        /**
         * the number of partitioning bins of distance from origin rho.
         */
        int getRhoNum() const;

        /**
         * Returns the min value of latitude theta.
         */
        Scalar getThetaMin() const;

        /**
         * Returns the min value of distance from origin rho.
         */
        Scalar getRhoMin() const;

        /**
         * Returns the bin resolition of latitude theta.
         */
        Scalar getThetaRes() const;

        /**
         * Returns the bin resolition of longitude phi.
         */
        Scalar getRhoRes() const;

        /**
         * Returns the value of Hough transform in the given bin.
         * @param itheta the index of latitude bin
         * @param irho the index of distance-to-origin bin
         */
        Counter getHoughTransform(int itheta, int irho) const;

        /**
         * Returns the value of Hough transform in the given bin.
         * @param indices the index array where
         *   indices[0] is the latitude bin index itheta,
         *   indices[1] is the distance-to-origin bin index irho
         */
        Counter getHoughTransform(const Indices2& indices) const;

        /**
         * Returns the value of Hough spectrum in the given bin.
         * @param itheta the index of angle bin
         */
        Counter getHoughSpectrum(int itheta) const;

        /**
         * Resets the value of Hough transform and spectturm to zero.
         */
        void setZero();

        /**
         * Sets the dimension of non maximum suppression (NMS) window when searching
         * maxima of Hough spectrum and transform.
         * @param ithetaWin the widow dimension of angle theta
         * @param irhoWin the widow dimension of distance-to-origin rho
         */
        void setPeakWindow(int ithetaWin, int irhoWin);

        /**
         * Sets the dimension of non maximum suppression (NMS) window when searching
         * maxima of Hough spectrum and transform.
         * @param thetaWin the widow dimension of angle theta
         * @param rhoWin the widow dimension of distance-to-origin rho
         */
        void setPeakWindow(Scalar thetaWin, Scalar rhoWin);

        /**
         * Inserts the given points and computes the Hough tansform and spectrum.
         * @param points vector of points
         */
        void insert(const VectorVector2& points);

        /**
         * Inserts the given points and computes the Hough tansform and spectrum.
         * It receives input points using iterators and conversion functor
         * from input point type to rofl::Vector2.
         *
         * Example: if the input point set is a PCL point cloud
         *
         *   pcl::PointCloud<pcl::PointXYZ> cloud;
         *   auto converter = [&](const pcl::PointXYZ& p) -> Vector3 {
         *     Vector3 v;
         *     v << p.x, p.y, p.z;
         *     return v;
         *   };
         *   insert(std::begin(cloud), std::end(cloud), converter);
         *
         * @param points vector of points
         */
        template <typename Iterator, typename Converter>
        void insert(Iterator beg, Iterator end, const Converter& converter);

        /**
         * Returns the maxima of Hough spectrum with non-maximum suppression
         * over window with half-size ithetaWin.
         * @param hsMaxima the indices of maxima in Hough spectrum
         */
        void findSpectrumIndicesMax(std::vector<Indices1>& indicesMaxima) const;

        /**
         * Returns the maxima of Hough spectrum with non-maximum suppression
         * over window with half-size ithetaWin.
         * @param hsMaxima the indices of maxima in Hough spectrum
         */
        void findSpectrumIndexMax(std::vector<Index>& indexMaxima) const;

        /**
         * Returns the maxima of Hough spectrum with non-maximum suppression
         * over window with half-size ithetaWin.
         * @param angleMaxima the angle theta of maxima in Hough spectrum
         */
        void findSpectrumAngleMax(std::vector<Scalar>& angleMaxima) const;

        /**
         * Returns the maxima of Hough spectrum with non-maximum suppression
         * over window with half-size [ithetaWin, iphiWin].
         * @param normals the normals of maxima lines
         */
        void findNormalMax(VectorVector2& normals) const;

        /**
         * Returns the detected planes using Hough spectrum and transform
         * with non-maximum suppression.
         * @param linesIndices the indices of maxima in Hough domains
         */
        void findLines(std::vector<Indices2>& linesIndices) const;

        /**
         * Returns the detected planes using Hough spectrum and transform
         * with non-maximum suppression
         * @param planeParams the plane params of Cartesian equation
         *    x * planeParams[i](0) + y * planeParams[i](1) + z * planeParams[i](2) + planeParams[i](3) = 0
         */
        void findLines(VectorLineParam& linesParams);

    private:
        HoughTransformGrid houghTransform_;
        HoughSpectrumGrid houghSpectrum_;
        NormalLutGrid normalLut_;
        int thetaNum_;
        int rhoNum_;
        Scalar thetaMin_;
        Scalar rhoMin_;
        Scalar thetaRes_;
        Scalar rhoRes_;
        int ithetaWin_;
        int irhoWin_;
    };

} // end of namespace rofl

#endif /* GEOMETRY_INCLUDE_ROFL_GEOMETRY_HOUGH_LINE_DETECTOR_H_ */
