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
#ifndef ROFL_GEOMETRY_HOUGH_PLANE_DETECTOR_H_
#define ROFL_GEOMETRY_HOUGH_PLANE_DETECTOR_H_

#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <rofl/common/macros.h>
#include <rofl/geometry/types.h>
#include <rofl/common/grid.h>
#include <rofl/common/peak_finder_d.h>

namespace rofl {

	/**
	 * Class HoughPlaneDetector computes planes using Hough Transform (HT) in 3D and Hough Spectrum (HS).
	 * Planes are represented according the so called Hessian or polar parameters, i.e.
	 *
	 *   x * cos(theta) * cos(phi) + y * cos(theta) * sin(phi) + z * sin(theta) = rho
	 *
	 * where by defaults theta belongs to [-0.5*PI, 0.5*PI], phi to [0, 2.0*PI] and rho to [0.0, rhoMax].
	 * As in standard HT the values of parameters theta, phi and rho are discretized into histogram bins
	 * which in this case has 3 dimensions (one for each parameter) and is accessed by the tuple
	 * of integer indices [itheta, iphi, irho].
	 * The relations between indices and parameters (in center of corresponding histogram bin):
	 *   theta = thetaMin + thetaRes * itheta   with itheta = 0, ..., thetaNum-1
	 *   phi = phiMin + phiRes * iphi           with iphi = 0, ..., phiNum-1
	 *   rho = rhoMin + rhoRes * irho           with irho = 0, ..., rhoNum-1
	 * with default thetaMin = -0.5*PI, phiMin = 0.0, rhoMin = 0.0 and
	 * thetaRes = PI/thetaNum, phiRes = 2*PI / phiNum.
	 *
	 * HS is derived from HT and geometrically measures the fitting between the input points
	 * and the pencil of parallel planes.
	 * It is computed from HT as:
	 *
	 *   HS(itheta, iphi) = \sum_{irho} HT(itheta, iphi, irho)^2
	 *
	 * See the following references for more information:
	 * - A Censi, L Iocchi, G Grisetti, "Scan matching in the Hough domain", IEEE ICRA 2005
	 * - A. Censi, S. Carpin, "HSM3D: feature-less global 6DOF scan-matching in the Hough/Radon domain",
	 *   IEEE CRA 2009, DOI: 10.1109/ROBOT.2009.5152431.
	 */
	class HoughPlaneDetector {
	public:
		using Counter = size_t;
		using Index = int;
		using HoughTransformGrid = Grid<3, Counter, Index, rofl::detail::RasterIndexer<3, Index>, std::vector, std::allocator>;
		using HoughSpectrumGrid = Grid<2, Counter, Index, rofl::detail::RasterIndexer<2, Index>, std::vector, std::allocator>;
		using NormalLutGrid = Grid<2, Vector3, Index, rofl::detail::RasterIndexer<2, Index>, std::vector, Eigen::aligned_allocator>;
		using Indices3 = typename HoughTransformGrid::Indices;
		using Indices2 = typename HoughSpectrumGrid::Indices;
		using PlaneParam = Vector4;  //Eigen::Matrix<Scalar, 4, 1>;
		using VectorPlaneParam = std::vector<PlaneParam, Eigen::aligned_allocator<PlaneParam> >;

//		struct PlaneHypothesis {
//			int itheta;
//			int iphi;
//			int irho;
//		};

		/**
		 * Default constructor. It creates an empty class.
		 */
		HoughPlaneDetector();

		/**
		 * Constructor that creates the Hough transform and spectrum space with the given
		 * dimensions.
		 * @param thetaNum the number of partitioning bins of latitude theta
		 * @param phiNum the number of partitioning bins of longitude phi
		 * @param rhoNum the number of partitioning bins of distance from origin rho
		 * @param rhoRes the resolution of rho cells
		 */
		HoughPlaneDetector(int thetaNum, int phiNum, int rhoNum, Scalar rhoRes);

		/**
		 * Destructor.
		 */
		virtual ~HoughPlaneDetector();

		/**
		 * Initializes the Hough transform and spectrum space with the given dimensions.
		 * It uses the whole interval: thetaMin = -0.5*PI, phiMin = 0.0, rhoMin = 0.0.
		 * Moreover, the resolution of angular parameters theta and phi is
		 *    thetaRes = PI / thetaNum
		 *    phiRes = 2 * PI / phiNum
		 * @param thetaNum the number of partitioning bins of latitude theta
		 * @param phiNum the number of partitioning bins of longitude phi
		 * @param rhoNum the number of partitioning bins of distance from origin rho
		 * @param rhoRes the resolution of rho cells
		 */
		void init(int thetaNum, int phiNum, int rhoNum, Scalar rhoRes);

		/**
		 * Initializes the Hough transform and spectrum space with the given dimensions.
		 * It allows to set a different interval for Hough parameter ranges.
		 */
		void init(const Indices3& isize, const Vector3& paramMin, const Vector3& paramRes);

		/**
		 * Returns the number of partitioning bins of latitude theta.
		 */
		int getThetaNum() const;

		/**
		 * Returns the number of partitioning bins of longitude phi.
		 */
		int getPhiNum() const;

		/**
		 * the number of partitioning bins of distance from origin rho.
		 */
		int getRhoNum() const;

		/**
		 * Returns the min value of latitude theta.
		 */
		Scalar getThetaMin() const;

		/**
		 * Returns the min value of longitude phi.
		 */
		Scalar getPhiMin() const;

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
		Scalar getPhiRes() const;

		/**
		 * Returns the bin resolition of longitude phi.
		 */
		Scalar getRhoRes() const;

		/**
		 * Returns the value of Hough transform in the given bin.
		 * @param itheta the index of latitude bin
		 * @param itheta the index of longitude bin
		 * @param irho the index of distance-to-origin bin
		 */
		Counter getHoughTransform(int itheta, int iphi, int irho) const;

		/**
		 * Returns the value of Hough transform in the given bin.
		 * @param indices the index array where
		 *   indices[0] is the latitude bin index itheta,
		 *   indices[1] is the longitude bin index iphi,
		 *   indices[2] is the distance-to-origin bin index irho
		 */
		Counter getHoughTransform(const Indices3& indices) const;

		/**
		 * Returns the value of Hough spectrum in the given bin.
		 * @param itheta the index of latitude bin
		 * @param itheta the index of longitude bin
		 */
		Counter getHoughSpectrum(int itheta, int iphi) const;

		/**
		 * Returns the value of Hough spectrum in the given bin.
		 * @param indices the index array where
		 *   indices[0] is the latitude bin index itheta,
		 *   indices[1] is the longitude bin index iphi
		 */
		Counter getHoughSpectrum(const Indices2& indices) const;

		/**
		 * Resets the value of Hough transform and spectturm to zero.
		 */
		void setZero();

		/**
		 * Sets the dimension of non maximum suppression (NMS) window when searching
		 * maxima of Hough spectrum and transform.
		 * @param ithetaWin the widow dimension of latitude theta
		 * @param iphiWin the widow dimension of longitude phi
		 * @param irhoWin the widow dimension of distance-to-origin rho
		 */
		void setPeakWindow(int ithetaWin, int iphiWin, int irhoWin);

		/**
		 * Sets the dimension of non maximum suppression (NMS) window when searching
		 * maxima of Hough spectrum and transform.
		 * @param thetaWin the widow dimension of latitude theta
		 * @param phiWin the widow dimension of longitude phi
		 * @param rhoWin the widow dimension of distance-to-origin rho
		 */
		void setPeakWindow(Scalar thetaWin, Scalar phiWin, Scalar rhoWin);

		/**
		 * Inserts the given points and computes the Hough tansform and spectrum.
		 * @param points vector of points
		 */
		void insert(const VectorVector3& points);

		/**
		 * Inserts the given points and computes the Hough tansform and spectrum.
		 * It receives input points using iterators and conversion functor
		 * from input point type to rofl::Vector3.
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
		template <typename Iterator,typename Converter>
		void insert(Iterator beg, Iterator end, const Converter& converter);

		/**
		 * Returns the maxima of Hough spectrum with non-maximum suppression
		 * over window with half-size [ithetaWin, iphiWin].
		 * @param hsMaxima the indices of maxima in Hough spectrum
		 *    (hsMaxima[i][0]: itheta, hsMaxima[i][1]: iphi)
		 */
		void findSpectrumMax(std::vector<Indices2>& hsMaxima) const;

		/**
		 * Returns the maxima of Hough spectrum with non-maximum suppression
		 * over window with half-size [ithetaWin, iphiWin].
		 * @param hsMaxima the indices of maxima in Hough spectrum
		 *    (hsMaxima[i][0]: itheta, hsMaxima[i][1]: iphi)
		 */
		void findNormalMax(VectorVector3& normals) const;

		/**
		 * Returns the detected planes using Hough spectrum and transform
		 * with non-maximum suppression.
		 * @param hypotheses the indices of maxima in Hough domains
		 *    (hypotheses[i][0]: itheta, hypotheses[i][1]: iphi, hypotheses[i][2]: irho)
		 */
		void findPlanes(std::vector<Indices3>& hypotheses) const;

		/**
		 * Returns the detected planes using Hough spectrum and transform
		 * with non-maximum suppression
		 * @param planeParams the plane params of Cartesian equation
		 *    x * planeParams[i](0) + y * planeParams[i](1) + z * planeParams[i](2) + planeParams[i](3) = 0
		 */
		void findPlanes(VectorPlaneParam& planeParams);

	private:
		HoughTransformGrid houghTransform_;
		HoughSpectrumGrid houghSpectrum_;
		NormalLutGrid normalLut_;
		int thetaNum_;
		int phiNum_;
		int rhoNum_;
		Scalar thetaMin_;
		Scalar phiMin_;
		Scalar rhoMin_;
		Scalar thetaRes_;
		Scalar phiRes_;
		Scalar rhoRes_;
		int ithetaWin_;
		int iphiWin_;
		int irhoWin_;

//		inline int indexTransform(int itheta, int iphi, int irho) const {
//			return (((itheta * phiNum_) + iphi) * rhoNum_ + irho);
//		}
//
//		inline int indexSpectrum(int itheta, int iphi) const {
//			return ((itheta * phiNum_) + iphi);
//		}
//
//		inline PlaneParam getPlaneParams(int itheta, int iphi, int irho) const {
//			const Vector3 &normal = normalLut_.value( { itheta, iphi });
//			return PlaneParam(normal(0), normal(1), normal(2), -(rhoRes_ * irho));
//		}
//
//		inline PlaneParam getPlaneParams(const PlaneHypothesis& ph) const {
//			return getPlaneParams(ph.itheta, ph.iphi, ph.irho);
//		}
	};

	// ------------------------------------------------------------------------
	// DEFINITION OF METHODS WITH TEMPLATE ARGUMENTS
	// ------------------------------------------------------------------------

	template <typename Iterator,typename Converter>
	void HoughPlaneDetector::insert(Iterator beg, Iterator end, const Converter& converter) {
		Scalar rho;
		int irho;
		int pointNum = std::distance(beg, end);
		int pointNotify = pointNum / 25;

		setZero();

		// Computes the Hough Transform (HT) by incrementing counting cells
		int pointCounter = 0;
		for (auto it = beg; it != end; ++it) {
			//if (pointCounter % pointNotify == 0) {
			//	std::cout << "HT: processed " << pointCounter << "/" << pointNum << " points (print every " << pointNotify << ")" << std::endl;
			//}
			const Vector3& p = converter(*it);
			for (int itheta = 0; itheta < thetaNum_; ++itheta) {
				for (int iphi = 0; iphi < phiNum_; ++iphi) {
					const Vector3 &normal = normalLut_.value({itheta, iphi});
					rho = normal(0) * p(0) + normal(1) * p(1) + normal(2) * p(2);
					irho = (int) round(rho / rhoRes_);
					if (0 <= irho && irho < rhoNum_) {
						houghTransform_.value({itheta, iphi, irho})++;
						//std::cout << "point [" << p.transpose()  << "]: itheta " << itheta << " iphi " << iphi
						//  << " rho " << rho << " irho " << irho << " HT " << houghTransform_[indexTransform(itheta, iphi, irho)] << "\n";
					}
				}
			}
			pointCounter++;
		}

		// Computes Hough Spectrum (HS)
		for (int itheta = 0; itheta < thetaNum_; ++itheta) {
			for (int iphi = 0; iphi < phiNum_; ++iphi) {
				Counter& hsVal = houghSpectrum_.value({itheta, irho });
				hsVal = 0;
				for (int irho = 0; irho < rhoNum_; ++irho) {
					Counter &htVal = houghTransform_.value( { itheta, iphi, irho });
					hsVal += htVal * htVal;
				}
				//std::cout << "itheta " << itheta << " irho " << irho << " hsIdx " << htIdx << ": HS " << houghSpectrum_[htIdx] << std::endl;
			}
		}
	}

}// end of namespace

#endif
