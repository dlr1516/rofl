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

	class HoughPlaneDetector {
	public:
		using Counter = size_t;
		using Index = int;
		using HoughTransformGrid = Grid<3, Counter, Index, rofl::detail::RasterIndexer<3, Index>, std::vector, std::allocator>;
		using HoughSpectrumGrid = Grid<2, Counter, Index, rofl::detail::RasterIndexer<2, Index>, std::vector, std::allocator>;
		using NormalLutGrid = Grid<2, Vector3, Index, rofl::detail::RasterIndexer<2, Index>, std::vector, Eigen::aligned_allocator>;
		//using NormalLutGrid = Grid<2, Vector3>;
		using Indices3 = typename HoughTransformGrid::Indices;
		using Indices2 = typename HoughSpectrumGrid::Indices;
		using PlaneParam = Vector4;  //Eigen::Matrix<Scalar, 4, 1>;
		using VectorPlaneParam = std::vector<PlaneParam, Eigen::aligned_allocator<PlaneParam> >;


		struct PlaneHypothesis {
			int itheta;
			int iphi;
			int irho;
		};

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
		 * Returns the value of Hough spectrum in the given bin.
		 * @param itheta the index of latitude bin
		 * @param itheta the index of longitude bin
		 */
		Counter getHoughSpecturm(int itheta, int iphi) const;

		/**
		 * Resets the value of Hough transform and spectturm to zero.
		 */
		void setZero();

//		void insert(const VectorPoint3& points);
//
		template <typename Iterator,typename Converter>
		void insert(Iterator beg, Iterator end, Converter& converter);
//
//		void findPlane(int& itheta, int& iphi, int& irho);
//
//		void findPlane(std::vector<PlaneHypothesis>& hypotheses) const;
//
//		void findPlane(PlaneParam& planeParam);
//
//		void findPlane(VectorPlaneParam& planeParams);

	private:
		HoughTransformGrid houghTransform_;
		HoughSpectrumGrid houghSpectrum_;
		NormalLutGrid normalLut_;
		//std::vector<Counter> houghTransform_;
		//std::vector<Counter> houghSpectrum_;
		//VectorPoint3 normalLut_;
		int thetaNum_;
		int phiNum_;
		int rhoNum_;
		Scalar thetaMin_;
		Scalar phiMin_;
		Scalar rhoMin_;
		Scalar thetaRes_;
		Scalar phiRes_;
		Scalar rhoRes_;

		inline int indexTransform(int itheta, int iphi, int irho) const {
			return (((itheta * phiNum_) + iphi) * rhoNum_ + irho);
		}

		inline int indexSpectrum(int itheta, int iphi) const {
			return ((itheta * phiNum_) + iphi);
		}

		inline PlaneParam getPlaneParams(int itheta, int iphi, int irho) const {
			const Vector3 &normal = normalLut_.value( { itheta, iphi });
			return PlaneParam(normal(0), normal(1), normal(2), -(rhoRes_ * irho));
		}

		inline PlaneParam getPlaneParams(const PlaneHypothesis& ph) const {
			return getPlaneParams(ph.itheta, ph.iphi, ph.irho);
		}
	};

	template <typename Iterator,typename Converter>
	void HoughPlaneDetector::insert(Iterator beg, Iterator end, Converter& converter) {
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
			Vector3 p = converter(*it);
			for (int itheta = 0; itheta < thetaNum_; ++itheta) {
				for (int iphi = 0; iphi < phiNum_; ++iphi) {
					Vector3 &normal = normalLut_.value({itheta, iphi});
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
