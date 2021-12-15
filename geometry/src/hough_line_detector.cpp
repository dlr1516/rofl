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
#include <rofl/geometry/hough_line_detector.h>

namespace rofl {

	HoughLineDetector::HoughLineDetector() : houghTransform_(), houghSpectrum_(), thetaNum_(0), rhoNum_(0), rhoRes_(0.0) {
		thetaNum_ = 0;
		rhoNum_ = 0;
		thetaMin_ = 0.0;
		rhoMin_ = 0.0;
		thetaRes_ = 0.0;
		rhoRes_ = 0.0;
		ithetaWin_ = 1;
		irhoWin_ = 1;
	}

	HoughLineDetector::HoughLineDetector(int thetaNum, int rhoNum, Scalar rhoRes) : houghTransform_(), houghSpectrum_(), thetaNum_(thetaNum), rhoNum_(rhoNum), rhoRes_(
			rhoRes) {
		init(thetaNum, rhoNum, rhoRes);
		ithetaWin_ = 1;
		irhoWin_ = 1;
	}

	HoughLineDetector::~HoughLineDetector() {
	}

	void HoughLineDetector::init(int thetaNum, int rhoNum, Scalar rhoRes) {
		Indices2 isize = { thetaNum, rhoNum };
		Vector2 paramMin, paramRes;

		paramMin << 0.0, 0.0;
		paramRes << M_PI / thetaNum, rhoRes;
		init(isize, paramMin, paramRes);
	}

	void HoughLineDetector::init(const Indices2& isize, const Vector2& paramMin, const Vector2& paramRes) {
		Scalar ct, st, theta, phi;

		thetaNum_ = isize[0];
		rhoNum_ = isize[1];
		thetaMin_ = paramMin(0);
		rhoMin_ = paramMin(1);
		thetaRes_ = paramRes(0);
		rhoRes_ = paramRes(1);
		houghTransform_.initBounds(isize);
		houghSpectrum_.resize(thetaNum_);
		normalLut_.resize(thetaNum_);

		houghTransform_.fill(0);
		std::fill(houghSpectrum_.begin(), houghSpectrum_.end(), 0);
		for (int itheta = 0; itheta < thetaNum_; ++itheta) {
			theta = thetaRes_ * itheta + thetaMin_;
			normalLut_[itheta] << cos(theta), sin(theta);
		}
	}

	int HoughLineDetector::getThetaNum() const {
		return thetaNum_;
	}

	int HoughLineDetector::getRhoNum() const {
		return rhoNum_;
	}

	Scalar HoughLineDetector::getThetaMin() const {
		return thetaMin_;
	}

	Scalar HoughLineDetector::getRhoMin() const {
		return rhoMin_;
	}

	Scalar HoughLineDetector::getThetaRes() const {
		return thetaRes_;
	}

	Scalar HoughLineDetector::getRhoRes() const {
		return rhoRes_;
	}

	HoughLineDetector::Counter HoughLineDetector::getHoughTransform(int itheta, int irho) const {
		ROFL_ASSERT_VAR2(0 <= itheta && itheta < thetaNum_, itheta, thetaNum_);
		ROFL_ASSERT_VAR2(0 <= irho && irho < rhoNum_, irho, rhoNum_);
		Indices2 indices = { itheta, irho };
		return houghTransform_.value(indices);
	}

	HoughLineDetector::Counter HoughLineDetector::getHoughTransform(const Indices2& indices) const {
		ROFL_ASSERT_VAR2(0 <= indices[0] && indices[0] < thetaNum_, indices[0], thetaNum_);
		ROFL_ASSERT_VAR2(0 <= indices[1] && indices[1] < rhoNum_, indices[1], rhoNum_);
		return houghTransform_.value(indices);
	}

	HoughLineDetector::Counter HoughLineDetector::getHoughSpectrum(int itheta) const {
		ROFL_ASSERT_VAR2(0 <= itheta && itheta < thetaNum_, itheta, thetaNum_);
		return houghSpectrum_[itheta];
	}

	void HoughLineDetector::setZero() {
		houghTransform_.fill(0);
		std::fill(std::begin(houghSpectrum_), std::end(houghSpectrum_), 0);
	}

	void HoughLineDetector::setPeakWindow(int ithetaWin, int irhoWin) {
		ithetaWin_ = ithetaWin;
		irhoWin_ = irhoWin;
	}

	void HoughLineDetector::setPeakWindow(Scalar thetaWin, Scalar rhoWin) {
		ithetaWin_ = (int) ceil(thetaWin / thetaRes_);
		irhoWin_ = (int) ceil(rhoWin / rhoRes_);
	}

	void HoughLineDetector::insert(const VectorVector2& points) {

	}

	void HoughLineDetector::findSpectrumIndicesMax(std::vector<Indices1>& indicesMaxima) const {
		// For maximum of HS
		PeakFinder1 peakHS;
		auto hsMap = [&](const Indices1& indices) -> Counter {
			return houghSpectrum_[indices[0]];
		};
		peakHS.setDomain( { thetaNum_ });
		peakHS.setPeakWindow( { ithetaWin_ });
		peakHS.detect(hsMap, std::back_inserter(indicesMaxima));
	}

	void HoughLineDetector::findSpectrumIndexMax(std::vector<Index>& indexMaxima) const {
		std::vector<Indices1> indicesMaxima;
		findSpectrumIndicesMax(indicesMaxima);
		std::transform(std::begin(indicesMaxima), std::end(indicesMaxima), std::back_inserter(indexMaxima), [](const Indices1& indices) -> Index {
			return indices[0];
		});
	}

	void HoughLineDetector::findSpectrumAngleMax(std::vector<Scalar>& angleMaxima) const {
		std::vector<Indices1> indicesMaxima;
		findSpectrumIndicesMax(indicesMaxima);
		std::transform(std::begin(indicesMaxima), std::end(indicesMaxima), std::back_inserter(angleMaxima), [&](const Indices1& indices) -> Scalar {
			return (thetaMin_ + thetaRes_ * indices[0]);
		});
	}

	void HoughLineDetector::findNormalMax(VectorVector2& normals) const {
		std::vector<Indices1> indicesMaxima;
		findSpectrumIndicesMax(indicesMaxima);
		std::transform(std::begin(indicesMaxima), std::end(indicesMaxima), std::back_inserter(normals), [&](const Indices1& indices) -> Vector2 {
			Scalar theta = (thetaMin_ + thetaRes_ * indices[0]);
			return Vector2(cos(theta), sin(theta));
		});
	}

	void HoughLineDetector::findLines(std::vector<Indices2>& linesIndices) const {
		std::vector<Indices1> hsMaxima, rhoMaxima;
		Indices2 lp;

		findSpectrumIndicesMax(hsMaxima);

		// For each normal direction (Hough spectrum maximum) searches the best rho
		PeakFinder1 peakRho;
		peakRho.setDomain( { rhoNum_ });
		peakRho.setPeakWindow( { irhoWin_ });
		for (auto &h : hsMaxima) {
			rhoMaxima.clear();
			auto htRho = [&](const Indices1& indices) -> Counter {
				return houghTransform_.value( { h[0], indices[0] });
			};
			peakRho.detect(htRho, std::back_inserter(rhoMaxima));
			lp[0] = h[0];
			for (auto &r : rhoMaxima) {
				lp[1] = r[0];
				linesIndices.push_back(lp);
			}
		}
	}

} // end of namespace
