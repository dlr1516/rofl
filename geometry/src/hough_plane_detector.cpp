#include <rofl/geometry/hough_plane_detector.h>

namespace rofl {

	HoughPlaneDetector::HoughPlaneDetector() : houghTransform_(), houghSpectrum_(), thetaNum_(0), phiNum_(0), rhoNum_(0), rhoRes_(0.0) {
	}

	HoughPlaneDetector::HoughPlaneDetector(int thetaNum, int phiNum, int rhoNum, Scalar rhoRes) : houghTransform_(), houghSpectrum_(), thetaNum_(thetaNum), phiNum_(
			phiNum), rhoNum_(rhoNum), rhoRes_(rhoRes) {
		init(thetaNum, phiNum, rhoNum, rhoRes);
	}

	HoughPlaneDetector::~HoughPlaneDetector() {
	}

	void HoughPlaneDetector::init(int thetaNum, int phiNum, int rhoNum, Scalar rhoRes) {
		Indices3 isize = { thetaNum, phiNum, rhoNum };
		Vector3 paramMin, paramRes;

		paramMin << -0.5 * M_PI, 0.0, 0.0;
		paramRes << M_PI / thetaNum, 2.0 * M_PI / phiNum, rhoRes;
		init(isize, paramMin, paramRes);
	}

	void HoughPlaneDetector::init(const Indices3& isize, const Vector3& paramMin, const Vector3& paramRes) {
		Scalar ct, st, theta, phi;

		thetaNum_ = isize[0];
		phiNum_ = isize[1];
		rhoNum_ = isize[2];
		thetaMin_ = paramMin(0);
		phiMin_ = paramMin(1);
		rhoMin_ = paramMin(2);
		thetaRes_ = paramRes(0);
		phiRes_ = paramRes(1);
		rhoRes_ = paramRes(2);
		houghTransform_.initBounds(isize);
		houghSpectrum_.initBounds( { thetaNum_, phiNum_ });
		normalLut_.initBounds( { thetaNum_, phiNum_ });

		houghTransform_.fill(0);
		houghSpectrum_.fill(0);

		for (int itheta = 0; itheta < thetaNum_; ++itheta) {
			theta = thetaRes_ * itheta + thetaMin_;
			ct = cos(theta);
			st = sin(theta);
			for (int iphi = 0; iphi < phiNum_; ++iphi) {
				phi = phiRes_ * iphi + phiMin_;
				normalLut_.value( { itheta, iphi }) << ct * cos(phi), ct * sin(phi), st;
			}
		}
	}

	int HoughPlaneDetector::getThetaNum() const {
		return thetaNum_;
	}

	int HoughPlaneDetector::getPhiNum() const {
		return phiNum_;
	}

	int HoughPlaneDetector::getRhoNum() const {
		return rhoNum_;
	}

	Scalar HoughPlaneDetector::getThetaMin() const {
		return thetaMin_;
	}

	Scalar HoughPlaneDetector::getPhiMin() const {
		return phiMin_;
	}

	Scalar HoughPlaneDetector::getRhoMin() const {
		return rhoMin_;
	}

	Scalar HoughPlaneDetector::getThetaRes() const {
		return thetaRes_;
	}

	Scalar HoughPlaneDetector::getPhiRes() const {
		return phiRes_;
	}

	Scalar HoughPlaneDetector::getRhoRes() const {
		return rhoRes_;
	}

	HoughPlaneDetector::Counter HoughPlaneDetector::getHoughTransform(int itheta, int iphi, int irho) const {
		ROFL_ASSERT_VAR2(0 <= itheta && itheta < thetaNum_, itheta, thetaNum_);
		ROFL_ASSERT_VAR2(0 <= iphi && iphi < phiNum_, iphi, phiNum_);
		ROFL_ASSERT_VAR2(0 <= irho && irho < rhoNum_, irho, rhoNum_);
		return houghTransform_.value( { itheta, iphi, irho });
	}

	HoughPlaneDetector::Counter HoughPlaneDetector::getHoughSpecturm(int itheta, int iphi) const {
		ROFL_ASSERT_VAR2(0 <= itheta && itheta < thetaNum_, itheta, thetaNum_);
		ROFL_ASSERT_VAR2(0 <= iphi && iphi < phiNum_, iphi, phiNum_);
		return houghSpectrum_.value( { itheta, iphi });
	}

	void HoughPlaneDetector::setZero() {
		houghTransform_.fill(0);
		houghSpectrum_.fill(0);
	}

}

