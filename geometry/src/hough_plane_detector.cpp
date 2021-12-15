#include <rofl/geometry/hough_plane_detector.h>

namespace rofl {

	HoughPlaneDetector::HoughPlaneDetector() : houghTransform_(), houghSpectrum_(), thetaNum_(0), phiNum_(0), rhoNum_(0), rhoRes_(0.0) {
		thetaNum_ = 0;
		phiNum_ = 0;
		rhoNum_ = 0;
		thetaMin_ = 0.0;
		phiMin_ = 0.0;
		rhoMin_ = 0.0;
		thetaRes_ = 0.0;
		phiRes_ = 0.0;
		rhoRes_ = 0.0;
		ithetaWin_ = 1;
		iphiWin_ = 1;
		irhoWin_ = 1;
	}

	HoughPlaneDetector::HoughPlaneDetector(int thetaNum, int phiNum, int rhoNum, Scalar rhoRes) : houghTransform_(), houghSpectrum_(), thetaNum_(thetaNum), phiNum_(
			phiNum), rhoNum_(rhoNum), rhoRes_(rhoRes) {
		init(thetaNum, phiNum, rhoNum, rhoRes);
		ithetaWin_ = 1;
		iphiWin_ = 1;
		irhoWin_ = 1;
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
		return houghTransform_.value({ itheta, iphi, irho });
	}

	HoughPlaneDetector::Counter HoughPlaneDetector::getHoughTransform(const Indices3& indices) const {
		ROFL_ASSERT_VAR2(0 <= indices[0] && indices[0] < thetaNum_, indices[0], thetaNum_);
		ROFL_ASSERT_VAR2(0 <= indices[1] && indices[1] < phiNum_, indices[1], phiNum_);
		ROFL_ASSERT_VAR2(0 <= indices[2] && indices[2] < rhoNum_, indices[2], rhoNum_);
		return houghTransform_.value(indices);
	}

	HoughPlaneDetector::Counter HoughPlaneDetector::getHoughSpectrum(int itheta, int iphi) const {
		ROFL_ASSERT_VAR2(0 <= itheta && itheta < thetaNum_, itheta, thetaNum_);
		ROFL_ASSERT_VAR2(0 <= iphi && iphi < phiNum_, iphi, phiNum_);
		return houghSpectrum_.value( { itheta, iphi });
	}

	HoughPlaneDetector::Counter HoughPlaneDetector::getHoughSpectrum(const Indices2& indices) const {
		ROFL_ASSERT_VAR2(0 <= indices[0] && indices[0] < thetaNum_, indices[0], thetaNum_);
		ROFL_ASSERT_VAR2(0 <= indices[1] && indices[1] < phiNum_, indices[1], phiNum_);
		return houghSpectrum_.value(indices);
	}

	void HoughPlaneDetector::setZero() {
		houghTransform_.fill(0);
		houghSpectrum_.fill(0);
	}

	void HoughPlaneDetector::setPeakWindow(int ithetaWin, int iphiWin, int irhoWin) {
		ithetaWin_ = ithetaWin;
		iphiWin_ = iphiWin;
		irhoWin_ = irhoWin;
	}

	void HoughPlaneDetector::setPeakWindow(Scalar thetaWin, Scalar phiWin, Scalar rhoWin) {
		ithetaWin_ = (int)ceil(thetaWin / thetaRes_);
		iphiWin_ = (int)ceil(phiWin / phiRes_);
		irhoWin_ = (int)ceil(rhoWin / rhoRes_);
	}

	void HoughPlaneDetector::insert(const VectorVector3& points) {
		insert(std::begin(points), std::end(points), [&](const Vector3& v) -> const Vector3& { return v; });
		// Also possibile to return the copy:
		//   insert(std::begin(points), std::end(points), [&](const Vector3& v) -> Vector3 { return v; });
	}

	void HoughPlaneDetector::findSpectrumMax(std::vector<Indices2>& hsMaxima) const {
		using PeakFinder2 = rofl::PeakFinderD<2, Counter, int, std::greater<Counter> >;

		// Searches normal directions that are robust maxima in Hough Spectrum
		PeakFinder2 peakHS;
		auto hsMap = [&](const Indices2& indices) -> Counter {
			return houghSpectrum_.value(indices);
		};
		peakHS.setDomain( { thetaNum_, phiNum_ });
		peakHS.setPeakWindow( { ithetaWin_, iphiWin_ });
		peakHS.detect(hsMap, std::back_inserter(hsMaxima));
	}

	void HoughPlaneDetector::findPlanes(std::vector<Indices3>& hypotheses) const {
		using PeakFinder1 = rofl::PeakFinderD<1, Counter, int, std::greater<Counter> >;
		using Indices1 = typename PeakFinder1::Indices;
		Indices3 ph;
		auto htCmp = [&](const Indices3& nh1, const Indices3& nh2) -> bool {
			return houghTransform_.value(nh1) > houghTransform_.value(nh2);
		};

		// Searches normal directions that are robust maxima in Hough Spectrum
		std::vector<Indices2> hsMaxima;
		findSpectrumMax(hsMaxima);

		// For each normal direction (Hough spectrum maximum) searches the best rho
		PeakFinder1 peakRho;
		std::vector<Indices1> rhoMaxima;
		peakRho.setDomain( { rhoNum_ });
		peakRho.setPeakWindow( { irhoWin_ });
		for (auto &h : hsMaxima) {
			rhoMaxima.clear();
			auto htRho = [&](const Indices1& indices) -> Counter {
				return houghTransform_.value({h[0], h[1], indices[0]});
			};
			peakRho.detect(htRho, std::back_inserter(rhoMaxima));
			ph[0] = h[0];
			ph[1] = h[1];
			for (auto &r : rhoMaxima) {
				ph[2] = r[0];
				hypotheses.push_back(ph);
			}
		}

		std::sort(hypotheses.begin(), hypotheses.end(), htCmp);
	}

	void HoughPlaneDetector::findPlanes(VectorPlaneParam& planeParams) {
		std::vector<Indices3> hypotheses;

		findPlanes(hypotheses);
		planeParams.resize(hypotheses.size());
		for (int i = 0; i < hypotheses.size(); ++i) {
			Vector3 &normal = normalLut_.value( { hypotheses[i][0], hypotheses[i][1] });
			planeParams[i] << normal(0), normal(1), normal(2), -(rhoRes_ * hypotheses[i][2]);
		}
	}

}

