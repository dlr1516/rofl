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
        Indices3 isize = {thetaNum, phiNum, rhoNum};
        Vector3 paramMin, paramRes;

        //paramMin << -0.5 * M_PI, 0.0, 0.0;
        //paramRes << M_PI / thetaNum, 2.0 * M_PI / phiNum, rhoRes;
        paramMin << 0.0, 0.0, -(rhoNum / 2) * rhoRes;
        paramRes << 0.5 * M_PI / thetaNum, 2.0 * M_PI / phiNum, rhoRes;
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
        houghSpectrum_.initBounds({thetaNum_, phiNum_});
        normalLut_.initBounds({thetaNum_, phiNum_});

        houghTransform_.fill(0);
        houghSpectrum_.fill(0);

        for (int itheta = 0; itheta < thetaNum_; ++itheta) {
            theta = thetaRes_ * itheta + thetaMin_;
            ct = cos(theta);
            st = sin(theta);
            for (int iphi = 0; iphi < phiNum_; ++iphi) {
                phi = phiRes_ * iphi + phiMin_;
                normalLut_.value({itheta, iphi}) << ct * cos(phi), ct * sin(phi), st;
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
        return houghTransform_.value({itheta, iphi, irho});
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
        return houghSpectrum_.value({itheta, iphi});
    }

    HoughPlaneDetector::Counter HoughPlaneDetector::getHoughSpectrum(const Indices2& indices) const {
        ROFL_ASSERT_VAR2(0 <= indices[0] && indices[0] < thetaNum_, indices[0], thetaNum_);
        ROFL_ASSERT_VAR2(0 <= indices[1] && indices[1] < phiNum_, indices[1], phiNum_);
        return houghSpectrum_.value(indices);
    }

    const Vector3& HoughPlaneDetector::getNormal(const Indices2& indices) const {
        ROFL_ASSERT_VAR2(0 <= indices[0] && indices[0] < thetaNum_, indices[0], thetaNum_);
        ROFL_ASSERT_VAR2(0 <= indices[1] && indices[1] < phiNum_, indices[1], phiNum_);
        return normalLut_.value(indices);
    }

    const Vector3& HoughPlaneDetector::getNormal(int itheta, int iphi) const {
        return getNormal({itheta, iphi});
    }

    HoughPlaneDetector::PlaneParam HoughPlaneDetector::getPlaneParam(const Indices3& indices) const {
        ROFL_ASSERT_VAR2(0 <= indices[0] && indices[0] < thetaNum_, indices[0], thetaNum_);
        ROFL_ASSERT_VAR2(0 <= indices[1] && indices[1] < phiNum_, indices[1], phiNum_);
        ROFL_ASSERT_VAR2(0 <= indices[2] && indices[2] < rhoNum_, indices[2], rhoNum_);
        PlaneParam planeParam;
        const Vector3 normal = getNormal(indices[0], indices[1]);
        planeParam << normal(0), normal(1), normal(2), -(rhoRes_ * indices[2] + rhoMin_);
        return planeParam;
    }

    HoughPlaneDetector::PlaneParam HoughPlaneDetector::getPlaneParam(int itheta, int iphi, int irho) const {
        ROFL_ASSERT_VAR2(0 <= itheta && itheta < thetaNum_, itheta, thetaNum_);
        ROFL_ASSERT_VAR2(0 <= iphi && iphi < phiNum_, iphi, phiNum_);
        ROFL_ASSERT_VAR2(0 <= irho && irho < rhoNum_, irho, rhoNum_);
        PlaneParam planeParam;
        const Vector3 normal = getNormal(itheta, iphi);
        planeParam << normal(0), normal(1), normal(2), -(rhoRes_ * irho);
        return planeParam;
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
        ithetaWin_ = (int) ceil(thetaWin / thetaRes_);
        iphiWin_ = (int) ceil(phiWin / phiRes_);
        irhoWin_ = (int) ceil(rhoWin / rhoRes_);
    }

    void HoughPlaneDetector::insert(const VectorVector3& points) {
        int indicesAngle[3];
        rofl::Scalar rho;
        int phiNotify = phiNum_ / 8;

        // Computes the Hough Transform (HT) by incrementing counting cells
        for (indicesAngle[0] = 0; indicesAngle[0] < thetaNum_; ++indicesAngle[0]) {
            for (indicesAngle[1] = 0; indicesAngle[1] < phiNum_; ++indicesAngle[1]) {
                if (indicesAngle[1] % phiNotify == 0) {
                    ROFL_VAR2(indicesAngle[0], indicesAngle[1]);
                }
                ROFL_COMMON_PROFILER_SCOPED_TIMER_HERE
                        //houghTransform_.value( { itheta, iphi, 0 })++;
                        const Vector3 &normal = normalLut_.value(indicesAngle);
                for (auto &p : points) {
                    rho = normal(0) * p(0) + normal(1) * p(1) + normal(2) * p(2);
                    //indicesAngle[2] = (int) round((rho - rhoMin_) / rhoRes_);
                    indicesAngle[2] = (int) ((rho - rhoMin_) / rhoRes_);
                    if (0 <= indicesAngle[2] && indicesAngle[2] < rhoNum_) {
                        //houghTransform_.value( { itheta, iphi, irho })++;
                        houghTransform_.value(indicesAngle)++;
                    }
                }
            }
        }

        // Computes Hough Spectrum (HS)
        for (indicesAngle[0] = 0; indicesAngle[0] < thetaNum_; ++indicesAngle[0]) {
            for (indicesAngle[1] = 0; indicesAngle[1] < phiNum_; ++indicesAngle[1]) {
                Counter &hsVal = houghSpectrum_.value(indicesAngle);
                hsVal = 0;
                for (indicesAngle[2] = 0; indicesAngle[2] < rhoNum_; ++indicesAngle[2]) {
                    Counter &htVal = houghTransform_.value(indicesAngle);
                    hsVal += htVal * htVal;
                }
                //std::cout << "itheta " << itheta << " irho " << irho << " hsIdx " << htIdx << ": HS " << houghSpectrum_[htIdx] << std::endl;
            }
        }
    }

    void HoughPlaneDetector::findSpectrumMax(std::vector<Indices2>& hsMaxima) const {
        using PeakFinder2 = rofl::PeakFinderD<2, Counter, int, std::greater<Counter> >;

        // Searches normal directions that are robust maxima in Hough Spectrum
        PeakFinder2 peakHS;
        auto hsMap = [&](const Indices2 & indices) -> Counter {
            return houghSpectrum_.value(indices);
        };
        peakHS.setDomain({thetaNum_, phiNum_});
        peakHS.setPeakWindow({ithetaWin_, iphiWin_});
        peakHS.detect(hsMap, std::back_inserter(hsMaxima));
    }

    void HoughPlaneDetector::findNormalMax(VectorVector3& normals) const {
        std::vector<Indices2> hsMaxima;
        findSpectrumMax(hsMaxima);
        normals.clear();
        normals.reserve(hsMaxima.size());
        for (auto& h : hsMaxima) {
            normals.push_back(normalLut_.value(h));
        }
    }

    void HoughPlaneDetector::findParallelPlanes(int itheta, int iphi, std::vector<Indices3>& indicesMaxima) const {
        using PeakFinder1 = rofl::PeakFinderD<1, Counter, int, std::greater<Counter> >;
        using Indices1 = typename PeakFinder1::Indices;
        std::vector<Indices1> rhoMaxima;
        Indices3 ph;

        PeakFinder1 peakRho;
        peakRho.setDomain({rhoNum_});
        peakRho.setPeakWindow({irhoWin_});
        auto htRho = [&](const Indices1 & indices) -> Counter {
            return houghTransform_.value({itheta, iphi, indices[0]});
        };
        peakRho.detect(htRho, std::back_inserter(rhoMaxima));
        ph[0] = itheta;
        ph[1] = iphi;
        indicesMaxima.clear();
        for (auto &r : rhoMaxima) {
            ph[2] = r[0];
            indicesMaxima.push_back(ph);
        }
    }

    void HoughPlaneDetector::findParallelPlanes(int itheta, int iphi, VectorPlaneParam& planeParams) const {
        std::vector<Indices3> indicesMaxima;

        findParallelPlanes(itheta, iphi, indicesMaxima);
        planeParams.resize(indicesMaxima.size());
        for (int i = 0; i < indicesMaxima.size(); ++i) {
            planeParams[i] = getPlaneParam(indicesMaxima[i]);
        }
    }

    void HoughPlaneDetector::findPlanes(std::vector<Indices3>& hypotheses) const {
        using PeakFinder1 = rofl::PeakFinderD<1, Counter, int, std::greater<Counter> >;
        using Indices1 = typename PeakFinder1::Indices;
        Indices3 ph;
        auto htCmp = [&](const Indices3& nh1, const Indices3 & nh2) -> bool {
            return houghTransform_.value(nh1) > houghTransform_.value(nh2);
        };

        // Searches normal directions that are robust maxima in Hough Spectrum
        std::vector<Indices2> hsMaxima;
        findSpectrumMax(hsMaxima);

        // For each normal direction (Hough spectrum maximum) searches the best rho
        PeakFinder1 peakRho;
        std::vector<Indices1> rhoMaxima;
        peakRho.setDomain({rhoNum_});
        peakRho.setPeakWindow({irhoWin_});
        for (auto &h : hsMaxima) {
            rhoMaxima.clear();
            auto htRho = [&](const Indices1 & indices) -> Counter {
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
            planeParams[i] = getPlaneParam(hypotheses[i]);
        }
    }

}

