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
#include <rofl/common/macros.h>
#include <rofl/common/param_map.h>
#include <rofl/common/tls_scalar_consensus.h>

#include <boost/version.hpp>
#include <iostream>
#include <random>
#include <vector>

int main(int argc, char** argv) {
    rofl::ParamMap params;
    std::vector<float> valuesSrc, valuesDst, valuesDif;
    std::vector<float> ranges;
    std::string filenameCfg;
    float translTrue, range, translEst;
    int valuesNum;
    std::random_device randDev;
    std::default_random_engine randEng(randDev());

    params.read(argc, argv);
    params.getParam<std::string>("cfg", filenameCfg, "");
    params.read(filenameCfg);
    params.getParam<int>("valuesNum", valuesNum, 5);
    params.getParam<float>("translTrue", translTrue, -1.8);
    params.getParam<float>("range", range, 0.4);

    std::cout << "Params:\n";
    params.write(std::cout);
    std::cout << std::endl;

    ROFL_VAR1(BOOST_VERSION);

    std::cout << "Creates perfectly matching source and destination point clouds\n";
    for (int i = 0; i < valuesNum; i++) {
        valuesSrc.push_back(1.0 * i);
        valuesDst.push_back(valuesSrc.back() + translTrue);
    }
    ROFL_VAR2(valuesSrc.size(), valuesDst.size());

    std::cout << "Add random points (outliers) to point clouds\n";
    std::uniform_real_distribution<float> distr(-translTrue, valuesNum + translTrue);
    for (int i = 0; i < valuesNum; ++i) {
        valuesSrc.push_back(distr(randEng));
        valuesDst.push_back(distr(randEng));
    }
    ROFL_VAR2(valuesSrc.size(), valuesDst.size());

    std::cout << "Computes candidates translations" << std::endl;
    for (int i = 0; i < valuesSrc.size(); ++i) {
        for (int j = 0; j < valuesDst.size(); ++j) {
            valuesDif.push_back(valuesDst[j] - valuesSrc[i]);
            ranges.push_back(range);
        }
    }
    ROFL_VAR2(valuesDif.size(), ranges.size());

    std::cout << "Performs histogram computation\n";
    std::vector<bool> inliers;
    rofl::estimateTLSEstimation(valuesDif.begin(), valuesDif.end(),
                                ranges.begin(), ranges.end(), translEst, inliers);

    std::cout << "Estimated translation: " << translEst << std::endl;

    return 0;
}