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
    std::vector<double> valuesSrc, valuesDst, valuesDif;
    std::vector<double> ranges;
    std::string filenameCfg, filenamePlot;
    double translTrue, range, translEst;
    int valuesNum;
    std::random_device randDev;
    std::default_random_engine randEng(randDev());

    params.read(argc, argv);
    params.getParam<std::string>("cfg", filenameCfg, "");
    params.read(filenameCfg);
    params.getParam<int>("valuesNum", valuesNum, 6);
    params.getParam<double>("translTrue", translTrue, -1.8);
    params.getParam<double>("range", range, 0.4);
    params.getParam<std::string>("plot", filenamePlot, "diff.plot");

    std::cout << "Params:\n";
    params.write(std::cout);
    std::cout << std::endl;

    // ROFL_VAR1(BOOST_VERSION);

    std::cout << "Creates perfectly matching source and destination point clouds\n";
    for (int i = 0; i < valuesNum; i++) {
        valuesSrc.push_back(0.8 * i * i);
        valuesDst.push_back(valuesSrc.back() + translTrue);
    }
    ROFL_VAR2(valuesSrc.size(), valuesDst.size());

    std::cout << "Add random points (outliers) to point clouds\n";
    std::uniform_real_distribution<float> distr(-translTrue, valuesNum + translTrue);
    for (int i = 0; 10 * i < valuesNum; ++i) {
        valuesSrc.push_back(distr(randEng));
        valuesDst.push_back(distr(randEng));
    }
    // std::cout << "\n\nSRC" << std::endl;
    // for (auto& v : valuesSrc)
    //     std::cout << v << ", ";
    // std::cout << "\n\nDST" << std::endl;
    // for (auto& v : valuesDst)
    //     std::cout << v << ", ";
    double srcBad[] = {0, 0.8, 3.2, 7.2, 12.8, 20, 28.8, 39.2, 51.2, 64.8, 80, 96.8, 115.2, 135.2, 156.8, 180, 204.8, 231.2, 259.2, 288.8, 320, 352.8, 387.2, 423.2, 460.8, 500, 540.8, 583.2, 627.2, 672.8, 720, 768.8, 819.2, 871.2, 924.8, 980, 1036.8, 1095.2, 1155.2, 1216.8, 1280, 1344.8, 1411.2, 1479.2, 1548.8, 1620, 1692.8, 1767.2, 1843.2, 1920.8, 2000, 2080.8, 2163.2, 2247.2, 2332.8, 2420, 2508.8, 2599.2, 2691.2, 2784.8, 43.5258, 3.31088, 16.9031, 3.07761, 22.6036, 5.31447};
    double dstBad[] = {-1.8, -1, 1.4, 5.4, 11, 18.2, 27, 37.4, 49.4, 63, 78.2, 95, 113.4, 133.4, 155, 178.2, 203, 229.4, 257.4, 287, 318.2, 351, 385.4, 421.4, 459, 498.2, 539, 581.4, 625.4, 671, 718.2, 767, 817.4, 869.4, 923, 978.2, 1035, 1093.4, 1153.4, 1215, 1278.2, 1343, 1409.4, 1477.4, 1547, 1618.2, 1691, 1765.4, 1841.4, 1919, 1998.2, 2079, 2161.4, 2245.4, 2331, 2418.2, 2507, 2597.4, 2689.4, 2783, 10.1156, 15.1098, 39.4784, 42.5467, 24.2535, 18.6853};
    valuesSrc.clear();
    valuesDst.clear();
    for (auto& s : srcBad)
        valuesSrc.push_back(s);
    for (auto& s : dstBad)
        valuesDst.push_back(s);

    ROFL_VAR2(valuesSrc.size(), valuesDst.size());

    std::cout << "Computes candidates translations" << std::endl;
    for (int i = 0; i < valuesSrc.size(); ++i) {
        for (int j = 0; j < valuesDst.size(); ++j) {
            valuesDif.push_back(valuesDst[j] - valuesSrc[i]);
            ranges.push_back(range);
        }
    }
    ROFL_VAR2(valuesDif.size(), ranges.size());

    std::sort(valuesDif.begin(), valuesDif.end());

    std::ofstream filePlot(filenamePlot);
    if (!filePlot) {
        ROFL_ERR("Cannot open file \"" << filenamePlot << "\"");
        return -1;
    }
    filePlot << "plot '-' w p pt 7 ps 0.6\n";
    for (int i = 0; i < valuesDif.size(); ++i) {
        filePlot << " " << valuesDif[i] << " 0.0\n";
    }
    filePlot << "e\n";

    std::cout << "Performs histogram computation\n";
    std::vector<bool> inliers;
    // rofl::estimateTranslationTls(valuesDif.begin(), valuesDif.end(),
    //                             ranges.begin(), ranges.end(), translEst, inliers);
    rofl::estimateTranslationTls(valuesDif, ranges, translEst, inliers);

    std::cout << "Estimated translation: " << translEst << std::endl;

    return 0;
}