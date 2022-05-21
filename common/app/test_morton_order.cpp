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
#include <rofl/common/morton_sort.h>
#include <rofl/common/param_map.h>

#include <array>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

int main(int argc, char** argv) {
    using PointI = std::array<int, 2>;
    using PointF = std::array<float, 2>;
    using VectorPointI = std::vector<PointI>;
    using VectorPointF = std::vector<PointF>;
    VectorPointI pointsInt;
    VectorPointF pointsFloat;
    PointI pInt;
    PointF pFloat;
    int xbeg, xend, ybeg, yend;
    unsigned xorI;
    float f1, f2, res, xorF;
    rofl::ParamMap params;
    std::string filenameCfg, filenamePlot;

    params.read(argc, argv);
    params.getParam<std::string>("cfg", filenameCfg, "");
    std::cout << "config filename: " << filenameCfg << std::endl;
    if (filenameCfg != "") {
        params.read(filenameCfg);
    }
    params.read(argc, argv);

    params.getParam<int>("xbeg", xbeg, -4);
    params.getParam<int>("xend", xend, +4);
    params.getParam<int>("ybeg", ybeg, -4);
    params.getParam<int>("yend", yend, +4);
    params.getParam<float>("res", res, 1.0f);
    params.getParam<std::string>("plot", filenamePlot, "morton.plot");

    std::cout << "Params:" << std::endl;
    params.write(std::cout);

    for (pInt[0] = xbeg; pInt[0] < xend; ++pInt[0]) {
        for (pInt[1] = ybeg; pInt[1] < yend; ++pInt[1]) {
            pointsInt.push_back(pInt);
            pFloat[0] = res * pInt[0];
            pFloat[1] = res * pInt[1];
            pointsFloat.push_back(pFloat);
        }
    }

    for (int i = xbeg; i < xend; ++i) {
        for (int j = i; j < xend; ++j) {
            xorI = rofl::IntegerTraits<int>::removeSign(i) ^ rofl::IntegerTraits<int>::removeSign(j);
            f1 = res * i;
            f2 = res * j;
            xorF = rofl::xorFloat(f1, f2);
            std::cout
                << "i " << std::setw(3) << i << " (" << std::setw(3) << (res * i) << ")  "
                << "j " << std::setw(3) << j << " (" << std::setw(3) << (res * j) << "): "
                << "xorI " << std::setw(3) << xorI << " xorF " << std::setw(3) << xorF << std::endl;
        }
    }

    std::sort(pointsInt.begin(), pointsInt.end(),
              [&](const PointI& p1, const PointI& p2) -> bool {
                  return rofl::MortonTraits<int, 2>::compare(p1.data(), p2.data());
                  // return rofl::mortonCmpInt<int, 2>(p1.data(), p2.data());
              });
    std::sort(pointsFloat.begin(), pointsFloat.end(),
              [&](const PointF& p1, const PointF& p2) -> bool {
                  return rofl::MortonTraits<float, 2>::compare(p1.data(), p2.data());
              });

    std::cout << "\nSorted Integer points in morton order:\n";
    for (auto& p : pointsInt) {
        std::cout << "  [" << p[0] << "," << p[1] << "]\n";
    }
    std::cout << "\nSorted Float points in morton order:\n";
    for (auto& p : pointsFloat) {
        std::cout << "  [" << p[0] << "," << p[1] << "]\n";
    }

    std::ofstream filePlot(filenamePlot);
    if (!filePlot) {
        std::cerr << "Cannot open output plot file \"" << filenamePlot << "\"" << std::endl;
        return -1;
    }
    filePlot << "set term wxt 0\n";
    filePlot << "set title \"Morton order integer in interval [" << xbeg << "," << xend
             << "[ x [" << ybeg << "," << yend << "[\"\n";
    filePlot << "set size ratio -1\n";
    filePlot << "set xrange [" << (xbeg - 1) << ":" << (xend + 1) << "]\n";
    filePlot << "set yrange [" << (ybeg - 1) << ":" << (yend + 1) << "]\n";
    filePlot << "plot '-' w l\n";
    for (auto& p : pointsInt) {
        filePlot << p[0] << " " << p[1] << "\n";
    }
    filePlot << "e" << std::endl;

    filePlot << "set term wxt 1\n";
    filePlot << "set title \"Morton order float in interval [" << (res * xbeg) << "," << (res * xend)
             << "[ x [" << (res * ybeg) << "," << (res * yend) << "[\"\n";
    filePlot << "set size ratio -1\n";
    filePlot << "set xrange [" << res * (xbeg - 1) << ":" << res * (xend + 1) << "]\n";
    filePlot << "set yrange [" << res * (ybeg - 1) << ":" << res * (yend + 1) << "]\n";
    filePlot << "plot '-' w l\n";
    for (auto& p : pointsFloat) {
        filePlot << p[0] << " " << p[1] << "\n";
    }
    filePlot << "e" << std::endl;
    filePlot.close();

    std::cout << "To view the outcome of Morton order, run the command:\n\ngnuplot -persist " << filenamePlot << "\n"
              << std::endl;

    return 0;
}
