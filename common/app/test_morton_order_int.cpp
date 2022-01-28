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
#include <iostream>
#include <fstream>
#include <array>
#include <vector>
#include <rofl/common/morton_sort.h>
#include <rofl/common/param_map.h>

int main(int argc, char** argv) {
    using Point = std::array<int, 2>;
    using VectorPoint = std::vector<Point>;
    VectorPoint points;
    Point pIn;
    int xbeg, xend, ybeg, yend;
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
    params.getParam<std::string>("plot", filenamePlot, "morton.plot");

    std::cout << "Params:" << std::endl;
    params.write(std::cout);

    for (pIn[0] = xbeg; pIn[0] < xend; ++pIn[0]) {
        for (pIn[1] = ybeg; pIn[1] < yend; ++pIn[1]) {
            points.push_back(pIn);
        }
    }
    std::sort(points.begin(), points.end(),
            [&](const Point& p1, const Point & p2) -> bool {
                return rofl::mortonCmpInt<int, 2>(p1.data(), p2.data()); }
    );

    std::cout << "\nSorted points in morton order:\n";
    for (auto& p : points) {
        std::cout << "  [" << p[0] << "," << p[1] << "]\n";
    }

    std::ofstream filePlot(filenamePlot);
    if (!filePlot) {
        std::cerr << "Cannot open output plot file \"" << filenamePlot << "\"" << std::endl;
        return -1;
    }
    filePlot << "set term wxt 0\n";
    filePlot << "set size ratio -1\n";
    filePlot << "set xrange [" << (xbeg - 1) << ":" << (xend + 1) << "]\n";
    filePlot << "set yrange [" << (ybeg - 1) << ":" << (yend + 1) << "]\n";
    filePlot << "plot '-' w l\n";
    for (auto& p : points) {
        filePlot << p[0] << " " << p[1] << "\n";
    }
    filePlot << "e" << std::endl;
    filePlot.close();

    std::cout << "To view the outcome of Morton order, run the command:\n\ngnuplot -persist " << filenamePlot << "\n" << std::endl;

    return 0;
}




