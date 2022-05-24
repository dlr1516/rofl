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
#include <rofl/common/morton_tree.h>
#include <rofl/common/param_map.h>

#include <iostream>
#include <vector>

int main(int argc, char** argv) {
    rofl::ParamMap params;
    rofl::MortonTree2f tree;
    rofl::MortonTree2f::Item item, query, boxMin, boxMax;
    rofl::MortonTree2f::IteratorContainer inRange;
    float angleBeg, angleInc, angle, rhoK, range;
    int angleNum;
    std::string filenamePlot;

    params.read(argc, argv);
    params.getParam<float>("angleBeg", angleBeg, float(0.0));
    params.getParam<float>("angleInc", angleInc, float(13.0));
    params.getParam<int>("angleNum", angleNum, int(100));
    params.getParam<float>("rhoK", rhoK, float(1.5));
    angleBeg *= M_PI / 180.0;
    angleInc *= M_PI / 180.0;
    params.getParamContainer("query", query.values, query.values + 2, "[1.5,0.1]", 0.0f, "[,]");
    params.getParam<float>("range", range, float(1.2));
    query.id = -1;
    params.getParam<std::string>("plot", filenamePlot, "morton.plot");

    std::cout << "Params:\n";
    params.write(std::cout);
    std::cout << std::endl;

    std::cout << "query point [" << query.values[0] << "," << query.values[1] << "]" << std::endl;

    // for (int i = 0; i < angleNum; ++i) {
    //     angle = angleBeg + angleInc * i;
    //     item.values[0] = rhoK * angle * cos(angle);
    //     item.values[1] = rhoK * angle * sin(angle);
    //     item.id = i;
    //     tree.insert(item);
    // }

    for (int ix = -4; ix < 4; ++ix) {
        for (int iy = -4; iy < 4; ++iy) {
            item.values[0] = ix;
            item.values[1] = iy;
            item.id = ix + iy;
            tree.insert(item);
        }
    }
    std::cout << "inserted " << tree.size() << " items" << std::endl;

    tree.searchInBox(query, range, inRange);
    std::cout << "query in range:\n";
    for (auto& i : inRange) {
        std::cout << "  [" << i->values[0] << ", " << i->values[1] << "]\n";
    }

    std::ofstream filePlot(filenamePlot);
    if (!filePlot) {
        std::cerr << "Cannot open output plot file \"" << filenamePlot << "\"" << std::endl;
        return -1;
    }
    filePlot << "set term wxt 0\n";
    filePlot << "set title \"Morton tree\"\n";
    filePlot << "set size ratio -1\n";
    filePlot << "set xrange [-5:5]\n";
    filePlot << "set yrange [-5:5]\n";
    filePlot << "plot '-' w l lw 1.0, '-' w l lw 0.5, '-' w p pt 7\n";
    filePlot
        << "  " << (query.values[0] - range) << " " << (query.values[1] - range) << "\n"
        << "  " << (query.values[0] - range) << " " << (query.values[1] + range) << "\n"
        << "  " << (query.values[0] + range) << " " << (query.values[1] + range) << "\n"
        << "  " << (query.values[0] + range) << " " << (query.values[1] - range) << "\n"
        << "  " << (query.values[0] - range) << " " << (query.values[1] - range) << "\n"
        << "e\n";
    for (auto& p : tree.getItems()) {
        filePlot << p.values[0] << " " << p.values[1] << "\n";
    }
    filePlot << "e" << std::endl;
    for (auto& p : tree.getItems()) {
        filePlot << p.values[0] << " " << p.values[1] << "\n";
    }
    filePlot << "e" << std::endl;

    return 0;
}