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
#include <rofl/common/interval_indices.h>

using II = rofl::IntervalIndices<2, int>;

int main(int argc,char** argv) {
	II win1, win2, win3, win12, win13;

    win1.initMinMax({0, 0}, {5, 3});
    win2.initWindow({6,5}, {3,4});
    win3 = win1;
    win3.translate({10, 10});

    win12 = win1.intersect(win2);
    win13 = win3.intersect(win13);

	std::cout << "win1 " << win1 << " size " << win1.size() << "\n"
			<< "win2 " << win2 << " size " << win2.size() << "\n"
			<< "win3 " << win3 << " size " << win3.size() << "\n"
			<< "win12 " << win12 << " size " << win12.size() << "\n"
			<< "win13 " << win13 << "  win13.empty() " << win13.empty() << " size " << win13.size() << "\n"
			<< std::endl;

	std::cout << "\nraster iterate on win12" << std::endl;
	for (auto it = win12.beginRaster(); it != win12.endRaster(); ++it) {
		std::cout << "  " << *it << "\n";
	}

	std::cout << "\nboustrophedon iterate on win12" << std::endl;
	int counter = 0;
	for (auto it = win12.beginBoustrophedon(); it != win12.endBoustrophedon() && counter < 20; ++it, ++counter) {
		std::cout << "  " << *it << "\n";
	}

	return 0;
}








