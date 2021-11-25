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
#include <cmath>
#include <vector>
#include <rofl/common/interval_indices.h>
#include <rofl/common/peak_finder_d.h>
#include <rofl/common/grid.h>

using MyPeakFinder = rofl::PeakFinderD<2, float, int, std::greater<float> >;
using MyGrid2f = rofl::Grid<2, float>;
using MyIndices = typename MyGrid2f::Indices;

void fillGaussian(MyGrid2f& grid, float kfactor, float width, int c0, int c1);

void plotGrid2(const MyGrid2f& grid, const std::string& filename, float factor = 1.0f);

int main(int argc, char** argv) {
	MyPeakFinder peakFinder;
	MyGrid2f grid2f( { 120, 100 });
	auto mymap = [&](const MyIndices& indices) -> float {
		return grid2f.value(indices);
	};
	std::vector<MyIndices> indicesMax;

	std::cout << "Filling grid with gaussians\n";
	grid2f.fill(0.0f);
	fillGaussian(grid2f, 8.0, 0.5, 55, 40);
	fillGaussian(grid2f, 7.0, 1.2, 102, 80);
	fillGaussian(grid2f, 4.0, 1.6, 20, 20);
	fillGaussian(grid2f, 3.0, 0.2, 20, 80);
	fillGaussian(grid2f, 3.0, 0.2, 26, 80);

	std::cout << "Exporting grid to Gnuplot file\n";
	plotGrid2(grid2f, "grid_example.plot", 2.0);

//	rofl::MinMaxHeap<int,std::greater<int> > queue;
//	for (int c = 0; c < 17; ++c) {
//		int item = (23 * c + 5) % 17;
//		queue.push(item);
//		std::cout << "pushed " << item << ", top " << queue.top() << ", bottom " << queue.bottom() << std::endl;
//	}
//	int counter = 0;
//	while (!queue.empty()) {
//		if (counter % 2 == 0) {
//			std::cout << "top " << queue.top() << ", bottom " << queue.bottom() << ": popTop" << std::endl;
//			queue.popTop();
//		}
//		else {
//			std::cout << "top " << queue.top() << ", bottom " << queue.bottom() << ": popBottom" << std::endl;
//			queue.popBottom();
//		}
//		counter++;
//	}
//	return 0;

	std::cout << "Finding maxima\n";
	peakFinder.setDomain(grid2f.dimensions());
	peakFinder.setPeakWindow( { 8, 8 });
	//peakFinder.enableFilterMinValue(true, 1.0f);
	peakFinder.detect(mymap, std::back_inserter(indicesMax));

	std::cout << "Maxima:\n";
	for (auto &idx : indicesMax) {
		std::cout << "  " << idx << " value " << mymap(idx) << " grid2.value() " << grid2f.value(idx) << std::endl;
	}

	return 0;
}

void fillGaussian(MyGrid2f& grid, float kfactor, float width, int c0, int c1) {
	float r2;
	int dim0 = grid.dimensions()[0];
	int dim1 = grid.dimensions()[1];

	for (int i0 = 0; i0 < dim0; ++i0) {
		for (int i1 = 0; i1 < dim1; ++i1) {
			r2 = ((i0 - c0) * (i0 - c0) + (i1 - c1) * (i1 - c1)) / (width * width);
			grid.value( { i0, i1 }) += kfactor * exp(-0.5 * r2);
		}
	}
}

void plotGrid2(const MyGrid2f& grid, const std::string& filename, float factor) {
	int dim0 = grid.dimensions()[0];
	int dim1 = grid.dimensions()[1];
	int dim0Reduced = round(dim0 / factor);
	int dim1Reduced = round(dim1 / factor);

	std::ofstream file(filename);
	if (!file) {
		std::cerr << "Cannot open \"" << filename << "\"" << std::endl;
		return;
	}

	file << "set grid nopolar\n" << "set style data lines\n" << "set dgrid3d " << dim0Reduced << "," << dim1Reduced << "\n" << "set hidden3d\n";
	//file << "xlabel \"i0\"\n" << "ylabel \"i1\"\n";

	file << "splot '-'\n";
	for (int i0 = 0; i0 < dim0; ++i0) {
		for (int i1 = 0; i1 < dim1; ++i1) {
			file << i0 << " " << i1 << " " << grid.value( { i0, i1 }) << "\n";
		}
	}
	file << "e\n";

	file.close();
}

