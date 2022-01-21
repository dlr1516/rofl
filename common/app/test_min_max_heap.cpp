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
#include <rofl/common/min_max_heap.h>

int main(int argc, char** argv) {
	rofl::MinMaxHeap<int> heap;
	int val;

	std::cout << "**** PUSH set ****" << std::endl;
	for (int i = 0; i < 23; ++i) {
		val = (29 * i + 7) % 23;
		if (!heap.empty())
			std::cout
					<< "---\ninsert value " << val << ": top " << heap.top() << " next top " << heap.nextTop() << " bottom " << heap.bottom() << " next bottom "
					<< heap.nextBottom() << "\n";
		else
			std::cout << "---\ninsert value " << val << ": empty heap\n";
		heap.push(val);
		heap.print(std::cout);
	}

	std::cout << "\n**** POP set ****" << std::endl;
	while (!heap.empty()) {
		if (heap.size() % 2) {
			std::cout
					<< "\n---\npopMax(): top " << heap.top() << " next top " << heap.nextTop() << " bottom " << heap.bottom() << " next bottom "
					<< heap.nextBottom() << std::endl;
			heap.popTop();
			heap.print(std::cout);
		} else {
			std::cout
					<< "\n---\npopMin(): top " << heap.top() << " next top " << heap.nextTop() << " bottom " << heap.bottom() << " next bottom "
					<< heap.nextBottom() << std::endl;
			heap.popBottom();
			heap.print(std::cout);
		}
	}

	return 0;
}
