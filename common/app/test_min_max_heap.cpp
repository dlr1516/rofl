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
