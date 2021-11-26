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
#ifndef ROFL_COMMON_MIN_MAX_HEAP_H_
#define ROFL_COMMON_MIN_MAX_HEAP_H_

#include <iostream>
#include <vector>
#include <rofl/common/macros.h>

namespace rofl {

	/**
	 * Class MinMaxHeap is a heap that efficiently keeps track about minimum and maximum items
	 * in the container.
	 * - Value: the item type stored in the container;
	 * - Comparator: the sorting functor type of the items; minimum value vmin is s.t. cmp(vmin, v) for all v;
	 * - Container: the min-max heap is a binary tree stored into a random access container like a std::vector;
	 * - Allocator: handler of memory dynamic allocation.
	 *
	 * Implementation details.
	 */
	template <typename Value, typename Comparator = std::less<Value>, template <typename, typename > class Container = std::vector,
			template <typename > class Allocator = std::allocator>
	class MinMaxHeap {
	public:
		using Type = MinMaxHeap<Value, Comparator, Container, Allocator>;
		using Storage = Container<Value, Allocator<Value> >;
		using Iterator = typename Storage::const_iterator;

		/** Creates an empty heap.
		 */
		MinMaxHeap() :
				data_(), cmp_() {

		}

		/** Creates an empty heap with the given comparator.
		 */
		MinMaxHeap(const Comparator& c) :
				data_(), cmp_(c) {
		}

		/** Destructor.
		 */
		~MinMaxHeap() {
		}

		/**
		 * Returns the number of elements currently in the heap.
		 */
		size_t size() const {
			return data_.size();
		}

		/**
		 * Says if the heap is empty.
		 */
		bool empty() const {
			return data_.empty();
		}

		/**
		 * Pushes a new value in the heap (warning: if the heap is full a value
		 * is removed!).
		 */
		void push(const Value& t) {
			data_.push_back(t);
			heapUp(data_.size() - 1);
		}

		/** Pops the minimum item.
		 */
		void popTop() {
			ROFL_ASSERT(!data_.empty());
			std::swap(data_[0], data_.back());
			data_.pop_back();
			heapDownMin(0);
		}

		/** Pops the maximum item.
		 */
		void popBottom() {
			ROFL_ASSERT(!data_.empty());
			if (data_.size() == 1) {
				data_.pop_back();
			} else {
				size_t imax = childMax(0);
				std::swap(data_[imax], data_.back());
				data_.pop_back();
				heapDownMax(imax);
			}
		}

		/** Returns the minimum element.
		 */
		const Value& top() const {
			ROFL_ASSERT(!data_.empty());
			return data_[0];
		}

		/** Returns the maximum element.
		 */
		const Value& bottom() const {
			ROFL_ASSERT(!data_.empty());
			if (data_.size() == 1) {
				return data_[0];
			} else {
				return data_[childMax(0)];
			}
		}

		/** Returns the next minimum element.
		 */
		const Value& nextTop() const {
			ROFL_ASSERT(!data_.empty());
			if (data_.size() == 1) {
				return data_[0];
			}
			else if (data_.size() < 4) {
				return data_[childMin(0)];
			}
			else {
				return data_[grandchildMin(0)];
			}
		}

		/** Returns the next minimum element.
		 */
		const Value& nextBottom() const {
			ROFL_ASSERT(!data_.empty());
			if (data_.size() < 3) {
				return data_[0];
			}
			else {
				return data_[childMin(0)];
			}
		}

		/**
		 * Output the content of the heap.
		 */
		void print(std::ostream& out) const {
			int l = 0;
			out << "\nlevel " << l << ": ";
			for (size_t i = 0; i < data_.size(); ++i) {
				if (l < level(i)) {
					l = level(i);
					out << "\nlevel " << l << ": ";
				}
				out << data_[i] << " [" << i << "], ";
			}
			out << std::endl;
		}

		Iterator begin() const {
			return data_.begin();
		}

		Iterator end() const {
			return data_.end();
		}

	private:
		Storage data_;
		Comparator cmp_;

		inline size_t level(size_t i) const {
			size_t n = 0;
			while (i > 0) {
				i = parent(i);
				n++;
			}
			return n;
		}

		inline size_t parent(size_t c) const {
			return ((c - 1) >> 1);
		}

		inline size_t child(size_t p, size_t pos) {
			return ((p << 1) + 1 + pos);
		}

		inline size_t childLeft(size_t p) const {
			return ((p << 1) + 1);
		}

		inline size_t childRight(size_t p) const {
			return ((p << 1) + 2);
		}

		inline size_t grandparent(size_t c) const {
			return ((c - 3) >> 2);
		}

		inline size_t grandchild(size_t p, size_t pos) const {
			return ((p << 2) + 3 + pos);
		}

		inline size_t grandchildLL(size_t p) const {
			return ((p << 2) + 3);
		}

		inline size_t grandchildLR(size_t p) const {
			return ((p << 2) + 4);
		}

		inline size_t grandchildRL(size_t p) const {
			return ((p << 2) + 5);
		}

		inline size_t grandchildRR(size_t p) const {
			return ((p << 2) + 6);
		}

		bool hasParent(size_t i) const {
			return (i > 0);
		}

		bool hasGrandparent(size_t i) const {
			return (i > 2);
		}

		bool hasChildren(size_t p) const {
			return (childLeft(p) < data_.size());
		}

		bool hasGrandchildren(size_t p) const {
			return (grandchild(p, 0) < data_.size());
		}

		/**
		 * Returns the minimum children of input node i.
		 * If input node has no children, then it returns an out-of-range index value.
		 */
		size_t childMin(size_t i) const {
			size_t imin = childLeft(i);
			size_t ichild = imin + 1;
			if (ichild < data_.size() && cmp_(data_[ichild], data_[imin])) {
				imin = ichild;
			}
			return imin;
		}

		/**
		 * Returns the minimum children of input node i.
		 * If input node has no children, then it returns an out-of-range index value.
		 */
		size_t childMax(size_t i) const {
			size_t imax = childLeft(i);
			size_t ichild = imax + 1;
			if (ichild < data_.size() && cmp_(data_[imax], data_[ichild])) {
				imax = ichild;
			}
			return imax;
		}

		/**
		 * Returns the position of the minimum grandchild of input node i.
		 * If input node has no children, then it returns an out-of-range index value.
		 */
		size_t grandchildMin(size_t i) const {
			size_t igch = grandchild(i, 0);
			size_t imin = igch;
			for (size_t j = igch + 1; j < igch + 4 && j < data_.size(); ++j) {
				if (cmp_(data_[j], data_[imin])) {
					imin = j;
				}
			}
			return imin;
		}

		/**
		 * Returns he position of the maximum grandchild of input node i.
		 * If input node has no children, then it returns an out-of-range index value.
		 */
		size_t grandchildMax(size_t i) const {
			size_t igch = grandchild(i, 0);
			size_t imax = igch;
			//ROFL_VAR3(i, igch, imax);
			for (size_t j = igch + 1; j < igch + 4 && j < data_.size(); ++j) {
				//ROFL_VAR6(igch, imax, j, data_[imax], data_[j], cmp_(data_[imax], data_[j]));
				if (cmp_(data_[imax], data_[j])) {
					imax = j;
				}
			}
			//ROFL_VAR1(imax);
			return imax;
		}

		/**
		 * Restores the invariant of the heap by taking item in position i towards the root
		 * when required.
		 * Given comparator functor cmp_() the invariant is the following:
		 *   if level(i) is even, then
		 *     !cmp_(data_[i], data_[grandparent(i)]) && cmp_(data_[i], data_[parent(i)])
		 *   else if level(i) is odd, then
		 *     cmp_(data_[i], data_[grandparent(i)]) && !cmp_(data_[i], data_[parent(i)])
		 */
		void heapUp(int i) {
			size_t imin, imax;

			// Even levels contain the minimum values of the subtree.
			// Odd levels contain the maxima values of subtree.
			imin = data_.size();
			imax = data_.size();
			bool onLevelMin = (level(i) % 2 == 0);
			if (onLevelMin) {
				if (hasGrandparent(i))
					imin = grandparent(i);
				if (hasParent(i))
					imax = parent(i);
			} else {
				if (hasParent(i))
					imin = parent(i);
				if (hasGrandparent(i))
					imax = grandparent(i);
			}
			//ROFL_VAR6(i, imin, imax, level(i), onLevelMin, data_.size());

			while (imin < data_.size() && cmp_(data_[i], data_[imin])) {
				//ROFL_MSG("swap items " << i << " and " << imin << ", i.e. " << data_[i] << " and " << data_[imin]);
				std::swap(data_[i], data_[imin]);
				i = imin;
				if (hasGrandparent(imin))
					imin = grandparent(imin);
				else
					imin = data_.size();
			}

			while (imax < data_.size() && !cmp_(data_[i], data_[imax])) {
				//ROFL_MSG("swap items " << i << " and " << imax << ", i.e. " << data_[i] << " and " << data_[imax]);
				std::swap(data_[i], data_[imax]);
				i = imax;
				if (hasGrandparent(imax))
					imax = grandparent(imax);
				else
					imax = data_.size();
			}
		}

		void heapDownMin(int i) {
			size_t icurr, ichild, igrand;
			icurr = i;
			while (icurr < data_.size()) {
				//ROFL_VAR3(icurr, grandchildMin(icurr), childMin(icurr));
				ichild = childMin(icurr);
				igrand = grandchildMin(icurr);
				// If the minimum value among icurr, the minimum child ichild and the minimum grandchild igrand is igrand,
				// then
				if (igrand < data_.size() && cmp_(data_[igrand], data_[icurr]) && cmp_(data_[igrand], data_[ichild])) {
					std::swap(data_[icurr], data_[igrand]);
					if (cmp_(data_[parent(igrand)], data_[igrand])) {
						std::swap(data_[parent(igrand)], data_[igrand]);
					}
					icurr = igrand;
				}
				else if (ichild < data_.size() && cmp_(data_[ichild], data_[icurr])) {
					std::swap(data_[icurr], data_[ichild]);
					icurr = ichild;
				} else {
					icurr = data_.size();
				}
			}
		}

		void heapDownMax(int i) {
			size_t icurr, ichild, igrand;
			icurr = i;
			while (icurr < data_.size()) {
				ichild = childMax(icurr);
				igrand = grandchildMax(icurr);
				//ROFL_VAR5(icurr, grandchildMax(icurr), igrand, childMax(icurr), ichild);
				// If the minimum value among icurr, the minimum child ichild and the minimum grandchild igrand is igrand,
				// then
				if (igrand < data_.size() && cmp_(data_[icurr],data_[igrand]) && cmp_(data_[ichild], data_[igrand])) {
					//ROFL_MSG("max grandchild " << data_[igrand] << " [" << igrand << "] swapped with " << data_[icurr] << " [" << icurr << "]");
					std::swap(data_[icurr], data_[igrand]);
					//ROFL_MSG("now " << data_[igrand] << " [" << igrand << "] and " << data_[icurr] << " [" << icurr << "]");
					if (cmp_( data_[igrand], data_[parent(igrand)])) {
						//ROFL_MSG("parent of " << data_[igrand] << " [" << igrand << "] is " << data_[parent(igrand)] << " [" << parent(igrand) << "]");
						std::swap(data_[parent(igrand)], data_[igrand]);
					}
					icurr = igrand;
				} else if (ichild < data_.size() && cmp_( data_[icurr], data_[ichild])) {
					std::swap(data_[icurr], data_[ichild]);
					icurr = data_.size();
				} else {
					icurr = data_.size();
				}
			}
		}

		void heapDown(int i) {
			if (level(i) % 2 == 0) {
				heapDownMin(i);
			}
			else {
				heapDownMax(i);
			}
		}
	};

}

#endif
