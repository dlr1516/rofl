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
#ifndef ROFL_COMMON_INTERVAL_INDICES_H_
#define ROFL_COMMON_INTERVAL_INDICES_H_

#include <iostream>
#include <array>
#include <rofl/common/macros.h>

template <typename I,size_t D>
std::ostream& operator<<(std::ostream& out, const std::array<I,D>& indices) {
	out << "[";
	for (int d = 0; d < D; ++d) {
		out << indices[d];
		if (d < D - 1)
			out << ",";
	}
	out << "]";
	return out;
}

namespace rofl {

	// ------------------------------------------------------------------------
	// DETAIL: indexers converting a position index into indices over an
	// interval
	// ------------------------------------------------------------------------

	namespace detail {

		/**
		 * Struct RasterIndexer converts position index pos into indices over
		 * an interval following raster order.
		 *
		 * Example: interval [3:5,1:3] is visited in order
		 *   pos 0 -> [3,1], pos 1 -> [4,1], pos 2 -> [5,1],
		 *   pos 3 -> [3,2], pos 4 -> [4,2], pos 5 -> [5,2],
		 *   pos 6 -> [3,3], pos 7 -> [4,3], pos 8 -> [5,3]
		 */
		template <size_t Dim,typename Index>
		struct RasterIndexer {
			using Indices = std::array<Index, Dim>;

			static Indices getIndices(const Indices& minval, const Indices& dimensions, Index pos) {
				Indices indices;
				size_t res = pos;
				for (Index d = 0; d < Dim; ++d) {
					indices[d] = minval[d] + res % dimensions[d];
					res = res / dimensions[d];
				}
				return indices;
			}
		};

		/**
		 * Struct BoustrophedonIndexer converts position index pos into indices over
		 * an interval following boustrophedron ("zig-zag") order.
		 *
		 * Example: interval [3:5,1:3] is visited in order
		 *   pos 0 -> [3,1], pos 1 -> [4,1], pos 2 -> [5,1],
		 *   pos 3 -> [5,2], pos 4 -> [4,2], pos 5 -> [3,2],
		 *   pos 6 -> [3,3], pos 7 -> [4,3], pos 8 -> [5,3]
		 */
		template <size_t Dim,typename Index>
		struct BoustrophedonIndexer {
			using Indices = std::array<Index, Dim>;

			static Indices getIndices(const Indices& minval, const Indices& dimensions, Index pos) {
				Indices indices;
				Index res = pos;
				for (Index d = 0; d < Dim; ++d) {
					indices[d] = res % dimensions[d];
					res = res / dimensions[d];
				}
				Index odd = 0;
				for (Index d = Dim - 1; d >= 0; --d) {
					//ROFL_VAR3(d, indices[d], odd);
					if (odd) {
						odd = indices[d] % 2;
						indices[d] = dimensions[d] - indices[d] - 1;
					} else {
						odd = indices[d] % 2;
					}
					indices[d] += minval[d];
				}
				//ROFL_MSG("indices computed " << indices);
				return indices;
			}
		};

		template <size_t Dim,typename Index,template <size_t,typename > class Indexer> class IntervalIterator;

		template <size_t Dim,typename Index> class RasterIterator;

		template <size_t Dim,typename Index> class BoustrophedonIterator;

	}

	template <size_t Dim,typename Index = int>
	class IntervalIndices {
	public:
		using ThisType = IntervalIndices<Dim, Index>;
		using Indices = std::array<Index, Dim>;
		using RasterIteratorType = detail::RasterIterator<Dim, Index>;
		using BoustrophedonIteratorType = detail::BoustrophedonIterator<Dim, Index>;

		friend RasterIteratorType;
		friend BoustrophedonIteratorType;

		static const size_t DIM = Dim;

		IntervalIndices() : min_(), dimensions_() {
			min_.fill(0);
			dimensions_.fill(0);
		}

		IntervalIndices(const Indices& min, const Indices& dimensions) : min_(min), dimensions_(dimensions) {
		}

		virtual ~IntervalIndices() {
		}

		void initBounds(const Indices& min, const Indices& dimensions) {
			min_ = min;
			dimensions_ = dimensions;
		}

		void initWindow(const Indices& winCenter, const Indices& winSize) {
			for (size_t d = 0; d < Dim; ++d) {
				min_[d] = winCenter[d] - winSize[d];
				dimensions_[d] = 2 * winSize[d] + 1;
			}
		}

		void initMinMax(const Indices& min, const Indices& max) {
			for (size_t d = 0; d < Dim; ++d) {
				min_[d] = min[d];
				dimensions_[d] = max[d] - min[d] + 1;
			}
		}

		ThisType clone() const {
			return ThisType(min_, dimensions_);
		}

		const Indices& dimensions() const {
			return dimensions_;
		}

		Index dimension(size_t d) const {
			return dimensions_[d];
		}

		const Indices& min() const {
			return min_;
		}

		Indices max() const {
			Indices ret;
			for (size_t d = 0; d < Dim; ++d) {
				ret[d] = min_[d] + dimensions_[d] - 1;
			}
			return ret;
		}

		const Indices& first() const {
			return min_;
		}

		Indices last() const {
			Indices ret;
			for (size_t d = 0; d < Dim; ++d) {
				ret[d] = min_[d] + dimensions_[d];
			}
			return ret;
		}

		/**
		 * Says if the interval is empty.
		 */
		bool empty() const {
			for (int d = 0; d < Dim; ++d) {
				if (dimensions_[d] <= 0) {
					return true;
				}
			}
			return false;
		}

		size_t size() const {
			size_t s = 1;
			for (int d = 0; d < Dim; ++d) {
				s *= dimensions_[d];
			}
			return s;
		}

		bool inside(const Indices& indices) const {
			for (int d = 0; d < Dim; ++d) {
				if (indices[d] < min_[d] || indices[d] >= min_[d] + dimensions_[d]) {
					return false;
				}
			}
			return true;
		}

		ThisType intersect(const ThisType& interval) const {
			ThisType intersection;
			Indices intersMin, intersMax;
			if (empty()) {
				return clone();
			} else if (interval.empty()) {
				return interval.clone();
			}
			for (int d = 0; d < Dim; ++d) {
				intersMin[d] = std::max(min_[d], interval.min_[d]);
				intersMax[d] = std::min(min_[d] + dimensions_[d], interval.min_[d] + interval.dimensions_[d]) - 1;
			}
			intersection.initMinMax(intersMin, intersMax);
			return intersection;
		}

		ThisType stacked(size_t d, Index incr) const {
			ROFL_ASSERT_VAR2(d < Dim, d, Dim);
			ThisType expansion;
			Indices expandMin = min_;
			Indices expandDim = dimensions_;
			if (incr > 0) {
				expandMin[d] += dimensions_[d];
				expandDim[d] = incr;
			} else {
				expandMin[d] += incr;
				expandDim[d] = -incr;
			}
			expansion.initBounds(expandMin, expandDim);
			return expansion;
		}

		void translate(size_t d, Index incr) {
			ROFL_ASSERT_VAR2(d < Dim, d, Dim);
			min_[d] += incr;
		}

		void translate(const Indices& t) {
			for (size_t d = 0; d < Dim; ++d) {
				min_[d] += t[d];
			}
		}

		RasterIteratorType beginRaster() const {
			return RasterIteratorType(this, 0);
		}

		RasterIteratorType endRaster() const {
			return RasterIteratorType(this, this->size());
		}

		BoustrophedonIteratorType beginBoustrophedon() const {
			return BoustrophedonIteratorType(this, 0);
		}

		BoustrophedonIteratorType endBoustrophedon() const {
			return BoustrophedonIteratorType(this, this->size());
		}

		template <size_t D,typename I>
		friend std::ostream& operator<<(std::ostream& out, const IntervalIndices<D,I>& interval) {
			out << "[";
			for (int d = 0; d < D; ++d) {
				out << interval.min()[d] << ":" << interval.max()[d];
				if (d < Dim - 1)
					out << ",";
			}
			out << "]";
			return out;
		}

	private:
		Indices min_;
		Indices dimensions_;
	};

	namespace detail {

		template <size_t Dim,typename Index,template <size_t,typename > class Indexer>
		class IntervalIterator {
		public:
			using ThisType = IntervalIterator<Dim, Index, Indexer>;
			using IntervalType = IntervalIndices<Dim, Index>;
			using IndexerType = Indexer<Dim, Index>;
			using Indices = typename IntervalType::Indices;

			using iterator_category = std::bidirectional_iterator_tag;
			using difference_type = std::ptrdiff_t;
			using value_type = Indices;
			using pointer = const Indices*;
			using reference = const Indices&;

			static const size_t DIM = Dim;

			IntervalIterator(const IntervalType* interval, Index pos) : interval_(interval), pos_(pos), indices_() {
				ROFL_ASSERT(interval_ != nullptr);
				indices_ = IndexerType::getIndices(interval_->min(), interval_->dimensions(), pos);
			}

			IntervalIterator(const IntervalIterator& it) : interval_(it.interval_), pos_(it.pos_), indices_(it.indices_) {
				ROFL_ASSERT(interval_ != nullptr);
			}

			virtual ~IntervalIterator() {
			}

			reference operator*() const {
				return indices_;
			}

			pointer operator->() const {
				return &indices_;
			}

			IntervalIterator& operator++() {
				++pos_;
				indices_ = IndexerType::getIndices(interval_->min(), interval_->dimensions(), pos_);
				return *this;
			}

			IntervalIterator operator++(int) {
				IntervalIterator tmp = *this;
				++(*this);
				return tmp;
			}

			IntervalIterator& operator--() {
				--pos_;
				indices_ = IndexerType::getIndices(interval_->min(), interval_->dimensions(), pos_);
				return *this;
			}

			IntervalIterator operator--(int) {
				IntervalIterator tmp = *this;
				--(*this);
				return tmp;
			}

			friend bool operator==(const IntervalIterator& it1, const IntervalIterator& it2) {
				return (it1.pos_ == it2.pos_);
			}

			friend bool operator!=(const IntervalIterator& it1, const IntervalIterator& it2) {
				return (it1.pos_ != it2.pos_);
			}

		private:
			const IntervalType *interval_;
			Index pos_;
			Indices indices_;
		};

		template <size_t Dim,typename Index>
		class RasterIterator: public IntervalIterator<Dim,Index,RasterIndexer> {
		public:
			using BaseType = IntervalIterator<Dim, Index, RasterIndexer>;
			using IntervalType = typename BaseType::IntervalType;
			using Indices = typename BaseType::Indices;

			RasterIterator(const IntervalType* interval, Index pos) : BaseType(interval, pos) {
			}

			virtual ~RasterIterator() {
			}
		};

		template <size_t Dim,typename Index>
		class BoustrophedonIterator: public IntervalIterator<Dim,Index,BoustrophedonIndexer> {
		public:
			using BaseType = IntervalIterator<Dim, Index, BoustrophedonIndexer>;
			using IntervalType = typename BaseType::IntervalType;
			using Indices = typename BaseType::Indices;

			BoustrophedonIterator(const IntervalType* interval, Index pos) : BaseType(interval, pos) {
			}

			virtual ~BoustrophedonIterator() {
			}
		};

	}

}

#endif /* COMMON_INCLUDE_ROFL_COMMON_INTERVAL_INDICES_H_ */

