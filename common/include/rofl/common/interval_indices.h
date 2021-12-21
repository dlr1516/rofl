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
#include <sstream>
#include <array>
#include <rofl/common/macros.h>

namespace rofl {

	/**
	 * operator<< is defined only in rofl workspace to avoid potential conflicts.
	 * To use it outside the namespace the following instruction is needed:
	 *
	 *    using rofl::operator<<
	 */
//	template <typename I,size_t D>
//	std::ostream& operator<<(std::ostream& out, const std::array<I,D>& indices) {
//		out << "[";
//		for (int d = 0; d < D; ++d) {
//			out << indices[d];
//			if (d < D - 1)
//				out << ",";
//		}
//		out << "]";
//		return out;
//	}

	// ------------------------------------------------------------------------
	// DETAIL: indexers converting a position index into indices over an
	// interval
	// ------------------------------------------------------------------------

	namespace detail {

		/**
		 * Struct StaticRasterIndexer converts position index pos into indices over
		 * an interval following raster order.
		 *
		 * Example: interval [3:5,1:3] is visited in order
		 *   pos 0 -> [3,1], pos 1 -> [4,1], pos 2 -> [5,1],
		 *   pos 3 -> [3,2], pos 4 -> [4,2], pos 5 -> [5,2],
		 *   pos 6 -> [3,3], pos 7 -> [4,3], pos 8 -> [5,3]
		 */
		template <size_t Dim,typename Index>
		struct StaticRasterIndexer {
			static Index getPos(const Index* start, const Index* dimensions, const Index* indices) {
				return (*indices - *start) + (*dimensions) * StaticRasterIndexer<Dim - 1,Index>::getPos(start + 1, dimensions + 1, indices + 1);
			}

			static void getIndices(Index pos, const Index* start, const Index* dimensions, const Index* indices) {
				Index *ptr = const_cast<Index*>(indices);
				*ptr = *start + pos % (*dimensions);
				StaticRasterIndexer<Dim - 1,Index>::getIndices(pos / (*dimensions), start + 1, dimensions + 1, indices + 1);
			}
		};

		template <typename Index>
		struct StaticRasterIndexer<0,Index> {
			static Index getPos(const Index* min, const Index* dimensions, const Index* indices) {
				return 0;
			}

			static void getIndices(Index pos, const Index* start, const Index* dimensions, const Index* indices) {
				return;
			}
		};

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

			static Index getPos(const Indices& minval, const Indices& dimensions, const Indices& indices) {
				Index index = 0;
				for (int d = Dim - 1; d >= 0; --d) {
					index = index * dimensions[d] + indices[d] - minval[d];
				}
				return index;
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
				for (int d = 0; d < Dim; ++d) {
					indices[d] = res % dimensions[d];
					res = res / dimensions[d];
				}
				Index odd = 0;
				for (int d = Dim - 1; d >= 0; --d) {
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

		/**
		 * Forward declaration only.
		 * Class IntervalIterator is the base class for iterators over multi-dimensional intervals.
		 * Template parameters:
		 * - Dim: the dimension of the space;
		 * - Index: integer type for the index (prefer a signed integer type);
		 * - Indexer: the indexing policy class corresponding to the concept
		 *
		 *     struct Indexer {
		 *        using Indices = std::array<Index, Dim>;
		 *        static Indices getIndices(const Indices& minval, const Indices& dimensions, Index pos);
		 *     };
		 */
		template <size_t Dim,typename Index,template <size_t,typename > class Indexer> class IntervalIterator;

		/**
		 * Forward declaration only of class RasterIterator.
		 * See its implementing policy RasterIndexer.
		 */
		template <size_t Dim,typename Index> class RasterIterator;

		/**
		 * Forward declaration only of class BoustrophedonIterator.
		 * See its implementing policy BoustrophedonIndexer.
		 */
		template <size_t Dim,typename Index> class BoustrophedonIterator;

	}  // end of namespace detail


	// ------------------------------------------------------------------------
	// INTERVAL INDICES
	// ------------------------------------------------------------------------

	/**
	 * Class IntervalIndices handles interval of indices over a multi-dimensional space
	 * of size Dim.
	 */
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

		/**
		 * Default constructor.
		 * It creates an empty interval.
		 */
		IntervalIndices() : min_(), dimensions_() {
			min_.fill(0);
			dimensions_.fill(0);
		}

		/**
		 * Constructor with given min indices and dimensions.
		 * Example: Dim =2, min=[4,1], dimensions=[3,2] creates an interval spanning
		 *   on indices[0] from 4 to 6 (end value 4+3 = 7 not included)
		 *   on indices[1] from 1 to 2 (end value 1+2 = 3 not included)
		 * @param min min values of indices in each coordinate
		 * @param dimensions dimension sizes of indices in each coordinate
		 */
		IntervalIndices(const Indices& min, const Indices& dimensions) : min_(min), dimensions_(dimensions) {
		}

		/**
		 * Destructor.
		 */
		virtual ~IntervalIndices() {
		}

		/**
		 * Initializes the interval with given min indices and dimensions.
		 * Example: Dim =2, min=[4,1], dimensions=[3,2] creates an interval spanning
		 *   on indices[0] from 4 to 6 (end value 4+3 = 7 not included)
		 *   on indices[1] from 1 to 2 (end value 1+2 = 3 not included)
		 * @param min min values of indices in each coordinate
		 * @param dimensions dimension sizes of indices in each coordinate
		 */
		void initBounds(const Indices& min, const Indices& dimensions) {
			min_ = min;
			dimensions_ = dimensions;
		}

		/**
		 * Initializes the interval centered on the given winCenter and half-size winSize.
		 * Example: Dim=2, winCenter=[2,5], winSize=[3,2] creates an interval spanning
		 *   on indices[0] from 2-3=-1 to 2+3=5
		 *   on indices[1] from 5-2= 3 to 5+2=7
		 * @param winCenter center of the interval ("window")
		 * @param winSize (half) dimension of the interval ("window")
		 */
		void initCentered(const Indices& winCenter, const Indices& winSize) {
			for (size_t d = 0; d < Dim; ++d) {
				min_[d] = winCenter[d] - winSize[d];
				dimensions_[d] = 2 * winSize[d] + 1;
			}
		}

		/**
		 * Initializes the interval with given min and max indices.
		 * Example: Dim =2, min=[4,1], max=[6,2] creates an interval spanning
		 *   on indices[0] from 4 to 6
		 *   on indices[1] from 1 to 2
		 * @param min min values of indices in each coordinate (included in interval)
		 * @param max min values of indices in each coordinate (included in interval)
		 */
		void initMinMax(const Indices& min, const Indices& max) {
			for (size_t d = 0; d < Dim; ++d) {
				min_[d] = min[d];
				dimensions_[d] = max[d] - min[d] + 1;
			}
		}

		/**
		 * Returns a copy of the interval.
		 */
		ThisType clone() const {
			return ThisType(min_, dimensions_);
		}

		/**
		 * Returns the dimension sizes of interval.
		 */
		const Indices& dimensions() const {
			return dimensions_;
		}

		/**
		 * Returns the dimension size on coordinate d.
		 */
		Index dimension(size_t d) const {
			return dimensions_[d];
		}

		/**
		 * Returns the min values of indices in each coordinate.
		 */
		const Indices& min() const {
			return min_;
		}

		/**
		 * Returns the (included) max values of indices in each coordinate.
		 */
		Indices max() const {
			Indices ret;
			for (size_t d = 0; d < Dim; ++d) {
				ret[d] = min_[d] + dimensions_[d] - 1;
			}
			return ret;
		}

		/**
		 * Returns the first values of indices in each coordinate.
		 * It is the same as min(), but with a different name!
		 */
		const Indices& first() const {
			return min_;
		}

		/**
		 * Returns the (NOT included) last values of indices in each coordinate.
		 * It is the same as max() - 1.
		 */
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

		/**
		 * Returns the number of indices items included in the interval.
		 */
		size_t size() const {
			size_t s = 1;
			for (int d = 0; d < Dim; ++d) {
				s *= dimensions_[d];
			}
			return s;
		}

		/**
		 * Says if the given indices array is included in interval.
		 */
		bool inside(const Indices& indices) const {
			for (int d = 0; d < Dim; ++d) {
				if (indices[d] < min_[d] || indices[d] >= min_[d] + dimensions_[d]) {
					return false;
				}
			}
			return true;
		}

		/**
		 * Intersects this interval with another interval.
		 */
		ThisType intersect(const ThisType& interval) const {
			ThisType intersection;
			Indices intersMin, intersMax, intersDim;
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
			for (int d = 0; d < Dim; ++d) {
				if (intersection.dimensions_[d] < 0) {
					intersection.dimensions_[d] = 0;
				}
			}
			return intersection;
		}

		/**
		 * Returns a new interval built adding incr new cells along coordinate d.
		 * Example: let interval win=[3:5,1:3], then
		 *   win.stacked(0, 1) -> [6:6,1:3]
		 *   win.stacked(1, -2) ->  [3:5,-2:0]
		 * @param d dimension of stacked expansion
		 * @param incr number of added cell after (if incr>0) or before (if incr<0)
		 */
		ThisType stacked(size_t d, Index incr) const {
			ROFL_ASSERT_VAR2(d < Dim, d, Dim);
			ThisType expansion;
			Indices expandMin = min_;
			Indices expandDim = dimensions_;
			if (incr >= 0) {
				expandMin[d] += dimensions_[d];
				expandDim[d] = incr;
			} else {
				expandMin[d] += incr;
				expandDim[d] = -incr;
			}
			expansion.initBounds(expandMin, expandDim);
			return expansion;
		}

		/**
		 * Translates interval according to a vector parallel to dimension d.
		 * @param d dimension of translation
		 * @param incr number of added cell after (if incr>0) or before (if incr<0)
		 */
		void translate(size_t d, Index incr) {
			ROFL_ASSERT_VAR2(d < Dim, d, Dim);
			min_[d] += incr;
		}

		/**
		 * Translates interval according to the given translation vector of indices.
		 * @param t translation vector
		 */
		void translate(const Indices& t) {
			for (size_t d = 0; d < Dim; ++d) {
				min_[d] += t[d];
			}
		}

		/**
		 * Returns the iterator to first position of iterator.
		 * The RasterIteratorType visits the interval in raster order where
		 * index 0 of indices is the inner index and index Dim-1 is the outer one.
		 */
		RasterIteratorType beginRaster() const {
			return RasterIteratorType(this, 0);
		}

		/**
		 * Returns the iterator to last position of iterator.
		 * The RasterIteratorType visits the interval in raster order where
		 * index 0 of indices is the inner index and index Dim-1 is the outer one.
		 */
		RasterIteratorType endRaster() const {
			return RasterIteratorType(this, this->size());
		}

		/**
		 * Returns the iterator to first position of iterator.
		 * The RasterIteratorType visits the interval in boustrophedron order.
		 */
		BoustrophedonIteratorType beginBoustrophedon() const {
			return BoustrophedonIteratorType(this, 0);
		}

		/**
		 * Returns the iterator to last position of iterator.
		 * The RasterIteratorType visits the interval in boustrophedron order.
		 */
		BoustrophedonIteratorType endBoustrophedon() const {
			return BoustrophedonIteratorType(this, this->size());
		}

//		template <size_t D,typename I>
//		friend std::ostream& operator<<(std::ostream& out, const IntervalIndices<D,I>& interval) {
		void print(std::ostream&out) const {
			out << "[";
			for (int d = 0; d < Dim; ++d) {
				out << min()[d] << ":" << max()[d];
				if (d < Dim - 1)
					out << ",";
			}
			out << "]";
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
				if (!interval->empty()) {
					indices_ = IndexerType::getIndices(interval_->min(), interval_->dimensions(), pos);
				}
				else {
					indices_ = interval_->min();
				}
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

	}  // end of namespace detail

	template <typename I,size_t D>
	std::string outIdx(const std::array<I,D>& indices) {
		std::stringstream out;
		out << "[";
		for (int d = 0; d < D; ++d) {
			out << indices[d];
			if (d < D - 1)
				out << ",";
		}
		out << "]";
		return out.str();
	}

} // end of namespace rofl

template <size_t D,typename I>
std::ostream& operator<<(std::ostream& out, const rofl::IntervalIndices<D,I>& interval) {
	interval.print(out);
	return out;
}

#endif /* COMMON_INCLUDE_ROFL_COMMON_INTERVAL_INDICES_H_ */

