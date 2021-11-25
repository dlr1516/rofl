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
#ifndef ROFL_COMMON_GRID_H_
#define ROFL_COMMON_GRID_H_

#include <vector>
#include <rofl/common/interval_indices.h>

namespace rofl {

	template <size_t Dim,typename Value,typename Index = int,typename Indexer = detail::RasterIndexer<Dim, Index>,
			template <typename,typename > class Container = std::vector,template <typename > class Allocator = std::allocator>
	class Grid {
	public:
		using Type = Grid<Dim, Value, Index, Indexer, Container, Allocator>;
		using Storage = Container<Value, Allocator<Value> >;
		using IntervalType = IntervalIndices<Dim, Index>;
		using Indices = typename IntervalType::Indices;
		//using IndexerType = Indexer<Dim, Index>;  // if "template <size_t,typename > class Indexer"
		using IndexerType = Indexer;

		Grid() : data_(), domain_() {
		}

		Grid(const Indices& min, const Indices& dimensions) : data_(), domain_(min, dimensions) {
			data_.resize(domain_.size());
		}

		Grid(const Indices& dimensions) : data_(), domain_() {
			Indices zeros;
			zeros.fill(0);
			domain_.initBounds(zeros, dimensions);
			data_.resize(domain_.size());
		}

		virtual ~Grid() {
		}

		void initBounds(const Indices& dimensions) {
			Indices zeros;
			zeros.fill(0);
			domain_.initBounds(zeros, dimensions);
			data_.resize(domain_.size());
		}

		void initBounds(const Indices& min, const Indices& dimensions) {
			domain_.initBounds(min, dimensions);
			data_.resize(domain_.size());
		}

		void initCenter(const Indices& icenter, const Indices& iwin) {
			domain_.initCentered(icenter, iwin);
			data_.resize(domain_.size());
		}

		void initMinMax(const Indices& imin, const Indices& imax) {
			domain_.initMinMax(imin, imax);
			data_.resize(domain_.size());
		}

		size_t size() const {
			return data_.size();
		}

		const Indices& dimensions() const {
			return domain_.dimensions();
		}

		const Value& value(const Indices& indices) const {
			Index pos = getPos(indices);
			return data_.at(pos);
		}

		Value& value(const Indices& indices) {
			Index pos = getPos(indices);
			return data_.at(pos);
		}

		const Value& value(const Index& pos) const {
			return data_.at(pos);
		}

		Value& value(const Index& pos) {
			return data_.at(pos);
		}

		void fill(const Value& value) {
			std::fill(std::begin(data_), std::end(data_), value);
		}

		Index getPos(const Indices& indices) const {
			return IndexerType::getPos(domain_.min(), domain_.dimensions(), indices);
		}

		Indices getIndices(const Index& index) const {
			return IndexerType::getIndices(domain_.min(), domain_.dimensions(), index);
		}

	protected:
		Storage data_;
		IntervalType domain_;
	};

} // end of namespace rofl

#endif /* COMMON_INCLUDE_ROFL_COMMON_MULTI_GRID_H_ */
