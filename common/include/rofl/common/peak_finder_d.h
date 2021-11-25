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
#ifndef ROFL_COMMON_PEAK_FINDER_D_H_
#define ROFL_COMMON_PEAK_FINDER_D_H_

#include <functional>
#include <vector>
#include <queue>
#include <rofl/common/macros.h>
#include <rofl/common/interval_indices.h>
#include <rofl/common/min_max_heap.h>

namespace rofl {

	/**
	 * Class PeakFinderND finds the maxima over histograms.
	 * It has the template parameters:
	 * - Dim: dimension of space;
	 * - Index: the type of the histogram index, which is usually an integer;
	 * - Value: the type of the histogram value;
	 * - ValueComp: the comparator to sort histogram values (default comparator finds maxima).
	 * You can set the following parameters:
	 * - the index interval;
	 * - the window size, i.e. the minimum distance between two maxima in order
	 *   to suppress too close local maxima due to random noise;
	 * - the minimum required value threshold for candidate maxima.
	 *
	 * How to use it in an example:
	 *
	 *
	 */
	template <size_t Dim,typename Value,typename Index = int,typename ValueComp = std::less<Value> >
	class PeakFinderD {
	public:
		using ThisType = PeakFinderD<Dim, Value, Index, ValueComp>;
		using IntervalType = IntervalIndices<Dim, Index>;
		using Indices = typename IntervalType::Indices;

		static const size_t DIM = Dim;

		/**
		 * Default constructor.
		 */
		PeakFinderD() : comp_(), domain_(), iwin_(), valueMin_(0), enableFilterMinValue_(false) {
			std::fill(iwin_.begin(), iwin_.end(), 0);
		}

		/**
		 * Constructor with
		 */
		PeakFinderD(const ValueComp& comp) : comp_(comp), domain_(), iwin_(), valueMin_(0), enableFilterMinValue_(false) {
			std::fill(iwin_.begin(), iwin_.end(), 0);
		}

		virtual ~PeakFinderD() {

		}

		void setDomain(const Indices& imin, const Indices& imax) {
			domain_.initMinMax(imin, imax);
		}

		void setDomain(const Indices& isize) {
			Indices zeros;
			zeros.fill(0);
			domain_.initBounds(zeros, isize);
		}

		void setDomainCentered(const Indices& icenter, const Indices& isize) {
			domain_.initCentered(icenter, isize);
		}

		void setPeakWindow(const Indices& iwin) {
			iwin_ = iwin;
		}

		void enableFilterMinValue(bool on, const Value& valueMin) {
			enableFilterMinValue_ = on;
			valueMin_ = valueMin;
		}

		template <typename Map,typename InsertIt>
		void detect(const Map& map, InsertIt insertMax) {
			// Priority queue storing the item in a given interval window sorted
			// according to the order set by comparator comp_() (peaks first!)
			// NOTE: in the discussion of the examples, we assume that comp_ = std::greater<>,
			// i.e. the examples are about finding maxima
			auto indicesCmp = [&](const Indices& indices0, const Indices& indices1) -> bool {
				return comp_(map(indices0), map(indices1));
			};
			//std::priority_queue<Indices,std::vector<Indices>,decltype(indicesCmp)> windowItems(indicesCmp);
			MinMaxHeap<Indices,decltype(indicesCmp)> windowItems(indicesCmp);
			Indices indicesPrev;
			IntervalType windowInterval, novelInterval;
			int dimMax, incrMax;

			// Initializes the window
			auto domainIt = domain_.beginBoustrophedon();
			auto domainEnd = domain_.endBoustrophedon();
			windowInterval.initCentered(*domainIt, iwin_);
			novelInterval = windowInterval.intersect(domain_);
			for (auto it = novelInterval.beginRaster(); it != novelInterval.endRaster(); ++it) {
				windowItems.push(*it);
			}
			ROFL_VAR4(*domainIt, windowInterval, novelInterval, windowItems.size());
			indicesPrev = *domainIt;
			++domainIt;

			// Moves the window on each cell of domain in boustrophedon order
			for (; domainIt != domainEnd; ++domainIt) {
				// Finds the interval not included in previous
				findIncrement(indicesPrev, *domainIt, dimMax, incrMax);
				novelInterval = windowInterval.stacked(dimMax, incrMax).intersect(domain_);
				//ROFL_VAR3(*domainIt, dimMax, incrMax);
				//ROFL_VAR4(novelInterval, novelInterval.empty(), novelInterval.size(), novelInterval.dimensions());
				for (auto it = novelInterval.beginRaster(); it != novelInterval.endRaster(); ++it) {
					windowItems.push(*it);
				}
				windowInterval.translate(dimMax, incrMax);
				//ROFL_VAR4(*domainIt, windowInterval, novelInterval, windowItems.size());

				// Extracts from the window queue the top items that are not in the durrect window
				while (!windowItems.empty() && !windowInterval.inside(windowItems.top())) {
					windowItems.popTop();
				}

				// Checks if current cell *domainIt is not dominated by the top item windowItems.top().
				// E.g. if comp_ = std::greater<>, then
				//   !comp_(map(windowItems.top()), map(*domainIt))) -> map(*domainIt) >= map(windowItems.top()
				if (!enableFilterMinValue_ || comp_(map(*domainIt), valueMin_)) {
					if (!windowItems.empty() && !comp_(map(windowItems.top()), map(*domainIt))) {
						Indices curTop = windowItems.top();
						windowItems.popTop();
						if (comp_(map(*domainIt), map(windowItems.top()))) {
						//	ROFL_VAR6(*domainIt, map(*domainIt), curTop, map(curTop), windowItems.top(), map(windowItems.top()));
							// Indices *domainIt is added only if there is no minium filter enabled or
							// map(*domainIt) is greater than valueMin_
							insertMax = *domainIt;
						}
						windowItems.push(curTop);
					}
				}
				indicesPrev = *domainIt;
			}
		}

	private:
		ValueComp comp_;
		IntervalType domain_;
		Indices iwin_;
		Value valueMin_;
		bool enableFilterMinValue_;

		void findIncrement(const Indices& iprev, const Indices& icurr, int& dimMax, int& incrMax) {
			int incr;
			dimMax = 0;
			incrMax = 0;
			for (int d = 0; d < Dim; ++d) {
				incr = icurr[d] - iprev[d];
				if (std::abs(incr) > std::abs(incrMax)) {
					dimMax = d;
					incrMax = incr;
				}
			}
		}

	};

} // end of namespace rofl

#endif /* COMMON_INCLUDE_ROFL_COMMON_PEAK_FINDER_D_H_ */
