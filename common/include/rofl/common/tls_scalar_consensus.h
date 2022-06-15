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
#ifndef ROFL_COMMON_TLS_SCALAR_CONSENSUS_H_
#define ROFL_COMMON_TLS_SCALAR_CONSENSUS_H_

#include <rofl/common/macros.h>

namespace rofl {

/**
 * @brief Given a collection of scalar values, it computes the value with maximum consensus,
 *  i.e. the one where more items are concentrated.
 *  Each value is characterized by a range (tolerance) for it being an inlier.
 *
 * @tparam It
 * @param valueBeg
 * @param valueEnd
 * @param rangeBeg
 * @param rangeEnd
 * @param valueMax
 * @param inliers
 */
template <typename It>
void estimateTLSEstimation(It valueBeg, It valueEnd, It rangeBeg, It rangeEnd, typename It::value_type& valueMax, std::vector<bool>& inliers) {
    using Scalar = typename It::value_type;
    size_t n = std::distance(valueBeg, valueEnd);
    size_t nr = std::distance(rangeBeg, rangeEnd);
    size_t ne = 2 * n;

    ROFL_ASSERT_VAR2(n == nr, n, nr);
    inliers.resize(n);

    // Creates a sorted list of endpoints
    std::vector<std::pair<Scalar, int>> endpoints;
    endpoints.reserve(ne);
    auto itV = valueBeg;
    auto itR = rangeBeg;
    for (int i = 0; i < n && itV != valueEnd && itR != rangeEnd; ++i, ++itV, ++itR) {
        endpoints.push_back(std::make_pair(*itV + *itR, i + 1));
        endpoints.push_back(std::make_pair(*itV - *itR, -i - 1));
    }
    std::sort(endpoints.begin(), endpoints.end(), [](const std::pair<Scalar, int>& a, const std::pair<Scalar, int>& b) -> bool { return a.first < b.first; });

    std::vector<Scalar> weights;
    weights.reserve(n);
    for (auto it = rangeBeg; it != rangeEnd; ++it) {
        if (std::abs(*it) < 1e-6) {
            weights.push_back(1e+12);
        } else {
            weights.push_back(1.0 / ((*it) * (*it)));
        }
    }
    std::vector<Scalar> valueHat(n);
    std::vector<Scalar> valueCost(n);
    Scalar dot_weights_consensus = 0;
    int consensus_set_cardinal = 0;
    Scalar sum_xi = 0;
    Scalar sum_xi_square = 0;
    Scalar dot_X_weights;
    Scalar ranges_inverse_sum;

    ROFL_VAR3(valueHat.size(), valueCost.size(), inliers.size());

    for (size_t i = 0; i < ne; ++i) {
        int idx = int(std::abs(endpoints.at(i).second)) - 1;  // Indices starting at 1
        auto itV = valueBeg;
        auto itR = rangeBeg;
        std::advance(itV, idx);
        std::advance(itR, idx);
        int incr = (endpoints.at(i).second > 0) ? 1 : -1;

        consensus_set_cardinal += incr;
        dot_weights_consensus += incr * weights[idx];
        dot_X_weights += incr * weights[idx] * (*itV);
        ranges_inverse_sum -= incr * (*itR);
        sum_xi += incr * (*itV);
        sum_xi_square += incr * (*itV) * (*itV);

        valueHat.at(idx) = dot_X_weights / dot_weights_consensus;

        Scalar residual = consensus_set_cardinal * valueHat[i] * valueHat[i] + sum_xi_square - 2 * sum_xi * valueHat[i];
        valueCost.at(idx) = residual + ranges_inverse_sum;
    }

    auto itMin = std::min_element(valueCost.begin(), valueCost.end());
    auto itMax = valueBeg;
    std::advance(itMax, std::distance(valueCost.begin(), itMin));
    valueMax = *itMax;

    auto itInlier = inliers.begin();
    auto itValue = valueBeg;
    auto itRange = rangeBeg;
    for (; itInlier != inliers.end() && itValue != valueEnd && itRange != rangeEnd; ++itInlier, ++itValue, ++itRange) {
        *itInlier = fabs(*itValue - valueMax) < *itRange;
    }
    ROFL_MSG("estimated " << valueMax);
}

}  // namespace rofl

#endif