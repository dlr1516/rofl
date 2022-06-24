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

template <typename Scalar>
void estimateTLSEstimation2(const std::vector<Scalar>& values, const std::vector<Scalar>& ranges, Scalar& valueMax, std::vector<bool>& inliers) {
    size_t n = values.size();
    size_t ne = 2 * n;

    ROFL_ASSERT_VAR2(n == ranges.size(), n, ranges.size());
    inliers.resize(n);
    std::vector<Scalar> weights(n);
    std::vector<Scalar> valueHat(ne);
    std::vector<Scalar> valueCost(ne);

    // Creates a sorted list of endpoints
    std::vector<std::pair<Scalar, int>> endpoints;
    endpoints.reserve(ne);
    for (size_t i = 0; i < n; ++i) {
        endpoints.push_back(std::make_pair(values[i] - ranges[i], i + 1));
        endpoints.push_back(std::make_pair(values[i] + ranges[i], -i - 1));
    }
    std::sort(endpoints.begin(), endpoints.end(),
              [](const std::pair<Scalar, int>& a, const std::pair<Scalar, int>& b) -> bool {
                  return a.first < b.first;
              });

    // Computes the weights of each value i as the inverse square of its range
    for (size_t i = 0; i < n; ++i) {
        if (std::abs(ranges[i]) < 1e-6) {
            weights[i] = 1e+12;
        } else {
            weights[i] = 1.0 / (ranges[i] * ranges[i]);
        }
        ROFL_VAR2(i, weights[i]);
    }

    Scalar consensusWeighted = 0;
    int consensusSetCardinal = 0;
    Scalar sumXi = 0;
    Scalar sumXiSquare = 0;
    Scalar valueWeighted = 0;
    Scalar rangesInverseSum = 0;

    for (size_t i = 0; i < ranges.size(); ++i) {
        rangesInverseSum += ranges[i];
    }

    ROFL_VAR3(valueHat.size(), valueCost.size(), inliers.size());

    // The algorithm visits the endpoints of intervals [value-range, value+range].
    // Given a value v, the consensus cardinality in v is the number of intervals
    // containing v.
    // Cardinality is tracked adding -1 when an interval starts and +1 when it ends.
    for (size_t j = 0; j < ne; ++j) {
        int i = int(std::abs<int>(endpoints[j].second)) - 1;  // Indices starting at 1
        int incr = (endpoints[j].second > 0) ? 1 : -1;

        consensusSetCardinal += incr;
        consensusWeighted += incr * weights[i];
        valueWeighted += incr * weights[i] * values[i];
        rangesInverseSum -= incr * ranges[i];
        sumXi += incr * values[i];
        sumXiSquare += incr * values[i] * values[i];

        ROFL_VAR1(consensusWeighted);

        valueHat[j] = valueWeighted / consensusWeighted;

        Scalar residual = consensusSetCardinal * valueHat[j] * valueHat[j] + sumXiSquare - 2 * sumXi * valueHat[j];
        valueCost[j] = residual + rangesInverseSum;

        for (size_t s = 0; s < std::abs(consensusSetCardinal); ++s) {
            std::cout << " ";
        }
        std::cout << i << " (" << j << "): "
                  << "endpoint " << endpoints[j].first
                  << ", value " << values[i]
                  << ", valueHat " << valueHat[j]
                  << ", valueCost " << valueCost[j]
                  << ", consensusSetCardinal " << consensusSetCardinal
                  << std::endl;
    }

    auto itMin = std::min_element(valueCost.begin(), valueCost.end());
    ROFL_VAR2(std::distance(valueCost.begin(), itMin), *itMin);
    valueMax = valueHat.at(std::distance(valueCost.begin(), itMin));

    for (size_t i = 0; i < n; ++i) {
        inliers[i] = fabs(values[i] - valueMax) < ranges[i];
    }
    ROFL_MSG("estimated " << valueMax);
}

}  // namespace rofl

#endif