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
 * @brief Estimates the value with a maximum consensus from a set of values
 * each provided with a range of acceptance.
 * This function follows the Truncated Least Square (TLS) formulation from
 * Teaser registration tool.
 *
 * H. Yang, J. Shi, and L. Carlone, "TEASER: Fast and Certifiable Point Cloud Registration,"
 * IEEE Trans. Robotics, 37(2), pp. 314-333, April 2021, doi: 10.1109/TRO.2020.3033695.
 *
 * The present implementation is an adaptation of software project
 * https://github.com/MIT-SPARK/TEASER-plusplus
 *
 * @tparam Scalar the scalar type (it must be a floating point like float or double)
 * @param values std::vector of scalar values
 * @param ranges std::vector of scalar ranges (tolerance threshold for inliers)
 * @param valueMax value with maximum consensus
 * @param inliers std::vector to mark inlier values
 */
template <typename Scalar>
void estimateTranslationTls(const std::vector<Scalar>& values, const std::vector<Scalar>& ranges, Scalar& valueMax, std::vector<bool>& inliers) {
    size_t n = values.size();
    size_t ne = 2 * n;

    ROFL_ASSERT_VAR2(n == ranges.size(), n, ranges.size());
    inliers.resize(n);
    std::vector<Scalar> weights(n);
    std::vector<Scalar> valueHat(ne, 0);
    std::vector<Scalar> valueCost(ne, 0);

    // Creates a sorted list of endpoints
    std::vector<std::pair<Scalar, int>> endpoints;
    // endpoints.reserve(ne);
    for (size_t i = 0; i < n; ++i) {
        endpoints.push_back(std::make_pair(values[i] - ranges[i], i + 1));
        endpoints.push_back(std::make_pair(values[i] + ranges[i], -i - 1));
    }
    std::sort(endpoints.begin(), endpoints.end(),
              [](const std::pair<Scalar, int>& a, const std::pair<Scalar, int>& b) -> bool {
                  return a.first < b.first;
              });

    // Computes the weights of each value i as the inverse square of its range
    for (size_t i = 0; i < ranges.size(); ++i) {
        if (std::abs(ranges[i]) < 1e-6) {
            weights[i] = 1e+12;
        } else {
            weights[i] = 1.0 / (ranges[i] * ranges[i]);
        }
        // ROFL_VAR2(i, weights[i]);
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

    // ROFL_VAR3(valueHat.size(), valueCost.size(), inliers.size());

    // The algorithm visits the endpoints of intervals [value-range, value+range].
    // Given a value v, the consensus cardinality in v is the number of intervals
    // containing v.
    // Cardinality is tracked adding -1 when an interval starts and +1 when it ends.
    for (size_t j = 0; j < ne; ++j) {
        int i = int(std::abs(endpoints[j].second)) - 1;  // Indices starting at 1
        int incr = (endpoints[j].second > 0) ? 1 : -1;

        consensusSetCardinal += incr;
        consensusWeighted += incr * weights[i];
        valueWeighted += incr * weights[i] * values[i];
        rangesInverseSum -= incr * ranges[i];
        sumXi += incr * values[i];
        sumXiSquare += incr * values[i] * values[i];

        // ROFL_VAR1(consensusWeighted);

        valueHat[j] = valueWeighted / consensusWeighted;

        Scalar residual = consensusSetCardinal * valueHat[j] * valueHat[j] + sumXiSquare - 2 * sumXi * valueHat[j];
        valueCost[j] = residual + rangesInverseSum;

        // for (size_t s = 0; s < std::abs(consensusSetCardinal); ++s) {
        //     std::cout << " ";
        // }

        // std::cout << i << " (" << j << "): "
        //           << "endpoint " << endpoints[j].first
        //           << ", value " << values[i]
        //           << ", valueHat " << valueHat[j]
        //           << ", valueCost " << valueCost[j]
        //           << ", consensusSetCardinal " << consensusSetCardinal
        //           << ", sumXiSquare " << sumXiSquare
        //           << ", sumXi " << sumXi
        //           << std::endl;
    }

    auto itMin = std::min_element(valueCost.begin(), valueCost.end());
    ROFL_VAR2(std::distance(valueCost.begin(), itMin), *itMin);
    valueMax = valueHat.at(std::distance(valueCost.begin(), itMin));

    for (size_t i = 0; i < n; ++i) {
        inliers[i] = fabs(values[i] - valueMax) < ranges[i];
    }
    // ROFL_MSG("estimated " << valueMax);
}

}  // namespace rofl

#endif