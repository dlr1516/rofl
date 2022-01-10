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
#ifndef ROFL_COMMON_MORTON_SORT_H_
#define ROFL_COMMON_MORTON_SORT_H_

#include <limits>
#include <rofl/common/bit_manip.h>
#include <rofl/common/numeric_traits.h>

namespace rofl {
    // ---------------------------------------------------------------
    // MORTON BIT OPERATION ON INTEGER SCALAR
    // ---------------------------------------------------------------

    /**
     * MSB: Most Significant Bit comparison. 
     * Given two integers i1 and i2, it computes if floor(log2(i1)) < floor(log2(i2))
     * using the well known Timothy Chan's trick illustrated in 
     *   
     * T.M. Chan, "A minimalist's implementation of an approximate nearest 
     * neighbor algorithm in fixed dimensions", 2006
     * http://tmc.web.engr.illinois.edu/pub_ann.html
     * 
     * E.g. i1 = 8 = 1000b, i2 = 11 = 1011b, i1 ^ i2 = 0011
     *    i1 < i2 -> TRUE, i1 = 1000b < 
     * 
     * @param i1 the first integer
     * @param i2 the second integer
     * @return 
     */
    template <typename Integer>
    bool msb(const Integer& i1, const Integer& i2) {
        return (i1 < i2 && i1 < (i1 ^ i2));
    }

    /**
     * Computes the smallest power 2 interval [low, upp[ that contains
     * the input interval [i1, i2[ and the middle value mid of [low, upp[.
     *
     * Example: i1 = 8 (b00001000), i2 = 11 (b00001011) as int8_t
     *   level = log2(i1 ^ i2) = log2(b00000111) = 3
     *   (log2() can be computed as bitnum - nlz...)
     *
     *   intervMask = 2^level - 1 = b00000111
     *  ~intervMask = b11111000
     *   low = 8
     *
     * @param i1 the first integer
     * @param i2 the second integer
     * @param low the lowest integer 2^level
     */
    template <typename I>
    int intervalPow2Int(const I& i1, const I& i2, I& low, I& mid, I& upp) {
    	using IT = IntegerTraits<I>;
    	using IntegerType = typename IT::IntegerType;
    	using UnsignedType = typename IT::UnsignedType;
    	IntegerType level, intervMask;

    	level = IT::BIT_NUM - IT::nlz(IT::removeSign(i1) ^ IT::removeSign(i2));
    	if (level < IT::BIT_NUM) {
    		intervMask = (1 << level) - 1;
    		low = i1 & (~intervMask);
    		upp = (low | intervMask);
    		mid = low | (1 << (level - 1));
    	} else {
    		intervMask = ~0;
    		low = std::numeric_limits<I>::min();
    		upp = std::numeric_limits<I>::max();
    		mid = (upp + low) / 2;
    	}
    	return (int)level;
    }

    /**
     * Compares two vectors of integer values with integer type I and
     * dimension Dim and sort them according to Morton order.
     * It is based on the so called Chan's trick:
     *
     * T.M. Chan, "A minimalist's implementation of an approximate nearest
     * neighbor algorithm in fixed dimensions", 2006
     * http://tmc.web.engr.illinois.edu/pub_ann.html
     *
     * @param v1 the first integer vector
     * @param v2 the second integer vector
     * @return true if v1 is before v2 in Morton Order.
     */
    template <typename I, int Dim>
    bool mortonCmpInt(const I* v1, const I* v2) {
    	using IT = IntegerTraits<I>;
    	using UnsignedType = typename IT::UnsignedType;
    	UnsignedType lastDim, lastXor, currXor;
    	lastDim = 0;
    	lastXor = IT::removeSign(v1[0]) ^ IT::removeSign(v2[0]);
    	for (int d = 1; d < Dim; ++d) {
    		currXor = IT::removeSign(v1[d]) ^ IT::removeSign(v2[d]));
    		if (msb<UnsignedType>(lastXor, currXor)) {
    			lastDim = d;
    			lastXor = currXor;
    		}
    	}
    	return (v1[lastDim] < v2[lastDim]);
    }

    template <typename I, int Dim>
    int mortonDistanceInt(const I* v1, const I* v2) {
    	using IT = IntegerTraits<I>;
    	using UnsignedType = typename IT::UnsignedType;
    	typename IT::IntegerType levelMax, level;
    	levelMax = 0;
    	for (int d = 0; d < Dim; ++d) {
    		level = IT::BIT_NUM - IT::nlz(IT::removeSign(v1[d]) ^ IT::removeSign(v2[d]));
    		if (level > levelMax) {
    			levelMax = level;
    		}
    	}
    	return levelMax;
    }

    template <typename I, int Dim>
    void mortonSplitInt(const I* v1, const I* v2, I* low, I* mid, I* upp) {
    	int dimSplit, levelSplit, level;

    	dimSplit = 0;
    	levelSplit = intervalPow2(v1[0], v2[0], low[0], mid[0], upp[0]);
    	for (int d = 1; d < Dim; ++d) {
    		level = intervalPow2(v1[d], v2[d], low[d], mid[d], upp[d]);
    		if (level > levelSplit) {
    			mid(dimSplit) = low(dimSplit);
    			dimSplit = d;
    			levelSplit = level;
    		}
    		else {
    			mid[d] = low[d];
    		}
    	}
    }

}


#endif 
