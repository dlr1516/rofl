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
    // MORTON BIT OPERATION ON INTEGER OR FLOATING POINT SCALARS
    // ---------------------------------------------------------------

    /**
     * MSB: Most Significant Bit comparison. 
     * Given two integers i1 and i2, it says if floor(log2(i1)) < floor(log2(i2))
     * using the well known Timothy Chan's trick illustrated in 
     *   
     * T.M. Chan, "A minimalist's implementation of an approximate nearest 
     * neighbor algorithm in fixed dimensions", 2006
     * http://tmc.web.engr.illinois.edu/pub_ann.html
     * 
     * Note: the conditions to be simultaneously checked to return true are:
     * a) i1 < i2: trivial to see that  floor(log2(i1)) < floor(log2(i2));
     * b) i1 < i1 ^ i2: it holds if the most significant bit of i1 is not the same of i2
     *    (bitwise XOR ^ sets to 1 only the bits that are different!)
     *
     *  e.g. i1 = 1000b, i2 = 1010b -> i1 ^ i2 = 0010b, 1000b < 0010b ? FALSE
     *       i1 = 0101b, i2 = 1001b -> i1 ^ i2 = 1100b, 0101b < 1100b ? TRUE
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
        return (int) level;
    }

    template <typename F>
    F computeBitDiff(const F& f1, const F& f2) {
        typename FloatTraits<F>::IntegerType xm, xe;
        return computeBitDiff<F>(f1, f2, xm, xe);
    }

    template <typename F>
    F computeBitDiff(const F& f1, const F& f2, typename FloatTraits<F>::IntegerType& mantissaDiff, typename FloatTraits<F>::IntegerType& exponentDiff) {
        using FT = FloatTraits<F>;
        using IntegerType = typename FT::IntegerType;
        using UnsignedType = typename FT::UnsignedType;
        IntegerType fm1, fm2, fe1, fe2, msbPos;
        bool fs1, fs2;

        FT::decompose(f1, fm1, fe1, fs1);
        FT::decompose(f2, fm2, fe2, fs2);

        // If the input numbers have different signs, then the returned value
        // is the infinity. 
        if (fs1 ^ fs2) {
            mantissaDiff = 0;
            exponentDiff = FT::EXPONENT_MASK - FT::EXPONENT_BIAS;
            return FT::compose(mantissaDiff, exponentDiff, false);
        }

        // f1 must be the maximum of the two
        if (fabs(f1) < fabs(f2)) {
            std::swap(f1, f2);
            std::swap(fe1, fe2);
            std::swap(fm1, fm2);
        }

        // Adding the implicit most significant bit to the two mantissas
        fm1 = fm1 | FT::MANTISSA_IMPLICIT_BIT;
        if (fe1 < fe2 + FT::BIT_NUM) {
            fm2 = (fm2 | FT::MANTISSA_IMPLICIT_BIT) >> (fe1 - fe2);
        } else {
            fm2 = 0;
        }
        // Changes the mantissas sign, if f1 and f2 are both negatives
        if (fs1) {
            fm1 = -fm1;
            fm2 = -fm2;
        }
        mantissaDiff = fm1 ^ fm2;

        if (mantissaDiff == 0) {
            exponentDiff = -FT::EXPONENT_BIAS;
            return FT::compose(mantissaDiff, exponentDiff, false);
        }
        msbPos = FT::MANTISSA_BITS - rofl::IntegerTraits<IntegerType>::log2Mod(mantissaDiff);
        exponentDiff = fe1 - msbPos;
        mantissaDiff = (mantissaDiff << msbPos) & FT::MANTISSA_MASK;

        return FT::compose(mantissaDiff, exponentDiff, false);
    }

    // ---------------------------------------------------------------
    // MORTON OPERATION ON ARRAYS/VECTORS OF SCALARS
    // ---------------------------------------------------------------

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
            currXor = IT::removeSign(v1[d]) ^ IT::removeSign(v2[d]);
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
            } else {
                mid[d] = low[d];
            }
        }
    }
    
    /**
     * Compares two vectors of floating point values with floating point type F 
     * and dimension Dim and sort them according to Morton order.
     * It follows the implementation of Morton order comparator for integer,
     * but taking into account the mantissa and exponent parts of floating 
     * point representation.
     *
     *
     * @param v1 the first floating point vector
     * @param v2 the second floating point vector
     * @return true if v1 is before v2 in Morton Order.
     */
    template <typename F, int Dim>
    bool mortonCmpFloat(const F* v1, const F* v2) {
        using FT = FloatTraits<F>;
        using IntegerType = typename FT::IntegerType;
        using UnsignedType = typename FT::UnsignedType;
        IntegerType currMantissa, currExponent, lastMantissa, lastExponent;
        int lastDim;
        F currXor, lastXor;

        lastDim = 0;
        lastXor = computeBitDiff(v1[0], v2[0], lastMantissa, lastExponent);
        for (int d = 1; d < Dim; ++d) {
            currXor = computeBitDiff(v1[d], v2[d], currMantissa, currExponent);
            if (lastXor < currXor && lastExponent < currExponent) {
                lastDim = d;
                lastExponent = currExponent;
                lastXor = currXor;
            }
        }
        return (v1[lastDim] < v2[lastDim]);
    }
    
    template <typename F, int Dim>
    int mortonDistanceFloat(const F* v1, const F* v2) {
        using FT = FloatTraits<F>;
        using IntegerType = typename FT::IntegerType;
        using UnsignedType = typename FT::UnsignedType;
        IntegerType mantissa, exponent, exponentMax;
        exponentMax = -FT::EXPONENT_BIAS;
        for (int d = 0; d < Dim; ++d) {
            computeBitDiff(v1[d], v2[d], mantissa, exponent);
            if (exponent > exponentMax) {
                exponentMax = exponent;
            }
        }
        return exponentMax;
    }
    
    // ---------------------------------------------------------------
    // MORTON TRAITS
    // ---------------------------------------------------------------
    
    template <typename Scalar, size_t Dim, typename Enable = void> 
    struct MortonTraits;
    
    
    template <typename Scalar, size_t Dim> 
    struct MortonTraits<Scalar, Dim, std::enable_if<std::is_integral_v<Scalar> > > {
        
        static bool compare(const Scalar* v1, const Scalar* v2) {
            return mortonCmpInt<Scalar, Dim>(v1, v2);
        }
        
        static int distance(const Scalar* v1, const Scalar* v2) {
            return mortonDistanceInt<Scalar, Dim>(v1, v2);
        }
        
        static void split(const Scalar* v1, const Scalar* v2, Scalar* low, Scalar* mid, Scalar* upp) {
            mortonSplitInt<Scalar, Dim>(v1, v2, low, mid, upp);
        }
    };
    
    template <typename Scalar, size_t Dim> 
    struct MortonTraits<Scalar, Dim, std::enable_if<std::is_floating_point_v<Scalar> > > { 
    
        static bool compare(const Scalar* v1, const Scalar* v2) {
            return mortonCmpFloat<Scalar, Dim>(v1, v2);
        }
        
        static int distance(const Scalar* v1, const Scalar* v2) {
            return mortonDistanceFloat<Scalar, Dim>(v1, v2);
        }
    };
    

}


#endif 
