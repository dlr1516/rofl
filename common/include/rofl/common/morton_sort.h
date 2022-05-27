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

#include <rofl/common/bit_manip.h>
#include <rofl/common/macros.h>
#include <rofl/common/numeric_traits.h>

#include <algorithm>  // std::swap()
#include <cmath>      // fabs()
#include <limits>
#include <type_traits>  // std::is_integral<>, std::is_floating_point<>

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
 * b) i1 < i1 ^ i2: it holds if the most significant bit of i1 is not the same
 * of i2 (bitwise XOR ^ sets to 1 only the bits that are different!)
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
 * Computes the extreme and middle values of the smallest subtree
 * containing the given interval [i1, i2[.
 * A subtree of level k covers an interval of size 2^k.
 * The minimum level is given by:
 *
 *   level = BIT_NUM - nlz(i1 ^ i2)   (discretized log2)
 *   low = 2^level * floor(i1 / 2^level)
 *   upp = 2^level * ceil(i1 / 2^level)
 *   mid = (low + upp) / 2 = low + 2^(level - 1)
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

    // When i1 and i2 have different signs (their most significant bits are
    // different), then the most significant bit of i1^i2 is equal to 1,
    // nlz(i1^i2) == 0 and level == IT::BIT_NUM. In such case interval (low,
    // upp) span the whole integer range.
    // The ordinary case is explained by the following example with 6 bit
    // integers:
    //   i1 = 13:    00001101
    //   i2 = 9:     00001001
    //   i1^i2       00000100  -> nlz(i1^i2) = 5, level = 8 - 5 = 3
    //   intervMask  00000111
    //   low         00001000  -> 8
    //   upp         00001111  -> 15
    //   mid         00001100  -> 12
    //
    //   i1 = 14:    00001110
    //   i2 = 5:     00000101
    //   i1^i2       00001011  -> nlz(i1^i2) = 4, level = 8 - 4 = 4
    //   intervMask  00001111
    //   low         00000000  -> 0
    //   upp         00001111  -> 15
    //   mid         00001000  -> 8
    level = IT::BIT_NUM - IT::nlz(IT::removeSign(i1) ^ IT::removeSign(i2));
    if (level < IT::BIT_NUM) {
        intervMask = (1 << level) - 1;
        low = i1 & (~intervMask);
        upp = (low | intervMask);
        mid = low | (1 << (level - 1));
        // ROFL_VAR1(level);
        // ROFL_MSG("i1          " << std::bitset<IT::BIT_NUM>(i1));
        // ROFL_MSG("i2          " << std::bitset<IT::BIT_NUM>(i2));
        // ROFL_MSG("xor         " << std::bitset<IT::BIT_NUM>(IT::removeSign(i1) ^ IT::removeSign(i2)));
        // ROFL_MSG("intervMask  " << std::bitset<IT::BIT_NUM>(intervMask));
        // ROFL_MSG("low         " << std::bitset<IT::BIT_NUM>(low));
        // ROFL_MSG("mid         " << std::bitset<IT::BIT_NUM>(mid));
        // ROFL_MSG("upp         " << std::bitset<IT::BIT_NUM>(upp));
    } else {
        intervMask = ~0;
        low = std::numeric_limits<I>::min();
        upp = std::numeric_limits<I>::max();
        mid = (upp + low) / 2;
    }
    return (int)level;
}

template <typename F>
F xorFloat(const F& fin1,
           const F& fin2,
           typename FloatTraits<F>::IntegerType& mantissaDiff,
           typename FloatTraits<F>::IntegerType& exponentDiff) {
    using FT = FloatTraits<F>;
    using IntegerType = typename FT::IntegerType;
    using UnsignedType = typename FT::UnsignedType;
    F f1, f2;
    IntegerType fm1, fm2, fe1, fe2, msbPos;
    bool fs1, fs2;

    // Arranges the input floating s.t. fabs(f1) > fabs(f2)
    if (fabs(fin1) > fabs(fin2)) {
        f1 = fin1;
        f2 = fin2;
    } else {
        f1 = fin2;
        f2 = fin1;
    }

    FT::decompose(f1, fm1, fe1, fs1);
    FT::decompose(f2, fm2, fe2, fs2);

    // If the input numbers have different signs, then the returned value
    // is the infinity.
    if (fs1 ^ fs2) {
        mantissaDiff = 0;
        exponentDiff = FT::EXPONENT_MAX;
        //            ROFL_MSG("different signs\n"
        //                    << std::bitset<FT::BIT_NUM>(mantissaDiff) << " xm
        //                    " << mantissaDiff << "\n"
        //                    << std::bitset<FT::BIT_NUM>(exponentDiff) << " xe
        //                    " << exponentDiff << "\n");
        return FT::compose(mantissaDiff, exponentDiff, false);
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

template <typename F>
F xorFloat(const F& f1, const F& f2) {
    typename FloatTraits<F>::IntegerType xm, xe;
    return xorFloat<F>(f1, f2, xm, xe);
}

/**
 * @brief Computes the extreme and middle values of the smallest
 * subtree containing the given interval [i1, i2[.
 * A subtree of level k covers an interval of size 2^k.
 * The minimum level is given by:
 *
 * @tparam F the type of floating point number
 * @param fin1 first value of interval
 * @param fin2 second value of interval
 * @param low
 * @param mid
 * @param upp
 * @return int
 */
template <typename F>
int intervalPow2Float(const F& fin1, const F& fin2, F& low, F& mid, F& upp) {
    using FT = FloatTraits<F>;
    using IntegerType = typename FT::IntegerType;
    using UnsignedType = typename FT::UnsignedType;
    using IT = IntegerTraits<IntegerType>;
    F f1, f2, powLevel;
    IntegerType fm1, fm2, fe1, fe2, fm12, fe12;
    IntegerType intervMask, mlow, mmid, mupp;
    bool fs1, fs2, fs12;
    int level;

    // Extracts mantissa, exponent and sign from each term
    FT::decompose(fin1, fm1, fe1, fs1);
    FT::decompose(fin2, fm2, fe2, fs2);

    // If signs are different, the tree containing the two items
    // spans the whole floating point range
    if (fs1 ^ fs2) {
        low = std::numeric_limits<F>::lowest();
        upp = std::numeric_limits<F>::max();
        mid = 0;
        level = FT::EXPONENT_MAX + 1;
        return level;
    }

    // Arranges the input floating s.t. fabs(f1) < fabs(f2)
    f2 = fin1;
    f2 = fin2;
    if (fe1 > fe2 || (fe1 == fe2 && fm1 > fm2)) {
        std::swap(fm1, fm2);
        std::swap(fe1, fe2);
        std::swap(f1, f2);
    }
    fm1 = fm1 | FT::MANTISSA_IMPLICIT_BIT;
    fm2 = fm2 | FT::MANTISSA_IMPLICIT_BIT;
    fe12 = fe2 - fe1;
    // ROFL_MSG("fm1         " << std::bitset<FT::BIT_NUM>(fm1));
    // ROFL_MSG("fm2         " << std::bitset<FT::BIT_NUM>(fm2));
    // ROFL_MSG("fm1 >> fe12 " << std::bitset<FT::BIT_NUM>(fm1 >> fe12));
    // ROFL_MSG("xor         " << std::bitset<FT::BIT_NUM>(fm2 ^ (fm1 >> fe12)));
    // ROFL_VAR2(log2Mod(fm2 ^ (fm1 >> fe12)), FT::MANTISSA_BITS - log2Mod(fm2 ^ (fm1 >> fe12)));
    if (fe12 <= FT::MANTISSA_BITS) {
        level = IT::BIT_NUM - IT::nlz(fm2 ^ (fm1 >> fe12));
    } else {
        level = IT::BIT_NUM - IT::nlz(fm2);
    }
    ROFL_VAR4(fe1, fe2, fe12, level);
    intervMask = (1 << level) - 1;
    mlow = fm2 & (~intervMask);
    mupp = (mlow | intervMask);
    mmid = mlow | (1 << (level - 1));
    if (mlow != 0) {
        low = FT::compose(mlow ^ FT::MANTISSA_IMPLICIT_BIT, fe2, fs2);
    } else {
        low = FT::compose(mlow, FT::EXPONENT_MIN, fs2);
    }
    if (mmid != 0) {
        mid = FT::compose(mmid ^ FT::MANTISSA_IMPLICIT_BIT, fe2, fs2);
    } else {
        mid = FT::compose(mmid, FT::EXPONENT_MIN, fs2);
    }
    if (mupp != 0) {
        upp = FT::compose(mupp ^ FT::MANTISSA_IMPLICIT_BIT, fe2, fs2);
    } else {
        upp = FT::compose(mupp, FT::EXPONENT_MIN, fs2);
    }
    if (fs2) {
        std::swap(low, upp);
    }
    level = fe2 + level - FT::MANTISSA_BITS;
    // ROFL_MSG("intervMask  " << std::bitset<FT::BIT_NUM>(intervMask));
    // ROFL_MSG("mlow        " << std::bitset<FT::BIT_NUM>(mlow) << "  " << low);
    // ROFL_MSG("     ->     " << std::bitset<FT::BIT_NUM>(FT::toInt(low)));
    // ROFL_MSG("mmid        " << std::bitset<FT::BIT_NUM>(mmid) << "  " << mid);
    // ROFL_MSG("     ->     " << std::bitset<FT::BIT_NUM>(FT::toInt(mid)));
    // ROFL_MSG("mupp        " << std::bitset<FT::BIT_NUM>(mupp) << "  " << upp);
    // ROFL_MSG("     ->     " << std::bitset<FT::BIT_NUM>(FT::toInt(upp)));

    return level;
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
        level = IT::BIT_NUM -
                IT::nlz(IT::removeSign(v1[d]) ^ IT::removeSign(v2[d]));
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
    levelSplit = intervalPow2Int(v1[0], v2[0], low[0], mid[0], upp[0]);
    for (int d = 1; d < Dim; ++d) {
        level = intervalPow2Int(v1[d], v2[d], low[d], mid[d], upp[d]);
        if (level > levelSplit) {
            mid[dimSplit] = low[dimSplit];
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
    lastXor = xorFloat(v1[0], v2[0], lastMantissa, lastExponent);
    for (int d = 1; d < Dim; ++d) {
        currXor = xorFloat(v1[d], v2[d], currMantissa, currExponent);
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
        xorFloat(v1[d], v2[d], mantissa, exponent);
        if (exponent > exponentMax) {
            exponentMax = exponent;
        }
    }
    return exponentMax;
}

template <typename F, int Dim>
void mortonSplitFloat(const F* v1, const F* v2, F* low, F* mid, F* upp) {
    int dimSplit, levelSplit, level;

    dimSplit = 0;
    levelSplit = intervalPow2Float(v1[0], v2[0], low[0], mid[0], upp[0]);
    // ROFL_VAR5(v1[0], v2[0], low[0], mid[0], upp[0]);
    for (int d = 1; d < Dim; ++d) {
        level = intervalPow2Float(v1[d], v2[d], low[d], mid[d], upp[d]);
        // ROFL_VAR5(v1[d], v2[d], low[d], mid[d], upp[d]);
        if (level > levelSplit) {
            mid[dimSplit] = low[dimSplit];
            dimSplit = d;
            levelSplit = level;
        } else {
            mid[d] = low[d];
        }
    }
}

// ---------------------------------------------------------------
// MORTON TRAITS
// ---------------------------------------------------------------

/**
 * Struct MortonTraits<Scalar, Dim> is a class that provides the functions
 * to perform operations related to Morton order on vectors/arrays of type
 * Scalar* and size Dim.
 * MortonTraits<> has specialized implementations according to the type of
 * Scalar, e.g. for integral types (basically integers) or floating types.
 * The specialized implementation uses the C++ type_traits with C++-17 syntax.
 * For example,
 *   std::enable_if_t<EXPR> (with C++-11/14 std::enable_if<EXPR>::type)
 *   std::is_integral_v<EXPR> (with C++-11/14 std::is_integral<EXPR>::value).
 *
 * The operations related to Morton order are the following:
 * - compare();
 * - distance();
 * - split();
 */
template <typename Scalar, size_t Dim, typename Enable = void>
struct MortonTraits;

template <typename Scalar, size_t Dim>
#if __cplusplus >= 201703L
struct MortonTraits<Scalar, Dim, std::enable_if_t<std::is_integral_v<Scalar> > >
#else
struct MortonTraits<
    Scalar,
    Dim,
    typename std::enable_if<std::is_integral<Scalar>::value>::type>
#endif
{

    static bool compare(const Scalar* v1, const Scalar* v2) {
        return mortonCmpInt<Scalar, Dim>(v1, v2);
    }

    static int distance(const Scalar* v1, const Scalar* v2) {
        return mortonDistanceInt<Scalar, Dim>(v1, v2);
    }

    static void split(const Scalar* v1,
                      const Scalar* v2,
                      Scalar* low,
                      Scalar* mid,
                      Scalar* upp) {
        mortonSplitInt<Scalar, Dim>(v1, v2, low, mid, upp);
    }
};

template <typename Scalar, size_t Dim>
#if __cplusplus >= 201703L
struct MortonTraits<Scalar,
                    Dim,
                    std::enable_if_t<std::is_floating_point_v<Scalar> > >
#else
struct MortonTraits<
    Scalar,
    Dim,
    typename std::enable_if<std::is_floating_point<Scalar>::value>::type>
#endif
{

    static bool compare(const Scalar* v1, const Scalar* v2) {
        return mortonCmpFloat<Scalar, Dim>(v1, v2);
    }

    static int distance(const Scalar* v1, const Scalar* v2) {
        return mortonDistanceFloat<Scalar, Dim>(v1, v2);
    }

    static void split(const Scalar* v1,
                      const Scalar* v2,
                      Scalar* low,
                      Scalar* mid,
                      Scalar* upp) {
        mortonSplitFloat<Scalar, Dim>(v1, v2, low, mid, upp);
    }
};

}  // namespace rofl

#endif
