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
#ifndef ROFL_COMMON_NUMERIC_TRAITS_H_
#define ROFL_COMMON_NUMERIC_TRAITS_H_

#include <cstdint>
#include <rofl/common/bit_manip.h>

namespace rofl {

    // ---------------------------------------------------------------
    // INTEGER TRAITS
    // ---------------------------------------------------------------

    template <typename I> struct IntegerTraits;

    template <> struct IntegerTraits<int8_t> {
        using IntegerType = int8_t;
        using UnsignedType = uint8_t;

        static const int BIT_NUM = 8;

        static UnsignedType removeSign(const IntegerType& i) {
            return (i ^ 0x01);
        }

        static IntegerType nlz(const IntegerType& i) {
            return nlz8(i);
        }

        static IntegerType log2Mod(const IntegerType& i) {
            if (i > 0)
                return (BIT_NUM - 1 - nlz8(i));
            else if (i < 0)
                return (BIT_NUM - 1 - nlz8(-i));
            else
                return 0; // arbitrary output for invalid argument 0
        }

        static IntegerType floorPow2(const IntegerType& i) {
            if (i >= 0)
                return flp2u8((UnsignedType) i);
            else
                return -clp2u8((UnsignedType) (-i));
        }

        static IntegerType ceilPow2(const IntegerType& i) {
            if (i >= 0)
                return clp2u8((UnsignedType) i);
            else
                return -flp2u8((UnsignedType) (-i));
        }
    };

    template <> struct IntegerTraits<uint8_t> {
        using IntegerType = uint8_t;
        using UnsignedType = uint8_t;

        static const int BIT_NUM = 8;

        static UnsignedType removeSign(const IntegerType& i) {
            return i;
        }

        static IntegerType nlz(const IntegerType& i) {
            return nlz8(i);
        }

        static IntegerType log2Mod(const IntegerType& i) {
            if (i != 0)
                return (BIT_NUM - nlz8(i) - 1);
            else
                return 0; // arbitrary output for invalid argument 0
        }

        static IntegerType floorPow2(const IntegerType& i) {
            return flp2u8(i);
        }

        static IntegerType ceilPow2(const IntegerType& i) {
            return clp2u8(i);
        }
    };

    template <> struct IntegerTraits<int16_t> {
        using IntegerType = int16_t;
        using UnsignedType = uint16_t;

        static const int BIT_NUM = 16;

        static UnsignedType removeSign(const IntegerType& i) {
            return (i ^ 0x0001);
        }

        static IntegerType nlz(const IntegerType& i) {
            return nlz16(i);
        }

        static IntegerType log2Mod(const IntegerType& i) {
            if (i > 0)
                return (BIT_NUM - 1 - nlz16(i));
            else if (i < 0)
                return (BIT_NUM - 1 - nlz16(-i));
            else
                return 0; // arbitrary output for invalid argument 0
        }

        static IntegerType floorPow2(const IntegerType& i) {
            if (i >= 0)
                return flp2u16((UnsignedType) i);
            else
                return -clp2u16((UnsignedType) (-i));
        }

        static IntegerType ceilPow2(const IntegerType& i) {
            if (i >= 0)
                return clp2u16((UnsignedType) i);
            else
                return -flp2u16((UnsignedType) (-i));
        }
    };

    template <> struct IntegerTraits<uint16_t> {
        using IntegerType = uint16_t;
        using UnsignedType = uint16_t;

        static const int BIT_NUM = 16;

        static UnsignedType removeSign(const IntegerType& i) {
            return i;
        }

        static IntegerType nlz(const IntegerType& i) {
            return nlz16(i);
        }

        static IntegerType log2Mod(const IntegerType& i) {
            if (i != 0)
                return (BIT_NUM - nlz16(i) - 1);
            else
                return 0; // arbitrary output for invalid argument 0
        }

        static IntegerType floorPow2(const IntegerType& i) {
            return flp2u16(i);
        }

        static IntegerType ceilPow2(const IntegerType& i) {
            return clp2u16(i);
        }
    };

    template <> struct IntegerTraits<int32_t> {
        using IntegerType = int32_t;
        using UnsignedType = uint32_t;

        static const int BIT_NUM = 32;

        static UnsignedType removeSign(const IntegerType& i) {
            return (i ^ 0x00000001);
        }

        static IntegerType nlz(const IntegerType& i) {
            return nlz32(i);
        }

        static IntegerType log2Mod(const IntegerType& i) {
            if (i > 0)
                return (BIT_NUM - 1 - nlz32(i));
            else if (i < 0)
                return (BIT_NUM - 1 - nlz32(-i));
            else
                return 0; // arbitrary output for invalid argument 0
        }

        static IntegerType floorPow2(const IntegerType& i) {
            if (i >= 0)
                return flp2u32((UnsignedType) i);
            else
                return -clp2u32((UnsignedType) (-i));
        }

        static IntegerType ceilPow2(const IntegerType& i) {
            if (i >= 0)
                return clp2u32((UnsignedType) i);
            else
                return -flp2u32((UnsignedType) (-i));
        }
    };

    template <> struct IntegerTraits<uint32_t> {
        using IntegerType = uint32_t;
        using UnsignedType = uint32_t;

        static const int BIT_NUM = 32;

        static UnsignedType removeSign(const IntegerType& i) {
            return i;
        }

        static IntegerType nlz(const IntegerType& i) {
            return nlz32(i);
        }

        static IntegerType log2Mod(const IntegerType& i) {
            if (i != 0)
                return (BIT_NUM - nlz32(i) - 1);
            else
                return 0; // arbitrary output for invalid argument 0
        }

        static IntegerType floorPow2(const IntegerType& i) {
            return flp2u32(i);
        }

        static IntegerType ceilPow2(const IntegerType& i) {
            return clp2u32(i);
        }
    };

    template <> struct IntegerTraits<int64_t> {
        using IntegerType = int64_t;
        using UnsignedType = uint64_t;

        static const int BIT_NUM = 64;

        static UnsignedType removeSign(const IntegerType& i) {
            return (i ^ 0x0000000000000001);
        }

        static IntegerType nlz(const IntegerType& i) {
            return nlz64(i);
        }

        static IntegerType log2Mod(const IntegerType& i) {
            if (i > 0)
                return (BIT_NUM - 1 - nlz8(i));
            else if (i < 0)
                return (BIT_NUM - 1 - nlz8(-i));
            else
                return 0; // arbitrary output for invalid argument 0
        }

        static IntegerType floorPow2(const IntegerType& i) {
            if (i >= 0)
                return flp2u64((UnsignedType) i);
            else
                return -clp2u64((UnsignedType) (-i));
        }

        static IntegerType ceilPow2(const IntegerType& i) {
            if (i >= 0)
                return clp2u64((UnsignedType) i);
            else
                return -flp2u64((UnsignedType) (-i));
        }
    };

    template <> struct IntegerTraits<uint64_t> {
        using IntegerType = uint64_t;
        using UnsignedType = uint64_t;

        static const int BIT_NUM = 64;

        static UnsignedType removeSign(const IntegerType& i) {
            return i;
        }

        static IntegerType nlz(const IntegerType& i) {
            return nlz64(i);
        }

        static IntegerType log2Mod(const IntegerType& i) {
            if (i != 0)
                return (BIT_NUM - nlz64(i) - 1);
            else
                return 0; // arbitrary output for invalid argument 0
        }

        static IntegerType floorPow2(const IntegerType& i) {
            return flp2u64(i);
        }

        static IntegerType ceilPow2(const IntegerType& i) {
            return clp2u64(i);
        }
    };

    // ---------------------------------------------------------------
    // INTEGER FUNCTIONS
    // ---------------------------------------------------------------

    /**
     * Returns the number of leading zeros of the given integer.
     * The number of leading zeros depends on the size of the integer type.
     * E.g.: int32_t i = 255;  nls(i) == 24
     */
    template <typename I>
    I nlz(const I& i) {
        return IntegerTraits<I>::nlz(i);
    }

    /**
     * Given the input integer i, it returns the exponent l of the power of 2 s.t.
     *   2^l <= abs(i) < 2^(l+1)
     * where abs(i) is the absolute value ("module") of l.
     */
    template <typename I>
    I log2Mod(const I& i) {
        return IntegerTraits<I>::log2Mod(i);
    }

    /**
     * Given an input integer i, it returns:
     * 1) i > 0: the highest power of 2 less than or equal to i;
     * 2) i == 0: zero;
     * 3) i < 0: the opposite of the highest power of 2 less than or equal to i.
     * E.g.  floorPow2(6) = 4, floorPow2(0) = 0, floorPow2(-5) = -8.
     *
     * @param i the input number
     * @return
     */
    template <typename I>
    I floorPow2(const I& i) {
        return IntegerTraits<I>::floorPow2(i);
    }

    /**
     * Given an input integer i, it returns:
     * 1) i > 0: the lowest power of 2 greater than or equal to i;
     * 2) i == 0: zero;
     * 3) i < 0: the opposite of the lowest power of 2 greater than or equal to i.
     * E.g.  ceilPow2(6) = 8, ceilPow2(0) = 0, ceilPow2(-5) = -4.
     *
     * @param i the input number
     * @return
     */
    template <typename I>
    I ceilPow2(const I& i) {
        return IntegerTraits<I>::ceilPow2(i);
    }

    // ---------------------------------------------------------------
    // FLOATING POINT NUMBER TRAITS
    // ---------------------------------------------------------------

    /**
     * Traits class for floating points for extraction of mantissa, exponent and sign.
     */
    template <typename FloatType> struct FloatTraits;

    /**
     * Struct FloatTraits specialized for single precision floating-point number.
     */
    template <> struct FloatTraits<float> {
        using FloatType = float;
        using IntegerType = int32_t;
        using UnsignedType = uint32_t;

        static const int BIT_NUM = 32;
        static const int MANTISSA_BITS = 23;
        static const int EXPONENT_BITS = 8;
        static const int EXPONENT_BIAS = 127;
        
        static constexpr IntegerType MANTISSA_IMPLICIT_BIT = (IntegerType)1 << MANTISSA_BITS;
        static constexpr IntegerType MANTISSA_MASK = ((IntegerType)1 << MANTISSA_BITS) - 1;
        static constexpr IntegerType EXPONENT_MASK = (((IntegerType)1 << EXPONENT_BITS) - 1) << MANTISSA_BITS;
        static constexpr IntegerType EXPONENT_MIN = -EXPONENT_BIAS;
        static constexpr IntegerType EXPONENT_MAX = (((IntegerType)1 << EXPONENT_BITS) - 1) - EXPONENT_BIAS;
        static constexpr IntegerType SIGN_MASK = (IntegerType)1 << (MANTISSA_BITS + EXPONENT_BITS);

        static FloatType compose(const IntegerType& mantissa, const IntegerType& exponent, bool sign) {
            return setMantissaExpSignF(mantissa, exponent, sign);
        }

        static void decompose(const FloatType& floatVal, IntegerType& mantissa, IntegerType& exponent, bool& sign) {
            getMantissaExpSignF(floatVal, mantissa, exponent, sign);
        }
    };

    /**
     * Struct FloatTraits specialized for double precision floating-point number.
     */
    template <> struct FloatTraits<double> {
        using FloatType = double;
        using IntegerType = int64_t;
        using UnsignedType = uint64_t;

        static const int BIT_NUM = 64;
        static const int MANTISSA_BITS = 52;
        static const int EXPONENT_BITS = 8;
        static const int EXPONENT_BIAS = 1023;
        
        static constexpr IntegerType MANTISSA_IMPLICIT_BIT = (IntegerType)1 << MANTISSA_BITS;
        static constexpr IntegerType MANTISSA_MASK = ((IntegerType)1 << MANTISSA_BITS) - 1;
        static constexpr IntegerType EXPONENT_MASK = (((IntegerType)1 << EXPONENT_BITS) - 1) << MANTISSA_BITS;
        static constexpr IntegerType EXPONENT_MIN = -EXPONENT_BIAS;
        static constexpr IntegerType EXPONENT_MAX = (((IntegerType)1 << EXPONENT_BITS) - 1) - EXPONENT_BIAS;
        static constexpr IntegerType SIGN_MASK = (IntegerType)1 << (MANTISSA_BITS + EXPONENT_BITS);

        static FloatType compose(const IntegerType& mantissa, const IntegerType& exponent, bool sign) {
            return setMantissaExpSignD(mantissa, exponent, sign);
        }

        static void decompose(const FloatType& floatVal, IntegerType& mantissa, IntegerType& exponent, bool& sign) {
            getMantissaExpSignD(floatVal, mantissa, exponent, sign);
        }
    };

} // end of namespace 

#endif 

