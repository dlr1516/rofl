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
#ifndef ROFL_INTEGER_TRAITS_H
#define ROFL_INTEGER_TRAITS_H

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
    };

    template <> struct IntegerTraits<uint8_t> {
        using IntegerType = uint8_t;
        using UnsignedType = uint8_t;

        static const int BIT_NUM = 8;

        static UnsignedType removeSign(const IntegerType& i) {
            return i;
        }
    };

    template <> struct IntegerTraits<int16_t> {
        using IntegerType = int16_t;
        using UnsignedType = uint16_t;

        static const int BIT_NUM = 16;

        static UnsignedType removeSign(const IntegerType& i) {
            return (i ^ 0x0001);
        }
    };

    template <> struct IntegerTraits<uint16_t> {
        using IntegerType = uint16_t;
        using UnsignedType = uint16_t;

        static const int BIT_NUM = 16;

        static UnsignedType removeSign(const IntegerType& i) {
            return i;
        }
    };

    template <> struct IntegerTraits<int32_t> {
        using IntegerType = int32_t;
        using UnsignedType = uint32_t;

        static const int BIT_NUM = 32;

        static UnsignedType removeSign(const IntegerType& i) {
            return (i ^ 0x00000001);
        }
    };

    template <> struct IntegerTraits<uint32_t> {
        using IntegerType = uint32_t;
        using UnsignedType = uint32_t;

        static const int BIT_NUM = 32;

        static UnsignedType removeSign(const IntegerType& i) {
            return i;
        }
    };

    template <> struct IntegerTraits<int64_t> {
        using IntegerType = int64_t;
        using UnsignedType = uint64_t;

        static const int BIT_NUM = 64;

        static UnsignedType removeSign(const IntegerType& i) {
            return (i ^ 0x0000000000000001);
        }
    };

    template <> struct IntegerTraits<uint64_t> {
        using IntegerType = uint64_t;
        using UnsignedType = uint64_t;

        static const int BIT_NUM = 64;

        static UnsignedType removeSign(const IntegerType& i) {
            return i;
        }
    };

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

        static void extract(const FloatType& floatVal, UnsignedType& mantissa, UnsignedType& exponent, bool& sign) {
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

        static void extract(const FloatType& floatVal, UnsignedType& mantissa, UnsignedType& exponent, bool& sign) {
            getMantissaExpSignD(floatVal, mantissa, exponent, sign);
        }
    };

} // end of namespace 

#endif 

