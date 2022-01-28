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
#ifndef ROFL_COMMON_BIT_MANIP_H_
#define ROFL_COMMON_BIT_MANIP_H_

#include <cstdint>
#include <bitset>

namespace rofl {

    // ---------------------------------------------------------------
    // NUMBER OF LEADING ZEROS
    // ---------------------------------------------------------------

    /**
     * Number of leading zeros in given argument.
     * @param x input argument
     */
    int8_t nlz8(uint8_t x);

    /**
     * Number of leading zeros in given argument.
     * @param x input argument
     */
    int16_t nlz16(uint16_t x);

    /**
     * Number of leading zeros in given argument.
     * @param x input argument
     */
    int32_t nlz32(uint32_t x);

    /**
     * Number of leading zeros in given argument.
     * @param x input argument
     */
    int64_t nlz64(uint64_t x);

    // ---------------------------------------------------------------
    // FLOOR/CEIL LOW POWER 2
    // ---------------------------------------------------------------

    /**
     * Returns the largest power of 2 less than the given argument.
     * Example: flp2(5) = 2^2.
     * @param x input argument
     */
    uint8_t flp2u8(uint8_t x);

    /**
     * Returns the largest power of 2 less than the given argument.
     * Example: flp2(5) = 2^2.
     * @param x input argument
     */
    uint16_t flp2u16(uint16_t x);

    /**
     * Returns the largest power of 2 less than the given argument.
     * Example: flp2(5) = 2^2.
     * @param x input argument
     */
    uint32_t flp2u32(uint32_t x);

    /**
     * Returns the largest power of 2 less than the given argument.
     * Example: flp2(5) = 2^2.
     * @param x input argument
     */
    uint64_t flp2u64(uint64_t x);

    /**
     * Returns the smallest power of 2 greater than the given argument.
     * Example: clp2(5) = 2^3.
     * @param x input argument
     */
    uint8_t clp2u8(uint8_t x);

    /**
     * Returns the smallest power of 2 greater than the given argument.
     * Example: clp2(5) = 2^3.
     * @param x input argument
     */
    uint16_t clp2u16(uint16_t x);

    /**
     * Returns the smallest power of 2 greater than the given argument.
     * Example: clp2(5) = 2^3.
     * @param x input argument
     */
    uint32_t clp2u32(uint32_t x);

    /**
     * Returns the smallest power of 2 greater than the given argument.
     * Example: clp2(5) = 2^3.
     * @param x input argument
     */
    uint64_t clp2u64(uint64_t x);

    // ---------------------------------------------------------------
    // IEEE 754: EXTRACTION OF MANTISSA, EXPONENT, SIGN FROM FLOATING POINT TYPES
    // ---------------------------------------------------------------

    /**
     * Returns the mantissa, exponent and sign of single precision floating point numbers.
     * The floating point number f is equal to
     *   (-1)^s * (1 + m * 2^(-lm)) * 2^(e)
     * (length of mantissa lm = 23 for single precision floating point).
     *
     * Example: f = 7.0  is written as
     *   1.11 * 2^(2) = (1 + 6291456 * 2^(-23)) * 2^(2)
     * where:
     *   m = 00000000011000000000000000000000 (decimal 6291456)
     *   e = 00000000000000000000000000000010 (decimal 2)
     *   s = 0
     *
     * @param f input floating point number
     * @param m mantissa value in integer form (the most significant bit 1 is implicit as in IEEE 754)
     * @param e exponent value in integer form (the bias 127 is removed from exponent)
     * @param s sign of floatinf point (true if f is negative, false if positive or zero)
     */
    void getMantissaExpSignF(const float& f, int32_t& m, int32_t& e, bool& s);

    /**
     * Returns the single precision floating point associated to the given mantissa
     * (the most significant bit must be implicit!), exponent (without bias) and sign.
     * @param m mantissa value in integer form (the most significant bit 1 is implicit as in IEEE 754)
     * @param e exponent value in integer form (the bias 127 is removed from exponent string)
     * @param s sign of floatinf point (true if f is negative, false if positive or zero)
     */
    float setMantissaExpSignF(int32_t m, int32_t e, bool s);

    /**
     * Computes the mantissa, exponent and sign of double precision floating point numbers.
     * The floating point number f is equal to
     *   (-1)^s * (1 + m * 2^(-lm)) * 2^(e)
     * (length of mantissa lm = 52 for double precision floating point).
     *
     * Example: f = 7.0  is written as
     *   1.11 * 2^(2) = (1 + 3377699720527872 * 2^(-52)) * 2^(2)
     * where:
     *   m = 0000000000001100000000000000000000000000000000000000000000000000 (decimal 3377699720527872)
     *   e = 0000000000000000000000000000000000000000000000000000000000000010 (decimal 2)
     *   s = 0
     *
     * @param f input floating point number
     * @param m mantissa value in integer form (the most significant bit 1 is implicit as in IEEE 754)
     * @param e exponent value in integer form (the bias 1023 is removed from exponent string)
     * @param s sign of floatinf point (true if f is negative, false if positive or zero)
     */
    void getMantissaExpSignD(const double& f, int64_t& m, int64_t& e, bool& s);

    /**
     * Returns the double precision floating point associated to the given mantissa
     * (the most significant bit must be implicit!), exponent (without bias) and sign.
     * @param m mantissa value in integer form (the most significant bit 1 is implicit as in IEEE 754)
     * @param e exponent value in integer form (the bias 1023 is removed from exponent string)
     * @param s sign of floatinf point (true if f is negative, false if positive or zero)
     */
    double setMantissaExpSignD(int64_t m, int64_t e, bool s);

} // end of namespace

#endif
