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
	 * Computes the manitissa, exponent and sign of single precision floating point numbers.
	 * @param f input floating point number
	 * @param m mantissa value in integer form
	 * @param e exponent value in integer form
	 * @param s sign of floatinf point (true if f is negative, false if positive or zero)
	 */
	void getMantissaExpSignF(const float& f, uint32_t& m, uint32_t& e, bool& s);

	/**
	 * Computes the manitissa, exponent and sign of double precision floating point numbers.
	 * @param f input floating point number
	 * @param m mantissa value in integer form
	 * @param e exponent value in integer form
	 * @param s sign of floatinf point (true if f is negative, false if positive or zero)
	 */
	void getMantissaExpSignD(const double& f, uint64_t& m, uint64_t& e, bool& s);

} // end of namespace

#endif
