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
#include <rofl/common/bit_manip.h>

namespace rofl {

    // ---------------------------------------------------------------
    // NUMBER OF LEADING ZEROS
    // ---------------------------------------------------------------

    int8_t nlz8(uint8_t x) {
        uint8_t y;
        int n;

        n = 8;
        y = x >> 4;
        if (y != 0) {
            n = n - 4;
            x = y;
        }
        y = x >> 2;
        if (y != 0) {
            n = n - 2;
            x = y;
        }
        y = x >> 1;
        if (y != 0)
            return n - 2;
        return n - x;
    }

    int16_t nlz16(uint16_t x) {
        uint16_t y;
        int n;

        n = 16;
        y = x >> 8;
        if (y != 0) {
            n = n - 8;
            x = y;
        }
        y = x >> 4;
        if (y != 0) {
            n = n - 4;
            x = y;
        }
        y = x >> 2;
        if (y != 0) {
            n = n - 2;
            x = y;
        }
        y = x >> 1;
        if (y != 0)
            return n - 2;
        return n - x;
    }

    int32_t nlz32(uint32_t x) {
        uint32_t y;
        int n;

        n = 32;
        y = x >> 16;
        if (y != 0) {
            n = n - 16;
            x = y;
        }
        y = x >> 8;
        if (y != 0) {
            n = n - 8;
            x = y;
        }
        y = x >> 4;
        if (y != 0) {
            n = n - 4;
            x = y;
        }
        y = x >> 2;
        if (y != 0) {
            n = n - 2;
            x = y;
        }
        y = x >> 1;
        if (y != 0)
            return n - 2;
        return n - x;
    }

    int64_t nlz64(uint64_t x) {
        uint64_t y;
        int n;

        n = 64;
        y = x >> 32;
        if (y != 0) {
            n = n - 32;
            x = y;
        }
        y = x >> 16;
        if (y != 0) {
            n = n - 16;
            x = y;
        }
        y = x >> 8;
        if (y != 0) {
            n = n - 8;
            x = y;
        }
        y = x >> 4;
        if (y != 0) {
            n = n - 4;
            x = y;
        }
        y = x >> 2;
        if (y != 0) {
            n = n - 2;
            x = y;
        }
        y = x >> 1;
        if (y != 0)
            return n - 2;
        return n - x;
    }

    // ---------------------------------------------------------------
    // FLOOR/CEIL LOW POWER 2
    // ---------------------------------------------------------------

    /**
     * Returns the larger power of 2 less than the given argument.
     * Example: flp2(5) = 2^2.
     * @param x input argument
     */
    uint8_t flp2u8(uint8_t x) {
        x = x | (x >> 1);
        x = x | (x >> 2);
        x = x | (x >> 4);
        return x - (x >> 1);
    }

    /**
     * Returns the larger power of 2 less than the given argument.
     * Example: flp2(5) = 2^2.
     * @param x input argument
     */
    uint16_t flp2u16(uint16_t x) {
        x = x | (x >> 1);
        x = x | (x >> 2);
        x = x | (x >> 4);
        x = x | (x >> 8);
        return x - (x >> 1);
    }

    /**
     * Returns the larger power of 2 less than the given argument.
     * Example: flp2(5) = 2^2.
     * @param x input argument
     */
    uint32_t flp2u32(uint32_t x) {
        //        std::cout << " x:       " << std::bitset<32>(x) << " | \n"
        //                  << " x >> 1:  " << std::bitset<32>(x >> 1) << "\n";
        x = x | (x >> 1);
        //        std::cout << " x:       " << std::bitset<32>(x) << " | \n"
        //                  << " x >> 2:  " << std::bitset<32>(x >> 2) << "\n";
        x = x | (x >> 2);
        //        std::cout << " x:       " << std::bitset<32>(x) << " | \n"
        //                  << " x >> 4:  " << std::bitset<32>(x >> 4) << "\n";
        x = x | (x >> 4);
        //        std::cout << " x:       " << std::bitset<32>(x) << " | \n"
        //                  << " x >> 8:  " << std::bitset<32>(x >> 8) << "\n";
        x = x | (x >> 8);
        //        std::cout << " x:       " << std::bitset<32>(x) << " | \n"
        //                  << " x >> 16: " << std::bitset<32>(x >> 16) << "\n";
        x = x | (x >> 16);
        //        std::cout << " x:       " << std::bitset<32>(x) << " - \n"
        //                  << " x >> 1:  " << std::bitset<32>(x >> 1) << "\n"
        //                  << " diff     " << (x - (x >> 1)) << std::endl;
        return x - (x >> 1);
    }

    /**
     * Returns the larger power of 2 less than the given argument.
     * Example: flp2(5) = 2^2.
     * @param x input argument
     */
    uint64_t flp2u64(uint64_t x) {
        x = x | (x >> 1);
        x = x | (x >> 2);
        x = x | (x >> 4);
        x = x | (x >> 8);
        x = x | (x >> 16);
        x = x | (x >> 32);
        return x - (x >> 1);
    }

    /**
     * Returns the smaller power of 2 greater than the given argument.
     * Example: clp2(5) = 2^3.
     * @param x input argument
     */
    uint8_t clp2u8(uint8_t x) {
        x = x - 1;
        x = x | (x >> 1);
        x = x | (x >> 2);
        x = x | (x >> 4);
        return x + 1;
    }

    /**
     * Returns the smaller power of 2 greater than the given argument.
     * Example: clp2(5) = 2^3.
     * @param x input argument
     */
    uint16_t clp2u16(uint16_t x) {
        x = x - 1;
        x = x | (x >> 1);
        x = x | (x >> 2);
        x = x | (x >> 4);
        x = x | (x >> 8);
        return x + 1;
    }

    /**
     * Returns the smaller power of 2 greater than the given argument.
     * Example: clp2(5) = 2^3.
     * @param x input argument
     */
    uint32_t clp2u32(uint32_t x) {
        x = x - 1;
        x = x | (x >> 1);
        x = x | (x >> 2);
        x = x | (x >> 4);
        x = x | (x >> 8);
        x = x | (x >> 16);
        return x + 1;
    }

    /**
     * Returns the smaller power of 2 greater than the given argument.
     * Example: clp2(5) = 2^3.
     * @param x input argument
     */
    uint64_t clp2u64(uint64_t x) {
        x = x - 1;
        x = x | (x >> 1);
        x = x | (x >> 2);
        x = x | (x >> 4);
        x = x | (x >> 8);
        x = x | (x >> 16);
        x = x | (x >> 32);
        return x + 1;
    }

    // ---------------------------------------------------------------
    // IEEE 754: EXTRACTION OF MANTISSA, EXPONENT, SIGN FROM FLOATING POINT TYPES
    // ---------------------------------------------------------------

    void getMantissaExpSignF(const float& f, int32_t& m, int32_t& e, bool& s) {

        union {
            float f;
            uint32_t i;
        } floatBits;

        floatBits.f = f;
        m = (0x007FFFFF & floatBits.i); // To add the implicit 1 bit, apply "| 0x00800000";
        e = ((0x7F800000 & floatBits.i) >> 23) - 127;
        s = 0x80000000 & floatBits.i;
    }

    float setMantissaExpSignF(int32_t m, int32_t e, bool s) {

        union {
            float f;
            uint32_t i;
        } floatBits;
        floatBits.i = (0x007FFFFF & m) | ((e + 127) << 23) | ((int32_t) s << 31);
        return floatBits.f;
    }

    void getMantissaExpSignD(const double& f, int64_t& m, int64_t& e, bool& s) {

        union {
            double f;
            uint64_t i;
        } floatBits;

        floatBits.f = f;
        m = (0x000FFFFFFFFFFFFF & floatBits.i); // To add the implicit 1 bit, apply " | 0x0010000000000000"
        e = ((0x7FF0000000000000 & floatBits.i) >> 52) - 1023;
        s = 0x8000000000000000 & floatBits.i;
    }

    double setMantissaExpSignD(int64_t m, int64_t e, bool s) {

        union {
            double f;
            uint64_t i;
        } floatBits;
        floatBits.i = (0x000FFFFFFFFFFFFF & m) | ((e + 1023) << 52) | ((int64_t) s << 63);
        return floatBits.f;
    }



} // end of namespace rofl
