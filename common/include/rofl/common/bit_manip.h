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
#ifndef ROFL_BIT_MANIP_H
#define ROFL_BIT_MANIP_H
 
#include <cstdint>
#include <bitset>

namespace rofl {

    // ---------------------------------------------------------------
    // FLOOR/CEIL LOW POWER 2
    // ---------------------------------------------------------------

    /**
     * Returns the larger power of 2 less than the given argument.
     * Example: flp2(5) = 2^2.
     * @param x input argument
     */
    uint8_t flp2(uint8_t x) {
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
    uint16_t flp2(uint16_t x) {
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
    uint32_t flp2(uint32_t x) {
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
    uint64_t flp2(uint64_t x) {
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
    uint8_t clp2(uint8_t x) {
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
    uint16_t clp2(uint16_t x) {
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
    uint32_t clp2(uint32_t x) {
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
    uint64_t clp2(uint64_t x) {
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
    // NUMBER OF LEADING ZEROS
    // ---------------------------------------------------------------

    int nlz(uint8_t x) {
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
        if (y != 0) return n - 2;
        return n - x;
    }

    int nlz(uint16_t x) {
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
        if (y != 0) return n - 2;
        return n - x;
    }

    int nlz(uint32_t x) {
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
        if (y != 0) return n - 2;
        return n - x;
    }

    int nlz(uint64_t x) {
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
        if (y != 0) return n - 2;
        return n - x;
    }

    // ---------------------------------------------------------------
    // DISCRETE LOG2
    // ---------------------------------------------------------------

    uint8_t log2(uint8_t x) {
        return (7 - nlz(x));
    }

    uint16_t log2(uint16_t x) {
        return (15 - nlz(x));
    }

    uint32_t log2(uint32_t x) {
        return (31 - nlz(x));
    }

    uint64_t log2(uint64_t x) {
        return (63 - nlz(x));
    }
    
} // end of namespace

#endif
