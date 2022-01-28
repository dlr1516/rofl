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
#include <iostream>
#include <fstream>
#include <bitset>
#include <rofl/common/bit_manip.h>
#include <rofl/common/numeric_traits.h>

template <typename F>
void testFloatTraits();

int main(int argc, char** argv) {

    testFloatTraits<float>();
    testFloatTraits<double>();
    return 0;
}

template <typename F>
void testFloatTraits() {
    using FT = rofl::FloatTraits<F>;
    using IntegerType = typename FT::IntegerType;
    using UnsignedType = typename FT::UnsignedType;

    std::cout << "\n---\nFloatTraits<" << typeid (F).name() << ">\n";
    std::cout << "BIT_NUM " << FT::BIT_NUM << "\n";
    std::cout << "MANTISSA_BITS " << FT::MANTISSA_BITS << "\n";
    std::cout << "EXPONENT_BITS " << FT::EXPONENT_BITS << "\n";
    std::cout << "EXPONENT_BIAS " << FT::EXPONENT_BIAS << "\n";
    std::cout << "MANTISSA_IMPLICIT_BIT " << std::bitset<FT::BIT_NUM>(FT::MANTISSA_IMPLICIT_BIT) << "\n";
    std::cout << "MANTISSA_MASK         " << std::bitset<FT::BIT_NUM>(FT::MANTISSA_MASK) << "\n";
    std::cout << "EXPONENT_MASK         " << std::bitset<FT::BIT_NUM>(FT::EXPONENT_MASK) << "\n";
    std::cout << "SIGN_MASK             " << std::bitset<FT::BIT_NUM>(FT::SIGN_MASK) << "\n";
}