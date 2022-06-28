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
#include <rofl/common/numeric_traits.h>
#include <rofl/common/param_map.h>

#include <bitset>
#include <fstream>
#include <iostream>

template <typename F>
void testFloatTraits(const F& f1, const F& f2);

int main(int argc, char** argv) {
    rofl::ParamMap params;
    float f1, f2;
    double d1, d2;

    params.read(argc, argv);
    params.getParam<float>("f1", f1, float(9.0f));
    params.getParam<float>("f2", f2, float(13.0f));

    std::cout << "Params:\n";
    params.write(std::cout);
    std::cout << std::endl;
    d1 = f1;
    d2 = f2;

    testFloatTraits<float>(f1, f2);
    testFloatTraits<double>(d1, d2);
    return 0;
}

template <typename F>
void testFloatTraits(const F& f1, const F& f2) {
    using FT = rofl::FloatTraits<F>;
    using IntegerType = typename FT::IntegerType;
    using UnsignedType = typename FT::UnsignedType;
    F div;
    IntegerType fm1, fm2, fmdiv, fe1, fe2, fediv;
    bool fs1, fs2, fsdiv;

    std::cout << "\n---\nFloatTraits<" << typeid(F).name() << ">\n";
    std::cout << "BIT_NUM " << FT::BIT_NUM << "\n";
    std::cout << "MANTISSA_BITS " << FT::MANTISSA_BITS << "\n";
    std::cout << "EXPONENT_BITS " << FT::EXPONENT_BITS << "\n";
    std::cout << "EXPONENT_BIAS " << FT::EXPONENT_BIAS << "\n";
    std::cout << "MANTISSA_IMPLICIT_BIT "
              << std::bitset<FT::BIT_NUM>(FT::MANTISSA_IMPLICIT_BIT) << "\n";
    std::cout << "MANTISSA_MASK         "
              << std::bitset<FT::BIT_NUM>(FT::MANTISSA_MASK) << "\n";
    std::cout << "EXPONENT_MASK         "
              << std::bitset<FT::BIT_NUM>(FT::EXPONENT_MASK) << "\n";
    std::cout << "EXPONENT_MIN          "
              << std::bitset<FT::BIT_NUM>(FT::EXPONENT_MIN) << " "
              << FT::EXPONENT_MIN << "\n";
    std::cout << "EXPONENT_MAX          "
              << std::bitset<FT::BIT_NUM>(FT::EXPONENT_MAX) << " "
              << FT::EXPONENT_MAX << "\n";
    std::cout << "SIGN_MASK             "
              << std::bitset<FT::BIT_NUM>(FT::SIGN_MASK) << "\n";
    std::cout << "" << std::endl;

    FT::decompose(f1, fm1, fe1, fs1);
    FT::decompose(f2, fm2, fe2, fs2);
    div = FT::dividePow2(f1, 3);
    FT::decompose(div, fmdiv, fediv, fsdiv);
    std::cout << "f1:  m " << std::bitset<FT::MANTISSA_BITS>(fm1)
              << " e " << std::bitset<FT::EXPONENT_BITS>(fe1)
              << " s " << fs1 << "  " << f1 << std::endl;
    std::cout << "f2:  m " << std::bitset<FT::MANTISSA_BITS>(fm2)
              << " e " << std::bitset<FT::EXPONENT_BITS>(fe2)
              << " s " << fs2 << "  " << f2 << std::endl;
    std::cout << "div = " << f1 << " / 2^3\n";
    std::cout << "div: m " << std::bitset<FT::MANTISSA_BITS>(fmdiv)
              << " e " << std::bitset<FT::EXPONENT_BITS>(fediv)
              << " s " << fsdiv << "  " << div << std::endl;
}