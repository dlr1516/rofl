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
#include <rofl/common/morton_sort.h>
#include <rofl/common/param_map.h>

#include <array>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

template <typename F>
std::string printME(const F& f) {
    using FT = rofl::FloatTraits<F>;
    using IntegerType = typename FT::IntegerType;
    using UnsignedType = typename FT::UnsignedType;
    IntegerType fm, fe;
    bool fs;
    std::stringstream ss;

    FT::decompose(f, fm, fe, fs);
    ss << "m " << std::bitset<FT::MANTISSA_BITS>(fm) << " e "
       << std::bitset<FT::EXPONENT_BITS>(fe) << " (" << fe << ")";

    return ss.str();
}

template <typename I>
void testInt(const I& i1, const I& i2) {
    using IT = rofl::IntegerTraits<I>;
    I low, mid, upp, diff;

    diff = IT::removeSign(i1) ^ IT::removeSign(i2);
    rofl::intervalPow2Int(i1, i2, low, mid, upp);

    std::cout << "i1   " << std::bitset<IT::BIT_NUM>(i1) << " " << i1 << "\n"
              << "i2   " << std::bitset<IT::BIT_NUM>(i2) << " " << i2 << "\n"
              << "diff " << std::bitset<IT::BIT_NUM>(diff) << " " << diff
              << " nlz " << (IT::BIT_NUM - IT::nlz(diff)) << "\n"
              << "low  " << std::bitset<IT::BIT_NUM>(low) << " " << low << "\n"
              << "mid  " << std::bitset<IT::BIT_NUM>(mid) << " " << mid << "\n"
              << "upp  " << std::bitset<IT::BIT_NUM>(upp) << " " << upp << "\n"
              << std::endl;
}

template <typename F>
void testFloat(const F& f1, const F& f2) {
    using FT = rofl::FloatTraits<F>;
    using IntegerType = typename FT::IntegerType;
    using UnsignedType = typename FT::UnsignedType;
    F fdiff, flow, fupp, fmid;
    IntegerType mantissaDiff, exponentDiff;
    int level;

    fdiff = rofl::xorFloat(f1, f2, mantissaDiff, exponentDiff);
    level = rofl::intervalPow2Float(f1, f2, flow, fmid, fupp);

    std::cout << "f1    " << printME(f1) << " " << f1 << "\n"
              << "f2    " << printME(f2) << " " << f2 << "\n"
              << "fdiff " << printME(fdiff) << " " << fdiff << " level "
              << level << "\n"
              << "flow  " << printME(flow) << " " << flow << "\n"
              << "fmid  " << printME(fmid) << " " << fmid << "\n"
              << "fupp  " << printME(fupp) << " " << fupp << "\n";
    std::cout << std::endl;
}

int main(int argc, char** argv) {
    rofl::ParamMap params;
    int i1, i2;
    float f1, f2;

    params.read(argc, argv);
    params.getParam<int>("i1", i1, int(4));
    params.getParam<int>("i2", i2, int(9));

    std::cout << "Params:\n";
    params.write(std::cout);
    std::cout << std::endl;
    f1 = i1;
    f2 = i2;

    std::cout << "testing integer/float\n";
    testInt<int>(13, 9);
    testFloat<float>(13.0f, 9.0f);

    std::cout << "testing integer/float\n";
    testInt<int>(5, 14);
    testFloat<float>(5.0f, 14.0f);

    std::cout << "testing integer/float\n";
    testInt<int>(-1, 17);
    testFloat<float>(-1.0f, 17.0f);

    std::cout << "Input number\n";
    testInt<int>(i1, i2);
    testFloat<float>(f1, f2);
    return 0;
}