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
#include <rofl/common/label_set.h>

int main(int argc, char** argv) {
    rofl::LabelSet lset1, lset2, ldiff, lunion, linters;
   
    for (int i = 0; i < 7; ++i) {
        lset1.insert(2 * i);
        lset2.insert(3 * i);
    }
    std::cout << "label set 1: ";
    lset1.print(std::cout);
    std::cout << "label set 2: ";
    lset2.print(std::cout);
    
    lunion = lset1.unionSet(lset2);
    linters = lset1.intersectionSet(lset2);
    ldiff = lset1.differenceSet(lset2);
    std::cout << "union:        ";
    lunion.print(std::cout);
    std::cout << "intersection: ";
    linters.print(std::cout);
    std::cout << "difference:   ";
    ldiff.print(std::cout);
    
    return 0;
}

