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
#include <filesystem>
#include <rofl/common/io.h>
#include <rofl/common/param_map.h>

int main(int argc, char** argv) {
    rofl::ParamMap params;
    std::string dir, prefix, postfix;
    std::vector<std::string> files;

    params.read(argc, argv);
    params.getParam<std::string>("dir", dir, ".");
    
    std::cout << "\nParameter values:\n";
    params.write(std::cout);
    std::cout << std::endl;

    std::cout << "List of files in directory \"" << dir << "\":" << std::endl;
    rofl::getDirectoryFiles(dir, files);
    
    for (auto& f : files) {
        prefix = std::filesystem::path(f).stem().string() + "_";
        postfix = std::filesystem::path(f).extension().string();
        std::cout << " \"" << f << "\": stamped \"" << rofl::generateStampedString(prefix, postfix) << "\"" << std::endl;
    }
    

    return 0;
}
