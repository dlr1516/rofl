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
#ifndef ROFL_COMMON_IO_H_
#define ROFL_COMMON_IO_H_

#include <string>
#include <vector>

namespace rofl {

    /** Returns a list of files based on Unix-like GLOB. 
     */
    //void glob(const std::string globPath, std::vector<std::string>& matchingFiles);

    /** Returns the list of files in the given directory. 
     */
    void getDirectoryFiles(const std::string& dirPath, std::vector<std::string>& matchingFiles);

    /** Generates a filename dependent on date and time.
     */
    std::string generateStampedString(const std::string prefix = "", const std::string postfix = "");

}

#endif 

