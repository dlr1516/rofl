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
#include <rofl/common/io.h>
#include <rofl/common/macros.h>

#include <boost/date_time/posix_time/posix_time.hpp>

#include <iostream>
#include <filesystem>
#include <ctime>
//#include <iomanip>
//#include <codecvt>

namespace rofl {

    // ----------------------------------------------
    // I/O OPERATIONS
    // ----------------------------------------------

    //    void glob(const std::string globPath, std::vector<std::string>& matchingFiles) {
    //        glob_t glob_result;
    //        matchingFiles.clear();
    //
    //        // glob struct resides on the stack
    //        memset(&glob_result, 0, sizeof (glob_result));
    //
    //        ::glob(globPath.c_str(), GLOB_TILDE, NULL, &glob_result);
    //        for (unsigned int i = 0; i < glob_result.gl_pathc; ++i) {
    //            matchingFiles.push_back(std::string(glob_result.gl_pathv[i]));
    //        }
    //        globfree(&glob_result);
    //    }

    void getDirectoryFiles(const std::string& dirPath, std::vector<std::string>& matchingFiles) {
        namespace fs = std::filesystem;
        const fs::path dirFS{dirPath};

        for (const fs::directory_entry& i : fs::directory_iterator(dirFS)) {
            if (i.is_regular_file()) {
                matchingFiles.push_back(i.path().string());
            }
        }
        std::sort(matchingFiles.begin(), matchingFiles.end());
    }

    std::string generateStampedString(const std::string prefix, const std::string postfix) {
        std::time_t now = std::time(nullptr);
        std::tm tm = *std::localtime(&now);
        std::stringstream ss;
        ss << prefix << std::put_time(&tm, "%Y%m%d_%H%M_%S") << postfix;
        return ss.str();
//        boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
//        std::ostringstream formatter;
//        std::string formatstring = prefix + "%Y%m%d_%H%M_%S" + postfix;
//        formatter.imbue(std::locale(std::cout.getloc(), new boost::posix_time::time_facet(formatstring.c_str())));
//        formatter << now;
//        return formatter.str();
    }

} // end of namespace

