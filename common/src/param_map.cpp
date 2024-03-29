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
#include <rofl/common/param_map.h>
#include <cstdio>
#include <sstream>

namespace rofl {

    ParamMap::ParamMap() : table_() {
    }

    ParamMap::~ParamMap() {
    };

    void ParamMap::clear() {
        table_.clear();
    }

    bool ParamMap::read(std::istream& in) {
        std::string name;
        std::string value;
        std::string length;
        char buffer[MAX_LEN];
        while (in.getline(buffer, MAX_LEN)) {
            std::stringstream ss(buffer);
            if (ss >> name >> value) {
                if (name.empty() || name.at(0) == COMMENT) {
                    continue;
                }
                table_.erase(name);
                table_.insert(std::make_pair(name, value));
            }
        }
        //  while (in >> name) {
        //    if (name.at(0) == COMMENT) {
        //      in.getline(buffer, MAX_LEN);
        //      continue;
        //    }
        //    if (in >> value) {
        //      table_.erase(name);
        //      table_.insert(std::make_pair(name, value));
        //    }
        //  }
        return true;
    }

    bool ParamMap::read(std::string& filename) {
        std::ifstream file(filename.c_str());
        if (!file) {
            std::cerr << __FILE__ << "," << __LINE__ << ": Cannot open \"" << filename << "\"" << std::endl;
            return false;
        }
        read(file);
        file.close();
        return true;
    }

    bool ParamMap::read(int argc, char** argv) {
        int argi = 1;
        while (argi < argc) {
            //        std::cout << "parameter " << argi << std::endl;
            std::string paramName(argv[argi]);
            if (argi + 1 < argc && isOption(paramName)) { // Checks if paramName is an option
                std::string paramNameStripped = paramName.substr(1);
                std::string paramValue(argv[argi + 1]);
                if (!isOption(paramValue)) {
                    //                std::cout << "Insert variable \"" << paramNameStripped << "\", value \"" << paramValue << "\"" << std::endl;
                    table_.erase(paramNameStripped);
                    table_.insert(std::make_pair(paramNameStripped, paramValue));
                    //                const_iterator v = table_.find(paramNameStripped);
                    //                std::cout << "after find" << std::endl;
                    //                if (v != table_.end()) {
                    //                    std::cout << "Table: find(" << v->first << "), value \"" << v->second << "\"" << std::endl;
                    //                } else {
                    //                    std::cout << "Table: find(" << paramNameStripped << "), NO VALUE" << std::endl;
                    //                }
                }
            }
            ++argi;
        }
        return true;
    }

    bool ParamMap::write(std::ostream& out) const {
        for (const_iterator it = table_.begin(); it != table_.end(); ++it) {
            out << it->first << " " << it->second << std::endl;
        }
        return true;
    }

    bool ParamMap::write(std::ostream &out, const std::string &linePrefix) const {
        for (const_iterator it = table_.begin(); it != table_.end(); ++it) {
            out << linePrefix << it->first << " " << it->second << std::endl;
        }
        return true;
    }

    bool ParamMap::write(std::string& filename) const {
        std::ofstream file(filename.c_str());
        if (!file) {
            std::cerr << __FILE__ << "," << __LINE__ << ": Cannot open \"" << filename << "\"" << std::endl;
            return false;
        }
        write(file);
        file.close();
        return true;
    }

    void ParamMap::setParamString(std::string paramName, std::string paramValue) {
        table_.erase(paramName);
        table_.insert(std::make_pair(paramName, paramValue));
    }

    void ParamMap::adaptTildeInPaths()
    {
        for (auto it = table_.begin(); it != table_.end(); ++it)
        {
            auto first = it->first;
            auto second = it->second;
            // std::cout << "second with tilde: " << second << std::endl;
            std::string homePath = getenv("HOME");
            homePath += "/"; // getenv() return only /home/user -> need to add another
            // std::cout << "homePath " << homePath << std::endl;
            if (second.rfind("~") == 0)
            {
                second.replace(0, 2, homePath);
                std::cout << "second WITHOUT tilde: " << second << std::endl;
                setParam(first, second);
            }
        }
    }

    bool ParamMap::isOption(std::string str) {
        return (str.length() >= 2 && str.at(0) == '-' && isalpha(str.at(1)));
    }

} // end of namespace
