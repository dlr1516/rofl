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
#ifndef ROFL_COMMON_PARAM_MAP_H_
#define ROFL_COMMON_PARAM_MAP_H_

#include <iostream>
#include <fstream>
#include <sstream>
//#include <unordered_map>
#include <map>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>

namespace rofl {

    /** Reads and stores parameters from a string, a file, etc.
     */
    class ParamMap {
    public:
        //typedef std::unordered_map<std::string, std::string> table_type;
        using table_type = std::map<std::string, std::string> ; // a map stores lexically ordered parameters (nicer to view!)
        using iterator = table_type::iterator;
        using const_iterator = table_type::const_iterator;

        const static char COMMENT = '#';
        const static unsigned int MAX_LEN = 2000;

        /** Default constructor.
         */
        ParamMap();

        /** Destructor.
         */
        virtual ~ParamMap();

        /** Clears all the content of param table.
         */
        void clear();

        /** Reads params from an input stream in the format:
         *   key1 value1
         *   key2 value2
         *   ...
         */
        bool read(std::istream& in);

        /** Reads params from an input file (format as above).
         */
        bool read(std::string& filename);

        /** Reads from a command line. Required format
         *   ...
         *   argv[i] = "-key1"      // starting with "-"
         *   argv[i+1] = "value1"
         */
        bool read(int argc, char** argv);

        /** Writes the pairs (key,value).
         */
        bool write(std::ostream& out) const;

        /** Writes the pairs (key,value) appending each line to linePrefix
         * (e.g. with linePrefix = "#  " -> line becomes "#  key value")
         */
        bool write(std::ostream& out, const std::string& linePrefix) const;

        /** Writes the parameters to an output file (format as above).
         */
        bool write(std::string& filename) const;

        /** Sets the param (as a set of string).
         */
        void setParamString(std::string paramName, std::string paramValue);

        /** Sets the param (as a set of string).
         */
        template <typename Value>
        void setParam(std::string paramName, const Value& paramValue) {
            std::stringstream sstr;
            sstr << paramValue;
            table_.erase(paramName);
            table_.insert(std::make_pair(paramName, sstr.str()));
        }

        /** Casts the string value of a given parameters to the desired value.
         */
        template <typename Value>
        bool getParam(std::string paramName, Value& value, const Value& defaultValue) {
            const_iterator v = table_.find(paramName);
            if (v != table_.end()) {
                try {
                    value = boost::lexical_cast<Value>(v->second);
                } catch (boost::bad_lexical_cast const&) {
                    std::cerr
                            << __FILE__ << "," << __LINE__ << ": Error: cannot cast string \"" << v->second << "\" to type \"" << typeid (Value).name()
                            << "\" for variable \"" << v->first << "\"" << std::endl;
                }
            } else {
                //            std::cerr << "Parameter " << paramName << " not found." << std::endl;
                value = defaultValue;
                setParam(paramName, defaultValue);
                return false;
            }
            return true;
        }

        /** Casts the string value of a given parameters to the desired value.
         */
        template <typename Value, typename Inserter>
        bool getParamContainerInserter(std::string paramName, Inserter ins, const Value &defaultValue, std::string delim = "[],") {
            const_iterator v = table_.find(paramName);
            if (v != table_.end()) {
                try {
                    // Splits the value into tokens, e.g. "[1,2,3]" with delim "[,]" should become tokens "1", "2" and "3"
                    boost::char_separator<char> sep(delim.c_str());
                    boost::tokenizer < boost::char_separator<char> > tokens(v->second, sep);
                    // Casts each token into a value
                    for (auto it = tokens.begin(); it != tokens.end(); ++it) {
                        ins = boost::lexical_cast < Value > (*it);
                    }

                } catch (boost::bad_lexical_cast const&) {
                    std::cerr << __FILE__ << "," << __LINE__ << ": Error: cannot cast string \"" << v->second << "\" to type \"" << typeid (Value).name()
                            << "\" for variable \"" << v->first << "\"" << std::endl;
                }
            } else {
                //std::cerr << "Parameter " << paramName << " not found." << std::endl;
                //ins = defaultValue;
                setParam(paramName, "");
                return false;
            }
            return true;
        }

    protected:
        table_type table_;

        static bool isOption(std::string str);
    };

} // end of namespace 

#endif

