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
#ifndef ROFL_COMMON_MACROS_H_
#define ROFL_COMMON_MACROS_H_

#include <iostream>
#include <boost/filesystem/path.hpp>

#define ROFL_COMMON_DETAIL_STRING(X) #X
#define ROFL_COMMON_DETAIL_STRING_OP(X) ROFL_COMMON_DETAIL_STRING(X)
#define ROFL_COMMON_STRING_LINE ROFL_COMMON_DETAIL_STRING_OP(__LINE__)

#define ROFL_FILENAME(STR) boost::filesystem::path(STR).filename().string()

#define ROFL_MSG(MSG) std::cout <<  ROFL_FILENAME(__FILE__) << "," << __LINE__ << ": " << MSG << std::endl;

#define ROFL_ERR(MSG) std::cerr <<  ROFL_FILENAME(__FILE__) << "," << __LINE__ << ": " << MSG << std::endl;

#define ROFL_VAR1(X1) std::cout <<  ROFL_FILENAME(__FILE__) << "," << __LINE__ << ": " << #X1 << " " << (X1) << std::endl; 

#define ROFL_VAR2(X1,X2) std::cout << ROFL_FILENAME(__FILE__) << "," << __LINE__ << ": " << (#X1) << " " << (X1) \
   << ", " << (#X2) << " " << (X2) << std::endl;

#define ROFL_VAR3(X1,X2,X3) std::cout << ROFL_FILENAME(__FILE__) << "," << __LINE__ << ": " << (#X1) << " " << (X1) \
   << ", " << (#X2) << " " << (X2) << ", " << (#X3) << " " << (X3) << std::endl;

#define ROFL_VAR4(X1,X2,X3,X4) std::cout << ROFL_FILENAME(__FILE__) << "," << __LINE__ << ": " << (#X1) << " " << (X1) \
   << ", " << (#X2) << " " << (X2) << ", " << (#X3) << " " << (X3) << ", " << (#X4) << " " << (X4) << std::endl;

#define ROFL_VAR5(X1,X2,X3,X4,X5) std::cout << ROFL_FILENAME(__FILE__) << "," << __LINE__ << ": " << (#X1) << " " << (X1) \
   << ", " << (#X2) << " " << (X2) << ", " << (#X3) << " " << (X3) << ", " << (#X4) << " " << (X4) \
   << ", " << (#X5) << " " << (X5) << std::endl;

#define ROFL_VAR6(X1,X2,X3,X4,X5,X6) std::cout << ROFL_FILENAME(__FILE__) << "," << __LINE__ << ": " << (#X1) << " " << (X1) \
   << ", " << (#X2) << " " << (X2) << ", " << (#X3) << " " << (X3) << ", " << (#X4) << " " << (X4) \
   << ", " << (#X5) << " " << (X5) << ", " << (#X6) << " " << (X6) << std::endl;

#define ROFL_VAR7(X1,X2,X3,X4,X5,X6,X7) std::cout << ROFL_FILENAME(__FILE__) << "," << __LINE__ << ": " << (#X1) << " " << (X1) \
   << ", " << (#X2) << " " << (X2) << ", " << (#X3) << " " << (X3) << ", " << (#X4) << " " << (X4) \
   << ", " << (#X5) << " " << (X5) << ", " << (#X6) << " " << (X7) << ", " << (#X7) << " " << (X7) << std::endl;

#define ROFL_ASSERT(COND) if (!(COND)) { std::cerr << __FILE__ << "," << __LINE__ << ": assertion failed on " << #COND << std::endl; exit(-1); }

#define ROFL_ASSERT_VAR1(COND,X1) \
   if (!(COND)) { std::cerr << __FILE__ << "," << __LINE__ << ": assertion failed on " << #COND << std::endl; \
   ROFL_VAR1(X1); exit(-1); }

#define ROFL_ASSERT_VAR2(COND,X1,X2) \
   if (!(COND)) { std::cerr << __FILE__ << "," << __LINE__ << ": assertion failed on " << #COND << std::endl; \
   ROFL_VAR2(X1,X2); exit(-1); }

#define ROFL_ASSERT_VAR3(COND,X1,X2,X3) \
   if (!(COND)) { std::cerr << __FILE__ << "," << __LINE__ << ": assertion failed on " << #COND << std::endl; \
   ROFL_VAR3(X1,X2,X3); exit(-1); }

#define ROFL_ASSERT_VAR4(COND,X1,X2,X3,X4) \
   if (!(COND)) { std::cerr << __FILE__ << "," << __LINE__ << ": assertion failed on " << #COND << std::endl; \
   ROFL_VAR4(X1,X2,X3,X4); exit(-1); }

#define ROFL_ASSERT_VAR5(COND,X1,X2,X3,X4,X5) \
   if (!(COND)) { std::cerr << __FILE__ << "," << __LINE__ << ": assertion failed on " << #COND << std::endl; \
   ROFL_VAR5(X1,X2,X3,X4,X5); exit(-1); }

#endif
