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
#include <rofl/common/morton_tree.h>

namespace rofl {

template class MortonTree<int, 2>;
template class MortonTree<float, 2>;
template class MortonTree<double, 2>;

template class MortonTree<int, 3>;
template class MortonTree<float, 3>;
template class MortonTree<double, 3>;

}  // namespace rofl