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
#include <rofl/common/label_set.h>
#include <rofl/common/macros.h>

namespace rofl {

    LabelSet::LabelSet() {
    }

    LabelSet::LabelSet(const LabelSet& ls) : labels_(ls.labels_) {
    }

    LabelSet::~LabelSet() {
    }

    void LabelSet::clear() {
        labels_.clear();
    }

    size_t LabelSet::size() const {
        return labels_.size();
    }

    std::set<int>::iterator LabelSet::begin() const {
        return labels_.begin();
    }

    std::set<int>::iterator LabelSet::end() const {
        return labels_.end();
    }

    bool LabelSet::inside(Label label) const {
        return (labels_.find(label) != labels_.end());
    }
    
    bool LabelSet::operator==(const LabelSet& ls) const {
        return std::equal(labels_.begin(), labels_.end(), ls.labels_.begin());
    }
    
    bool LabelSet::operator!=(const LabelSet& ls) const {
        return !std::equal(labels_.begin(), labels_.end(), ls.labels_.begin());
    }

    bool LabelSet::insert(Label label) {
        bool itemIn = inside(label);
        if (!itemIn) {
            labels_.insert(label);
        }
        return itemIn;
    }

    LabelSet LabelSet::unionSet(Label label) const {
        LabelSet setNew(*this);
        setNew.insert(label);
        return setNew;
    }

    LabelSet LabelSet::unionSet(const LabelSet& ls) const {
        LabelSet setNew(*this);
        //setNew.labels_.insert(ls.begin(), ls.end());
        std::set_union(labels_.begin(), labels_.end(),
                ls.labels_.begin(), ls.labels_.end(),
                std::inserter(setNew.labels_, setNew.labels_.begin())); 
        return setNew;
    }

    LabelSet LabelSet::intersectionSet(const LabelSet& ls) const {
        //LabelSet setNew(*this);
        std::vector<Label> tmp;
        std::set_intersection(labels_.begin(), labels_.end(),
                ls.labels_.begin(), ls.labels_.end(),
                std::back_inserter(tmp));
                //std::inserter(setNew.labels_, setNew.labels_.begin())); 
//        ROFL_MSG("insersection: ");
//        for (auto& t : tmp) {
//            std::cout << t << ", ";
//        }
//        std::cout << std::endl;
        //setNew.print(std::cout);
        return LabelSet(tmp.begin(), tmp.end());
    }

    LabelSet LabelSet::differenceSet(const LabelSet& ls) const {
        //LabelSet setNew(*this);
        std::vector<Label> tmp;
        std::set_difference(labels_.begin(), labels_.end(),
                ls.labels_.begin(), ls.labels_.end(),
                std::back_inserter(tmp));
                //std::inserter(setNew.labels_, setNew.labels_.begin()));
        //ROFL_MSG("difference: ");
        //setNew.print(std::cout);
        return LabelSet(tmp.begin(), tmp.end());
    }

    void LabelSet::print(std::ostream& out) const {
        out << "{"; 
        for (auto it = labels_.begin(); it != labels_.end(); ++it) {
            if (it != labels_.begin()) {
                out << ", ";
            }
            out << *it;
        }
        out << "}";
    }

} // end of namespace

