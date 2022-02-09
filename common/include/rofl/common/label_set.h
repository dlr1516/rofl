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
#ifndef ROFL_COMMON_LABEL_SET_H_
#define ROFL_COMMON_LABEL_SET_H_

#include <iostream>
#include <set>
#include <algorithm>


namespace rofl {

    /**
     * Class LabelSet handles a set of labels/indices.
     */
    class LabelSet {
    public:
        using Label = int;
        using Iterator = std::set<int>::iterator;

        /**
         * Default constructor of an empty set.
         */
        LabelSet();

        /**
         * Copy constructor. 
         * @param c
         */
        LabelSet(const LabelSet& ls);
        
        template <typename It>
        LabelSet(It beg, It end) : labels_(beg, end) {
        }

        /**
         * Drstructor.
         */
        virtual ~LabelSet();
        
        /**
         * Clear the content of label set. 
         */
        void clear();
        
        /**
         * Returns the size of the set. 
         */
        size_t size() const;
        

        /**
         * Returns an iterator pointing to the first element of set. 
         */
        Iterator begin() const;

        /**
         * Returns an iterator pointing to the end of set. 
         */
        Iterator end() const;

        /**
         * Says if the given label is inside.
         * @param label the label to be checked
         */
        bool inside(Label label) const;
        
        /**
         * Compares the content of this label set with the given one. 
         * @param ls the other label set to be compared
         * @return true if the two label sets are equal 
         */
        bool operator==(const LabelSet& ls) const;
        
        /**
         * Compares the content of this label set with the given one. 
         * @param ls the other label set to be compared
         * @return true if the two label sets are NOT equal 
         */
        bool operator!=(const LabelSet& ls) const;

        /**
         * Inserts the label in the set.
         * @param label the label to be added. 
         * @return 
         */
        bool insert(Label label);

        /**
         * Returns a new label set that is the unin of this set and the given 
         * label.
         * @param label the label to be added
         * @return the union set
         */
        LabelSet unionSet(Label label) const;
        
        /**
         * Returns a new label set that is the union of this and given label sets. 
         * @param ls the given label set
         * @return the union of two label sets
         */
        LabelSet unionSet(const LabelSet& ls) const;
        
        /**
         * Returns a new label set that is the intersection of this and given label sets. 
         * @param ls the given label set
         * @return the intersection of two label sets
         */
        LabelSet intersectionSet(const LabelSet& ls) const;
        
        /**
         * Returns a new label set that is the unin of this set and the given 
         * label.
         * @param label the label to be added
         * @return the union set
         */
        LabelSet differenceSet(const LabelSet& ls) const;

        /**
         * Prints the content of the set to output streams.
         * @param out an output stream 
         */
        void print(std::ostream& out) const;

    private:
        std::set<int> labels_;
    };

} // end of namespace

std::ostream& operator<<(std::ostream& out, const rofl::LabelSet& ls) {
    ls.print(out);
    return out;
}

#endif /* IDSET_H */

