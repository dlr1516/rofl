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
#ifndef ROFL_COMMON_MORTON_TREE_H_
#define ROFL_COMMON_MORTON_TREE_H_

#include <rofl/common/bit_manip.h>
#include <rofl/common/macros.h>
#include <rofl/common/morton_sort.h>
#include <rofl/common/numeric_traits.h>

#include <algorithm>
#include <deque>
#include <iostream>
#include <sstream>
#include <vector>

namespace rofl {

template <typename Scalar, size_t Dim>
class MortonTree {
   public:
    /**
     * @brief Structure containing vector items.
     */
    struct Item {
        Scalar values[Dim];
        int id;

        void print(std::ostream& out) const {
            out << "[";
            for (int d = 0; d < Dim; ++d) {
                if (d != 0)
                    out << ",";
                out << values[d];
            }
            out << "](id " << id << ")";
        }

        std::string toString() const {
            std::stringstream ss;
            print(ss);
            return ss.str();
        }
    };
    using Traits = MortonTraits<Scalar, Dim>;
    using Container = std::vector<Item>;
    using Iterator = typename Container::iterator;
    using ConstIterator = typename Container::const_iterator;
    using IteratorContainer = std::vector<ConstIterator>;
    using TreeInterval = std::pair<ConstIterator, ConstIterator>;

    /**
     * @brief Construct a new Morton Tree object
     */
    MortonTree() : items_(), ordered_(true) {}

    /**
     * @brief Construct a new Morton Tree object as a copy.
     * @param mt Morton tree to be copied
     */
    MortonTree(const MortonTree<Scalar, Dim>& mt) = default;

    /**
     * @brief Destroy the Morton Tree object
     */
    virtual ~MortonTree() = default;

    /**
     * @brief Returns the numer of items in the tree.
     *
     * @return size_t
     */
    size_t size() const {
        return items_.size();
    }

    /**
     * @brief Returns the iterator pointing to the first position of the tree.
     *
     * @return ConstIterator
     */
    ConstIterator begin() const { return items_.begin(); }

    /**
     * @brief Returns the iterator pointing to the last position of the tree.
     *
     * @return ConstIterator
     */
    ConstIterator end() const { return items_.end(); }

    /**
     * @brief Get a reference to the internal container object with items.
     *
     * @return const Container&
     */
    const Container& getItems() const { return items_; }

    /**
     * @brief Returns the iterator pointing to the item that is immediatly
     * before the query point in Morton order.
     *
     * @param q the query point
     * @return ConstIterator
     */
    ConstIterator findItem(const Item& q) {
        if (!ordered_) {
            sort();
        }
        // ConstIterator it =
        // std::lower_bound(std::begin(items_), std::end(items_), q,
        //                  [&](const Item& item1, const Item& item2) -> bool {
        //                      return Traits::compare(item1.values, item2.values);
        //                  });
        return std::lower_bound(std::begin(items_), std::end(items_), q, compareItems);
    }

    /**
     * @brief Computes the level of the minimum subtree containing the points
     * in the interval between the two iterators [first, last[.
     *
     * @param first iterator to first point of interval
     * @param last iterator to last position of interval
     * @return int the level of the subtree
     */
    int computeLevel(ConstIterator first, ConstIterator last) {
        if (!ordered_) {
            sort();
        }

        // Iterator last points to the position after the last item of interval
        // (conventionally, iterator intervals are open on last element).
        // To compute the interval we need the last item: so we decrease the
        // iterator.
        if (last != first) {
            --last;
        }
        return Traits::distance(first->values, last->values);
    }

    /**
     * @brief Inserts a new item in the tree.
     * If the resetId is set true, the id of the new item follows
     * the increasing order.
     *
     * @param item the item to be inserted
     * @param resetId the flag is set if the id of input item is
     * the insertion order into the tree.
     */
    void insert(const Item& item, bool resetId = true) {
        Item itemCopy;
        std::copy(item.values, item.values + Dim, itemCopy.values);
        if (resetId) {
            itemCopy.id = items_.size();
        }
        items_.push_back(itemCopy);
        ordered_ = false;
    }

    /**
     * @brief Inserts the items in
     *
     * @tparam It iterator type for a container of MortonTree<Scalar,Dim>::Item.
     * @param beg iterator pointing to the first position in input container
     * @param end iterator pointing to the last position in input container
     * @param resetId the flag is set if the id of input item is
     * the insertion order into the tree.
     */
    template <typename It>
    void insert(It beg, It end, bool resetId = true) {
        size_t sizeCur = items_.size();
        size_t sizeInc = std::distance(beg, end);
        int id;

        items_.reserve(sizeInc);
        id = sizeCur;
        for (It it = beg; it != end; ++it, ++id) {
            if (resetId) {
                it->id = id;
            }
            insert(*it, resetId);
        }
        ordered_ = false;
    }

    /**
     * @brief Inserts the items from a container, coverts each of them
     * into a MortonTree<Scalar, Dim>::Item and inserts it into the tree.
     *
     * @tparam It iterator type for a container containing the items
     *   to be inserted.
     *   The type may or may not be the the MortonTree::Item.
     *   If not, the user must set the proper Converter operator.
     * @tparam Converter is a functor with the following method:
     *    MortonTree<Scalar, Dim>::Item operator()(const It::value_type& val)
     * @param beg iterator pointing to the first position in input container
     * @param end iterator pointing to the last position in input container
     * @param converter the functor object
     * @param resetId the flag is set if the id of input item is
     * the insertion order into the tree.
     */
    template <typename It, typename Converter>
    void insert(It beg, It end, Converter& converter, bool resetId = true) {
        Item item;
        Scalar* ref;
        size_t sizeInc = std::distance(beg, end);
        size_t sizeCur = items_.size();

        items_.reserve(sizeInc);
        for (It it = beg; it != end; ++it) {
            item = converter(*it);
            if (resetId) {
                item.id = items_.size();
            }
            items_.push_back(item);
        }
        ordered_ = false;
    }

    /**
     * @brief Sorts the tree in Morton order.
     */
    void sort() {
        // Sorts in morton order
        std::sort(std::begin(items_), std::end(items_),
                  [&](const Item& item1, const Item& item2) -> bool {
                      return Traits::compare(item1.values, item2.values);
                  });
        ordered_ = true;
    }

    void searchInBox(const Item& boxMin, const Item& boxMax, std::vector<ConstIterator>& items) {
        using TreeIntervalQueue = std::deque<TreeInterval>;
        TreeIntervalQueue queue;
        TreeInterval cur, low, upp;
        ConstIterator itLow, itMid, itUpp;

        if (!ordered_) {
            sort();
        }

        ROFL_VAR2(boxMin.toString(), boxMax.toString());

        cur.first = std::lower_bound(std::begin(items_), std::end(items_), boxMin, compareItems);
        cur.second = std::upper_bound(std::begin(items_), std::end(items_), boxMax, compareItems);
        queue.push_back(cur);

        ROFL_MSG("initial interval " << cur.first->toString() << " : " << cur.second->toString());

        int count = 0;
        while (!queue.empty() && count < 10) {
            cur = queue.front();
            queue.pop_front();
            std::cout << "\n---\nEXPLORING " << printInter(cur) << std::endl;
            findTreeBucket(cur.first, cur.second, itLow, itMid, itUpp);
            ROFL_MSG("splitting\n"
                     << "  low " << printIt(itLow)
                     << "  mid " << printIt(itMid)
                     << "  upp " << printIt(itUpp));
            if (cur.first == itLow && cur.second == itUpp) {
                for (auto it = itLow; it != itUpp; ++it) {
                    items.push_back(it);
                }
            } else {
                low.first = cur.first;
                low.second = itMid;
                upp.first = itMid;
                upp.second = cur.second;
                ROFL_VAR2(printInter(low), printInter(upp));
                if (upp.first != upp.second)
                    queue.push_back(upp);
                if (low.first != low.second)
                    queue.push_back(low);
            }
            count++;
        }
    }

    void searchInBox(const Item& boxCenter, const Scalar& boxRange, std::vector<ConstIterator>& items) {
        Item boxMin, boxMax;
        for (int d = 0; d < Dim; ++d) {
            boxMin.values[d] = boxCenter.values[d] - boxRange;
            boxMax.values[d] = boxCenter.values[d] + boxRange;
        }
        searchInBox(boxMin, boxMax, items);
    }

    // void searchKNearest(const Scalar* q,
    //                     int k,
    //                     std::vector<int>& indices,
    //                     std::vector<Scalar>& distances) const {}

   private:
    Container items_;
    volatile bool ordered_;

    static bool compareItems(const Item& item1, const Item& item2) {
        return Traits::compare(item1.values, item2.values);
    }

    void findTreeBucket(ConstIterator& first, ConstIterator& last, ConstIterator& low, ConstIterator& mid, ConstIterator& upp) {
        Item boxLow, boxMid, boxUpp;
        if (last != first)
            --last;
        Traits::split(first->values, last->values, boxLow.values, boxMid.values, boxUpp.values);
        ROFL_VAR3(boxLow.toString(), boxMid.toString(), boxUpp.toString());
        low = std::lower_bound(std::begin(items_), std::end(items_), boxLow, compareItems);
        mid = std::lower_bound(std::begin(items_), std::end(items_), boxMid, compareItems);
        // for (auto it = std::begin(items_); it != std::end(items_); ++it) {
        //     ROFL_VAR3(it->toString(), boxUpp.toString(), compareItems(*it, boxUpp));
        // }
        upp = std::upper_bound(std::begin(items_), std::end(items_), boxUpp, compareItems);
        ROFL_VAR4(low->toString(), mid->toString(), upp->toString(), upp == std::end(items_));
        ROFL_VAR3(std::distance(std::cbegin(items_), low),
                  std::distance(std::cbegin(items_), mid),
                  std::distance(std::cbegin(items_), upp));
    }

    std::string printIt(ConstIterator it) const {
        if (it != items_.end()) {
            return it->toString();
        } else {
            return "END";
        }
    }

    std::string printInter(TreeInterval interval) const {
        std::stringstream ss;
        ss << "{ " << printIt(interval.first) << " : " << printIt(interval.second) << " }";
        return ss.str();
    }
};

using MortonTree2i = MortonTree<int, 2>;
using MortonTree2l = MortonTree<long, 2>;
using MortonTree2f = MortonTree<float, 2>;
using MortonTree2d = MortonTree<double, 2>;

using MortonTree3i = MortonTree<int, 3>;
using MortonTree3l = MortonTree<long, 3>;
using MortonTree3f = MortonTree<float, 3>;
using MortonTree3d = MortonTree<double, 3>;

}  // namespace rofl

template <typename Scalar, size_t Dim>
std::ostream& operator<<(std::ostream& out, const typename rofl::MortonTree<Scalar, Dim>::Item& item) {
    item.print(out);
    return out;
}

#endif