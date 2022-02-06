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
#ifndef ROFL_GEOMETRY_MAXIMUM_CLIQUE_ASSOCIATION_H_
#define ROFL_GEOMETRY_MAXIMUM_CLIQUE_ASSOCIATION_H_

#include <vector>
#include <deque>
#include <Eigen/Dense>
#include <rofl/geometry/types.h>
#include <rofl/common/macros.h>

namespace rofl {

    /**
     * Class CorrespondenceGraphAssociation associates points/items of a source
     * set with their corresponding points/items of the destination set. 
     * The association criterion is internal coherence between two sets. 
     * Some more detail:
     * 1) let ps[is] and pd[id] be the source and destination sets where 
     *    is = 0, ..., numSrc-1 and id = 0, ..., numDst-1;
     * 2) pair distances inside each set are defined as
     *    distSrc[is0][is1] = distance(ps[is0], ps[is1])  and
     *    distDst[is0][is1] = distance(pd[id0], ps[id1]);
     * 3) the associated items (is[k], id[k]) must be mutually coherent, i.e.
     *    (is[k0], id[k0]) and (is[k1], id[k1]) are coherent if
     *    distSrc[is[k0]][is[k1]] ~= distDst[id[k0]][id[k1]]. 
     * The above problem is equivalent to finding the mamimum clique in 
     * correspondence graph as described in:
     * 
     * - T. Bailey, "Mobile Robot Localisation and Mapping in Extensive Outdoor 
     *   Environments",  PhD Thesis, University of Sydney, 2002.
     * - T. Bailey, E. M. Nebot, J. K. Rosenblatt and H. F. Durrant-Whyte, 
     *   "Data Association for Mobile Robot Navigation: A Graph Theoretic Approach",
     *   IEEE ICRA 2001
     *    
     */
    class CorrespondenceGraphAssociation {
    public:

        /**
         * Struct PairDistance stores the distance between two items of 
         * the same set represented by indices/identifiers i0 and i1. 
         */
        struct PairDistance {
            int i0;
            int i1;
            Scalar distance;

            PairDistance(int _i0, int _i1, Scalar _distance) : i0(_i0), i1(_i1), distance(_distance) {
            }
        };
        using PairDistanceVector = std::vector<PairDistance>;

        struct CorrespondenceNode {
            int index;
            int isrc;
            int idst;
            std::vector<int> adjacents;
        };
        using VectorCorrespondenceNode = std::vector<CorrespondenceNode>;

        /**
         * Default constructor. 
         */
        CorrespondenceGraphAssociation();

        /**
         * Destructor. 
         */
        virtual ~CorrespondenceGraphAssociation();

        /**
         * Sets the tolerance on distance. 
         * @param 
         */
        void setTolerance(Scalar tol);
        
        /**
         * Sets the minimum distance. 
         * @param 
         */
        void setDistanceMin(Scalar dmin);

        /**
         * Inserts the source items with a distance functor.
         * The distance functor must have operator() s.t.
         * 
         *    dist(*it0, *it1) -> return rofl::Scalar type
         * 
         * where it0, it1 = beg, beg+1, ..., end. 
         * 
         * @param beg iterator pointing to the beginning of item container
         * @param end iterator pointing to the end of item container
         * @param dist a distance functor returning the distance between two items
         */
        template <typename It, typename Dist>
        void insertSrc(It beg, It end, Dist dist) {
            numSrc_ = std::distance(beg, end);
            distancesSrc_.resize(numSrc_ * (numSrc_ - 1) / 2);
            insertPairDistances(beg, end, dist, std::back_inserter(distancesSrc_));

            std::sort(distancesSrc_.begin(), distancesSrc_.end(),
                    [&](const PairDistance& pd0, const PairDistance & pd1) -> bool {
                        return pd0.distance < pd1.distance;
                    });
        }

        /**
         * Inserts the source items with a distance functor.
         * The distance functor must have operator() s.t.
         * 
         *    dist(*it0, *it1) -> return rofl::Scalar type
         * 
         * where it0, it1 = beg, beg+1, ..., end. 
         * 
         * @param beg iterator pointing to the beginning of item container
         * @param end iterator pointing to the end of item container
         * @param dist a distance functor returning the distance between two items
         */
        template <typename It, typename Dist>
        void insertDst(It beg, It end, Dist dist) {
            numDst_ = std::distance(beg, end);
            distancesSrc_.resize(numDst_ * (numDst_ - 1) / 2);
            insertPairDistances(beg, end, dist, std::back_inserter(distancesDst_));
                    
            std::sort(distancesDst_.begin(), distancesDst_.end(),
                    [&](const PairDistance& pd0, const PairDistance & pd1) -> bool {
                        return pd0.distance < pd1.distance;
                    });
        }

        void associate();


    private:
        VectorCorrespondenceNode nodes_;
        PairDistanceVector distancesSrc_;
        PairDistanceVector distancesDst_;
        size_t numSrc_;
        size_t numDst_;
        Scalar distTol_;
        Scalar distMin_;

        template <typename It, typename Dist, typename Ins>
        static void insertPairDistances(It beg, It end, Dist distance, Ins inserter) {
            Scalar d;
            int i0, i1;
            i0 = 0;
            for (It it0 = beg; it0 != end; ++it0, ++i0) {
                i1 = i0 + 1;
                for (It it1 = std::next(it0, 1); it1 != end; ++it1, ++i1) {
                    d = distance(*it0, *it1);
                    inserter = PairDistance(i0, i1, d);
                }
            }
        }
    };

} // end of namespace

#endif /* MAXIMUM_CLIQUE_ASSOCIATION_H */

