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
#include <rofl/geometry/correspondence_graph_association.h>
#include <rofl/common/min_max_heap.h>
#include <numeric>

#include "rofl/common/min_max_heap.h" // for std::iota

namespace rofl {
    
    CorrespondenceGraphAssociation::CorrespondenceGraphAssociation() 
    : nodes_(), distancesSrc_(), distancesDst_(), numSrc_(0), numDst_(0), distTol_(0.1), distMin_(0.0) {
    }
    
    CorrespondenceGraphAssociation::~CorrespondenceGraphAssociation() {
    }
    
    void CorrespondenceGraphAssociation::setTolerance(Scalar tol) {
        distTol_ = tol;
    }
    
    void CorrespondenceGraphAssociation::setDistanceMin(Scalar dmin) {
        distMin_ = dmin;
    }
    
    void CorrespondenceGraphAssociation::associate() {
        auto cmp = [](const LabelSet& clique0,const LabelSet& clique1) -> bool {
            return clique0.size() < clique1.size();
        };
        rofl::MinMaxHeap<LabelSet, decltype(cmp)> queue(cmp);
        LabelSet parentClique, childClique, candidates;
        CorrespondenceNode node;
        int isrc0, isrc1, idst0, idst1, index0, index1;
        
        // Creates the list of correspondence nodes.
        // Each node represents an association between a source and a destination
        // point/item. 
        nodes_.reserve(numSrc_ * numDst_);
        node.adjacents.clear();
        for (int is = 0; is < numSrc_; ++is) {
            node.isrc = is;
            for (int id = 0; id < numDst_; ++id) {
                node.idst = id;
                node.index = is * numDst_ + id;
                nodes_.push_back(node);
            }
        }
        
        // Adds an edge between compatible correspondence nodes
        auto begSrc = std::upper_bound(distancesSrc_.begin(), distancesSrc_.end(), distMin_,
                [](const Scalar& dmin, const PairDistance & pd) -> bool {
                    return dmin < pd.distance;
                });
        for (auto itSrc = begSrc; itSrc != distancesSrc_.end(); ++itSrc) {
            auto begDst = std::upper_bound(distancesDst_.begin(), distancesDst_.end(), itSrc->distance - distTol_,
                [](const Scalar& dmin, const PairDistance & pd) -> bool {
                    return dmin < pd.distance;
                });
            isrc0 = itSrc->i0;
            isrc1 = itSrc->i1;
            Scalar distMax = itSrc->distance + distTol_;
            for (auto itDst = begDst; itDst != distancesDst_.end() && itDst->distance < distMax; ++itDst) {
                idst0 = itDst->i0;
                idst1 = itDst->i1;
                index0 = isrc0 * numDst_ + idst0; 
                index1 = isrc1 * numDst_ + idst1; 
                ROFL_ASSERT_VAR4(index0 < nodes_.size(), index0, isrc0, idst0, numDst_);
                ROFL_ASSERT_VAR4(index1 < nodes_.size(), index1, isrc1, idst1, numDst_);
                nodes_[index0].adjacents.insert(index1);
                nodes_[index1].adjacents.insert(index0);
                index0 = isrc0 * numDst_ + idst1; 
                index1 = isrc1 * numDst_ + idst0; 
                ROFL_ASSERT_VAR4(index0 < nodes_.size(), index0, isrc0, idst1, numDst_);
                ROFL_ASSERT_VAR4(index1 < nodes_.size(), index1, isrc1, idst0, numDst_);
                nodes_[index0].adjacents.insert(index1);
                nodes_[index1].adjacents.insert(index0);
            }
        }
                
        ROFL_MSG("nodes:");
        for (auto& n : nodes_) {
            //std::sort(n.adjacents.begin(), n.adjacents.end());
            std::cout << "  node " << n.index << ": isrc " << n.isrc << " idst " << n.idst << " compatible with " << n.adjacents.size() << "\n";
        }
        
        // Initializes clique hypotheses queue
        std::cout << "Initialize cliques:\n";
        for (int i = 1; i < nodes_.size(); ++i) {
            childClique = parentClique.unionSet(i);
            childClique.print(std::cout);
            queue.push(childClique);
        }
                
        // Brute-force visits the cliques
        while (!queue.empty()) {
            parentClique = queue.top();
            queue.popTop(); 
            
            // Tries to expand the parentClique as follows.
            // 1) For each node l in parentClique, it finds the adjacent nodes 
            //    and put them into candidates set. 
            // 2) Removes from candidates all the nodes already in parentClique. 
            // 3) Repeat 
            candidates.clear();
            for (auto it = parentClique.begin(); it != parentClique.end(); ++it) {
                int l = *it;
                if (it == parentClique.begin()) {
                    candidates = nodes_[l].adjacents.differenceSet(parentClique);
                }
                else {
                    candidates = candidates.intersectionSet(nodes_[l].adjacents); 
                }
            }
            for (auto& candidate : candidates) {
                childClique = parentClique.unionSet(candidate);
            }
//            for (auto& item : parentClique) {
//                std::set_difference(nodes_[item].adjacents.begin(), nodes_[item].adjacents.end(), 
//                        );
//            }
        }
        
    }

} // end of namespace
