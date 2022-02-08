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

    void CorrespondenceGraphAssociation::associate(std::vector<LabelSet>& cliquesMax) {
        //        auto cmp = [](const LabelSet& clique0,const LabelSet& clique1) -> bool {
        //            return clique0.size() < clique1.size();
        //        };
        //        rofl::MinMaxHeap<LabelSet, decltype(cmp)> queue(cmp);
        std::deque<LabelSet> queue;
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

        std::deque<int> nodeList(nodes_.size());
        std::iota(nodeList.begin(), nodeList.end(), 0);
        std::sort(nodeList.begin(), nodeList.end(),
                [&](int n0, int n1) ->bool {
                    return nodes_[n0].adjacents.size() > nodes_[n1].adjacents.size();
                });

        for (auto& node0 : nodeList) {
            candidates = nodes_[node0].adjacents.unionSet(node0);
            for (auto& node1 : nodes_[node0].adjacents) {
                candidates = candidates.intersectionSet(nodes_[node0].adjacents.unionSet(node1));
            }
            std::cout << "maximum clique with node " << node0 << ": " << candidates << "\n";

            if (cliquesMax.empty() || candidates.size() == cliquesMax.back().size()) {
                cliquesMax.push_back(candidates);
                ROFL_VAR2(cliquesMax.back().size(), candidates);
            } else if (candidates.size() > cliquesMax.back().size()) {
                cliquesMax.clear();
                cliquesMax.push_back(candidates);
                ROFL_VAR2(cliquesMax.back().size(), candidates);
            }
        }
        ROFL_VAR2(cliquesMax.size(), cliquesMax.back().size());
    }

    void CorrespondenceGraphAssociation::associate(std::vector<AssociationHypothesis>& associations) {
        std::vector<LabelSet> cliquesMax;
        AssociationHypothesis assoc;

        associate(cliquesMax);

        associations.clear();
        for (auto& c : cliquesMax) {
            convertToAssociation(c, assoc);
            associations.push_back(assoc);
        }
    }
    
    void CorrespondenceGraphAssociation::convertToAssociation(const LabelSet& correspNodes, AssociationHypothesis& assocHyp) const {
        assocHyp.associations.clear();
        for (auto& node : correspNodes) {
            std::cout << "  node " << node << ": isrc " << nodes_[node].isrc << " idst " << nodes_[node].idst << std::endl;
            assocHyp.associations.push_back(std::make_pair(nodes_[node].isrc, nodes_[node].idst));
        }
        
//        for (int i = 0; i < correspNodes.size(); ++i) {
//            int is0 = nodes_[i].isrc;
//            int id0 = nodes_[i].idst;
//            for (int i = i+1; j < correspNodes.size(); ++j) {
//                int is1 = nodes_[j].isrc;
//                int id1 = nodes_[j].idst;
//            }
//        }
    }

} // end of namespace
