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
#include <iostream>
#include <array>
#include <vector>
#include <rofl/geometry/types.h>
#include <rofl/geometry/base_geometry2.h>
#include <rofl/geometry/correspondence_graph_association.h>
#include <rofl/common/param_map.h>

int main(int argc, char** argv) {
    rofl::CorrespondenceGraphAssociation corr;
    rofl::VectorVector2 pointsSrc, pointsDst; 
    rofl::Transform2 transf;
    rofl::Scalar distMin, distTol;
    rofl::ParamMap params;
    std::string filenameCfg;
    
    params.read(argc, argv);
    params.getParam<std::string>("cfg", filenameCfg, std::string(""));
    if (!params.read(filenameCfg)) {
        std::cout << "Cannot open configuration file \"" << filenameCfg << "\": using default values" << std::endl;
    }
    params.read(argc, argv);

    // Read params
    params.getParam<rofl::Scalar>("distMin", distMin, 0.0);
    params.getParam<rofl::Scalar>("distTol", distTol, 0.1);
    
    std::cout << "\nParams:\n";
    params.write(std::cout);
    std::cout << std::endl;
    
   
    rofl::poseToIsometry2(-6.0f, -2.0, 143.0 / 180.0 * rofl::K_PI, transf);
    
    std::cout << "Filling example source and destination point sets" << std::endl;
    
    pointsSrc.push_back(rofl::Vector2(1.5f, 0.8));
    pointsSrc.push_back(rofl::Vector2(2.0f, 2.0));
    pointsSrc.push_back(rofl::Vector2(-1.0f, 5.0));
    pointsSrc.push_back(rofl::Vector2(4.0f, 3.0f));
    pointsSrc.push_back(rofl::Vector2(1.0f, 4.5f));
    
    for (auto& p : pointsSrc) {
        pointsDst.push_back(transf * p);
    }
    
    pointsSrc.push_back(rofl::Vector2(3.0f, 3.0f));
    pointsSrc.push_back(rofl::Vector2(-0.5f, -1.0f));
    
    pointsDst.push_back(transf * rofl::Vector2(3.5f, 2.5f));
    pointsDst.push_back(transf * rofl::Vector2(1.5f, 2.5f));
            
    std::cout << "pointsSrc:\n";
    for (auto& p : pointsSrc) {
        std::cout << "  [" << p.transpose() << "]\n";
    }
    std::cout << "pointsDst:\n";
    for (auto& p : pointsDst) {
        std::cout << "  [" << p.transpose() << "]\n";
    }
    
    corr.setDistanceMin(distMin);
    corr.setTolerance(distTol);
    
    auto norm2 = [](const rofl::Vector2& v0, const rofl::Vector2& v1) -> rofl::Scalar {
        return (v1 - v0).norm();
    };
    corr.insertSrc(pointsSrc.begin(), pointsSrc.end(), norm2);
    corr.insertDst(pointsDst.begin(), pointsDst.end(), norm2);
    
    //std::vector<rofl::LabelSet> cliquesMax;
    //corr.associate(cliquesMax);
    //std::cout << "\nmaximum cliques:\n";
    //for (auto& clique : cliquesMax) {
    //    std::cout << clique << "\n";
    //}
    std::vector<rofl::CorrespondenceGraphAssociation::AssociationHypothesis> associationsBest;
    corr.associate(associationsBest);
    for (int k = 0; k < associationsBest.size(); ++k) {
        std::cout << "---\nassocitation best " << k << ":\n";
        for (auto& p : associationsBest[k].associations) {
            int isrc = p.first;
            int idst = p.second;
            std::cout << "  isrc " << isrc << ", idst " << idst << "\n";
        }
        
        for (int i = 0; i < associationsBest[k].associations.size(); ++i) {
            int is0 = associationsBest[k].associations[i].first;
            int id0 = associationsBest[k].associations[i].second;
            for (int j = i+1; j < associationsBest[k].associations.size(); ++j) {
                int is1 = associationsBest[k].associations[j].first;
                int id1 = associationsBest[k].associations[j].second;
                
                std::cout << "  src dist(" << is0 << "," << is1 << ") " << (pointsSrc[is0] - pointsSrc[is1]).norm() << "; "
                        << "  dst dist(" << id0 << "," << id1 << ") " << (pointsDst[id0] - pointsDst[id1]).norm() << "\n";
            }
        }
    }
    
    return 0;
}

