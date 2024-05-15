// Copyright 2024 zhouxinliang
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     https://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "TrackGraph.h"
#include <memory>
#include <iostream>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>





// Link::Link(Node* nodeA, Node* nodeB, LinkType type){
//     this->nodeA = nodeA;
//     this->nodeB = nodeB;
//     this->type = type;
// }

// TrackSegment::TrackSegment(int id){
//     this->id = id;
//     this->nodeA.parent = this;
//     this->nodeB.parent = this;
// }

// Terminator::Terminator(int id){
//     this->id = id;
//     this->node.parent = this;
// }

// Terminator* TrackGraph::addTerminator(int id){
//     Terminator* term = new Terminator(id);
//     for(auto terminator : this->terminators){
//         /* make sure the id is unique */
//         if (terminator->id == id){
//             return nullptr;
//         }
//     }
//     terminators.push_back(term);
//     return term;
// }

// TrackSegment* TrackGraph::addTrackSegment(int id){
//     TrackSegment* seg = new TrackSegment(id);
//     for(auto trackSegment : this->trackSegments){
//         /* make sure the id is unique */
//         if (trackSegment->id == id){
//             return nullptr;
//         }
//     }
//     trackSegments.push_back(seg);
//     return seg;
// }

// TrackSegment * TrackGraph::getTrackSegment(int id){
//     for(auto trackSegment : this->trackSegments){
//         if (trackSegment->id == id){
//             return trackSegment;
//         }
//     }
//     return nullptr;
// }

// Terminator * TrackGraph::getTerminator(int id){
//     for(auto terminator : this->terminators){
//         if (terminator->id == id){
//             return terminator;
//         }
//     }
//     return nullptr;
// }

// int TrackGraph::addLink(Node* nodeA, Node* nodeB, LinkType type){
    
//     for(auto link : this->links){
//         if ((link->nodeA == nodeA && link->nodeB == nodeB) || (link->nodeA == nodeB && link->nodeB == nodeA)){
//             /* if the link already exists, return 0*/
//             return 0;
//         }
//     }
    
//     this->links.push_back(new Link(nodeA, nodeB, type));

//     /* if the node has already been in a junction, add another one into the junction*/
//     for(auto junction : this->junctions){
//         for(auto node : *junction){
//             if (node == nodeA){
//                 junction->push_back(nodeB);
//                 return 0;
//             }
//             if (node == nodeB){
//                 junction->push_back(nodeA);
//                 return 0;
//             }
//         }
//     }

//     /* if the node has already been in a link, create a junction and put the nodes into the junction*/
//     for(auto link : this->links){
//         if (link->nodeA == nodeA || link->nodeA == nodeB){
//             Junction *junction = new Junction();
//             junction->push_back(nodeA);
//             junction->push_back(nodeB);
//             junction->push_back(link->nodeB);
//             nodeA->has_light = true;
//             nodeA->light_state = false;
//             nodeB->has_light = true;
//             nodeB->light_state = false;
//             link->nodeB->has_light = true;
//             link->nodeB->light_state = false;
//             this->junctions.push_back(junction);
//             return 0;
//         }
//         if (link->nodeB == nodeA || link->nodeB == nodeB){
//             Junction *junction = new Junction();
//             junction->push_back(nodeA);
//             junction->push_back(nodeB);
//             junction->push_back(link->nodeA);
//             nodeA->has_light = true;
//             nodeA->light_state = false;
//             nodeB->has_light = true;
//             nodeB->light_state = false;
//             link->nodeA->has_light = true;
//             link->nodeA->light_state = false;
//             this->junctions.push_back(junction);
//             return 0;
//         }
//     }
//     return 0;
// }

// void TrackGraph::printGraph(){
//     /* print junctions*/
//     std::cout << this->junctions.size() << " junctions" << std::endl;
// }
// 