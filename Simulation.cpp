#include "Simulation.h"
#include <memory>

#include "TrackGraph.h"
#include "Train.h"
#include "Controller.h"
#include <stdlib.h>


Simulation::Simulation(){

    
}

void Simulation::createRandmoLinks(){
    srand(time(NULL));
    int numLinks = rand() % 10;
    for(int i = 0; i < numLinks; i++){
        Node *node1 = &(this->trackGraph->trackSegments[rand() % this->trackGraph->trackSegments.size()]->nodeA);
        Node *node2 = &(this->trackGraph->trackSegments[rand() % this->trackGraph->trackSegments.size()]->nodeB);
        this->trackGraph->addLink(node1, node2, SEGMENT_SEGMENT);
    }
    for (auto terminator : this->trackGraph->terminators){
        Node *node1 = &(terminator->node);
        Node *node2 = &(this->trackGraph->trackSegments[rand() % this->trackGraph->trackSegments.size()]->nodeA);
        this->trackGraph->addLink(node1, node2, TERMINATOR_SEGMENT);
    }
}

void Simulation::init(){
    /*create a graph*/
    this->trackGraph = std::make_unique<TrackGraph>();

    for(int i = 0; i < 10; i++){
        this->trackGraph->addTrackSegment(i);
    }
    this->createRandmoLinks();

    this->trackGraph->printGraph();

    /*create trains*/
    this->trains.push_back(new Train(this->trackGraph->getTrackSegment(0), this->trackGraph->getTerminator(0)));

    /*create controller*/
    this->controller = std::make_unique<Controller>(this->trackGraph.get(), &(this->trains));

    // this->controller->calculateBestRoutes();
}

void Simulation::start(){
    while(true){
        for(auto train : this->trains){
            if(train->isMoving){
                break;
            }
        }
        for(auto train : this->trains){
            train->move();
        }
        this->controller->controlSignal();
    }
}