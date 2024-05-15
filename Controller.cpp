#include "Controller.h"

Controller::Controller(TrackGraph* trackGraph, std::vector<Train*>* trains){
    this->trackGraph = trackGraph;
    this->trains = trains;
}


void Controller::calculateBestRoutes(){
    //TODO: implement
}

void Controller::controlSignal(){
    //TODO - implement
}