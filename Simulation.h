#ifndef SIMULATION_H
#define SIMULATION_H

#include "Train.h"
#include "Controller.h"
#include "TrackGraph.h"
#include <memory>

class Simulation{
public:
    Simulation();
    void init();
    void start();
    void stop();
    void createTrain(TrackSegment *start, Direction direction, Terminator *destination);
    void createRandmoLinks();
private:
    std::unique_ptr<TrackGraph> trackGraph;
    std::unique_ptr<Controller> controller;
    std::vector<Train*> trains;
};

#endif