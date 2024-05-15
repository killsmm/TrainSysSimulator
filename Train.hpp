#ifndef TRAIN_H
#define TRAIN_H

#include "TrackGraph.hpp"
#include <list>
#include <vector>

using namespace std;

class Train{
private:
    int source;
    int destination;
    int currentPosition;
    TrackGraph *graph;
    list<int> route;
    bool isArrived = false;


public:
    Train(int source, int destination, TrackGraph *graph){
        this->source = source;
        this->currentPosition = source;
        this->destination = destination;
        this->graph = graph;
    }



    void move(){
        //move to the next position in the route
        if(isArrived){
            return;
        }
        int from = currentPosition;
        int to = route.front();
        if(graph->checkPassable(from, to)){
            currentPosition = to;
            route.pop_front();
            if(currentPosition == this->destination){
                isArrived = true;
            }
        }
    }

    bool hasArrived(){
        return isArrived;
    }

    int getCurrentPosition(){
        return currentPosition;
    }

    int getNextPosition(){
        if(route.size() > 0){
            return route.front();
        }
        return -1;
    }

    int getSource(){
        return source;
    }

    int getDestination(){
        return destination;
    }

    list<int>* getRoute(){
        return &(this->route);
    }
};

#endif