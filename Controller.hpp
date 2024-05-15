#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "TrackGraph.hpp"
#include "Train.hpp"
#include <vector>


/*
    * Controller class is responsible for creating and controlling trains
    * It is responsible for creating trains and controlling the signal lights based on the knowledge 
    * of the track graph and the current state of the trains
*/
class Controller{
private:
    TrackGraph &trackGraph;
    std::vector<Train> &trains;
    std::vector<int> bannedVertices; //next positions of the trains
public:
    Controller(TrackGraph &trackGraph, std::vector<Train> &trains){
        this->trackGraph = trackGraph;
        this->trains = trains;
    }

    int findBestRoute(Train &train){
        return trackGraph.findShortestPath(train.getSource(), train.getDestination(), train.getRoute());
    }
    
    void controlSignals(){
        bannedVertices.clear();
        //control signal lights based on the current state of the trains
        for(Train &train : trains){
            bannedVertices.push_back(train.getCurrentPosition());
            //TODO - set all of the signal to this vertex to not passable
            
        }

        for(Train &train : trains){
            //check if the train's next position is not banned
            if(std::find(bannedVertices.begin(), bannedVertices.end(), train.getNextPosition()) == bannedVertices.end()){
                //TODO - set the signal to passable
                bannedVertices.push_back(train.getNextPosition());
            }
        }
    }
    
};

#endif // 