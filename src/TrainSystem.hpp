#ifndef TRAINSYSTEM_HPP
#define TRAINSYSTEM_HPP

#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <list>
#include <algorithm>
#include <iomanip>
#include "Train.hpp"
#include "TrackGraph.hpp"

class TrainSystem {
    std::unordered_map<int, std::unordered_set<int>> trackOccupancy;
    std::vector<Train> trains;
    TrackGraph &trackGraph;

    int findNextVertex(const Train& train, int vertex) {
        auto it = std::find(train.path.begin(), train.path.end(), vertex);
        if(it == train.path.end()){
            return -1;
        }
        auto nextIt = std::next(it);
        if (nextIt != train.path.end()) {
            return *nextIt;
        }
        return -1;
    }

    void updateTrackOccupancy(const Train& train) {
        int currentTime = train.startTime;
        for (int vertex : train.path) {
            trackOccupancy[vertex].insert(currentTime);
            //occupy next vertex on the path if it is not the last vertex
            int nextVertex = findNextVertex(train, vertex);
            if (nextVertex != -1) {
                trackOccupancy[nextVertex].insert(currentTime);
            }
            currentTime += 1; // increment time for next segment assuming uniform speed
        }
    }

    bool detectCollision(std::list<int> path, int currentPosition, int startTime){
        auto it = std::find(path.begin(), path.end(), currentPosition);
        int currentTime = startTime;

        for(; it != path.end(); it++){
            if(trackOccupancy[*it].find(currentTime) != trackOccupancy[*it].end() //current position is occupied
            || (std::next(it) != path.end() && trackOccupancy[*std::next(it)].find(currentTime) != trackOccupancy[*std::next(it)].end()) //next position is occupied
            ){
                return true;
            }
            currentTime += 1;
        }
        return false;
    }

    bool couldWaitToPass(std::list<int> &path, int currentTime, int currentPosition, int maxWaitTime, 
                    int &waitTime, std::unordered_map<int, std::unordered_set<int>> trackOccupancy) {                
        int time = currentTime;
        std::list<int> pathCopy = path;

        for(int i = 0; i < maxWaitTime; i++){
            if(!detectCollision(path, currentPosition, currentTime + i)){
                waitTime = i;
                path = pathCopy;
                return true;
            }
            if (trackOccupancy[currentPosition].find(time) != trackOccupancy[currentPosition].end() ){
                //colision happens while waiting
                return false;
            }else{
                auto it = std::find(pathCopy.begin(), pathCopy.end(), currentPosition);
                pathCopy.insert(it, currentPosition);
                time++;
            }
        }
        return false;
    }
    

/**
 * @brief check if the path is feasible with waiting
 * 
 * @param path (in) path to check
 * @param startTime (in) original start time
 * @param startPosition (in) original start position
 * @param maxWaitTime (in) max waiting time
 * @param waitTime (out) waiting time, ignored if return false
 * @param modifiedPath (out) modified path, ignored if return false
 * @param trackOccupancy (in) track occupancy table
 * @return true feasible
 * @return false not feasible
 */
    bool isPathFeasibleWithWaiting(std::list<int> path, int startTime, int startPosition, int maxWaitTime, 
                            int &waitTime, std::list<int> &modifiedPath,
                            std::unordered_map<int, std::unordered_set<int>> trackOccupancy) {
        int waitPoint = startPosition;
        modifiedPath = path;
        //TODO: improve the algorithm to find the best wait point
        return couldWaitToPass(modifiedPath, startTime, waitPoint, maxWaitTime, waitTime, trackOccupancy);
    }

public:
    //load the track graph from file

    TrainSystem(TrackGraph &graph) : trackGraph(graph) {}

    int addTrain(int id, int source, int destination, int nextPosition) {
        std::list<std::list<int>> paths;
        Train train(id, source, destination, nextPosition);

        // if the source position or destination position is not in the graph, return error
        auto vertices = trackGraph.getVertices();
        if (vertices.find(train.source) == vertices.end() || vertices.find(train.destination) == vertices.end()) {
            std::cout << "Source or destination position not in the graph for train " << train.id << "\n";
            return -1;
        }

        // if the source position is occupied, return error
        if (trackOccupancy[train.source].find(train.startTime) != trackOccupancy[train.source].end()) {
            std::cout << "Source position is occupied for train " << train.id << "\n";
            return -1;
        }

        // if the destination position is same as another train's destination, return error
        for (const Train& existingTrain : trains) {
            if (existingTrain.destination == train.destination) {
                std::cout << "Destination position conflict for train " << train.id << "\n";
                return -1;
            }
        }

        // if the destination position is not a terminator, return error
        if (trackGraph.getVertexType(train.destination) != TERMINATOR) {
            std::cout << "Destination position is not a terminator for train " << train.id << "\n";
            return -1;
        }

        // get all paths from source to destination
        trackGraph.getAllPaths(train.source, train.destination, train.nextPosition, paths);

        // print all paths
        std::cout << paths.size() <<  " possible paths from (" << train.source << " -> " << train.nextPosition << ")" << " to " 
                        << train.destination << " for train " << train.id << ":\n";
        for (auto path : paths) {
            printPath(path);
        }

        // Select the shortest feasible path
        for (const auto& path : paths) {
            train.setPath(path);
            std::list<int> modifiedPath;
            int waitTime = 0;
            if (isPathFeasibleWithWaiting(train.path, train.startTime, train.source, 5, waitTime, modifiedPath, trackOccupancy)){
                train.setPath(modifiedPath);
                trains.push_back(train);
                updateTrackOccupancy(train);
                return 0;
            }
        }

        std::cout << "No feasible path found for train " << train.id << "\n";
        return -1;
    }

    void printTrains() {
        for (Train& train : trains) {
            std::cout << "Train " << train.id << " path: ";
            for (int vertex : train.path) {
                if(vertex == train.path.back())
                    std::cout << std::setw(2) << vertex;
                else
                    std::cout << std::setw(2) << vertex << " -> ";
            }
            std::cout << "\n";
        }
    }

    void printTrackOccupancy(std::unordered_map<int, std::unordered_set<int>> occupancy) {
        for (size_t i = 0; i < occupancy.size(); i++) {
            std::cout << "Vertex " << i << " occupied at times: ";
            for (int time : occupancy[i]) {
                std::cout << time << " ";
            }
            std::cout << "\n";
        }
    }

    void printTrackOccupancy(){
        printTrackOccupancy(trackOccupancy);
    }

    void printPath(std::list<int> path) {
        for (int vertex : path) {
            std::cout << vertex << " ";
        }
        std::cout << "\n";
    }

    void setSignalWithTrainState(int time){
        trackGraph.resetSignal();
        trackGraph.resetVertexHasTrain();
        for(Train& train : trains){
            trackGraph.forbidEnter(train.getPositionAtTime(time));
            trackGraph.exclusiveEnter(train.getPositionAtTime(time), train.getNextPositionAtTime(time));
            trackGraph.setVertexHasTrain(train.getPositionAtTime(time), train.id);
        }
    }

    bool checkIfAllTrainsArrived(int time){
        for(Train& train : trains){
            if(train.getPositionAtTime(time) != train.destination){
                return false;
            }
        }
        return true;
    }
};

#endif // TRAINSYSTEM_H
