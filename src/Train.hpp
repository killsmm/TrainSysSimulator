#ifndef TRAIN_HPP
#define TRAIN_HPP

#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <list>
#include <algorithm>
#include "TrackGraph.hpp"
#include <iomanip>

class Train {
public:
    int id;
    int source;
    int destination;
    int nextPosition; // To indicate direction
    std::list<int> path;
    int startTime;
    int speed; // assuming uniform speed for simplicity

    // Constructor
    Train(int id, int source, int destination, int nextPosition, int startTime = 0, int speed = 1)
        : id(id), source(source), destination(destination), nextPosition(nextPosition), startTime(startTime), speed(speed) {}

    // Function to set the path
    void setPath(const std::list<int>& newPath) {
        path = newPath;
    }

    // Function to get the path
    std::list<int> getPath() {
        return path;
    }

    // Function to get the position of a time
    int getPositionAtTime(int time) {
        if (time < startTime) {
            return source;
        }
        int currentTime = startTime;
        int currentPosition = source;
        for (int vertex : path) {
            if (currentTime + 1 > time) {
                return currentPosition;
            }
            currentTime += 1;
            currentPosition = vertex;
        }
        return currentPosition;
    }

    // Function to get the next position
    int getNextPositionAtTime(int time) {
        if (time < startTime) {
            return path.front();
        }
        int currentTime = startTime;
        for (int vertex : path) {
            if (currentTime + 1 > time) {
                return vertex;
            }
            currentTime += 1;
        }
        return path.back();
    }
};

#endif