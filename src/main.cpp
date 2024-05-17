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

#include "TrackGraph.hpp"
#include "Train.hpp"
#include "TrainSystem.hpp"
#include "Simulator.hpp"

int main(int argc, char* argv[]) {
    std::string filename = "";
    if(argc > 1){
        filename = argv[1];
    }
    Simulator* simulator = Simulator::getInstance();

    if(filename == ""){
        simulator->generateRandomGraph(30, 100);
        simulator->putRandomTrains(500);
    }else{
        if(simulator->loadGraphFromFile(filename)){
            std::cout << "Failed to read graph from file" << std::endl;
            return -1;
        };
        simulator->addTrain(0, 8, 7, 2);
        simulator->addTrain(1, 5, 3, 2);
        simulator->addTrain(2, 9, 6, 5);
        simulator->addTrain(3, 0, 12, 4);
        simulator->addTrain(4, 10, 13, 11);

    }

    simulator->showInfo();
    simulator->start();
    return 0;
}