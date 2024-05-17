#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP
#include "TrackGraph.hpp"
#include "TrainSystem.hpp"

class Simulator {
private:
    static std::unique_ptr<Simulator> instance;
    TrackGraph graph;
    TrainSystem trainsystem;
    Simulator():trainsystem(graph) {}
    Simulator(const Simulator&) = delete;
    Simulator& operator=(const Simulator&) = delete;

public:
    static Simulator* getInstance() {
        if (instance == nullptr) {
            instance = std::unique_ptr<Simulator>(new Simulator());
        }
        return instance.get();
    }

    /**
     * @brief generate a random graph
     * 
     * @param numVertices number of vertices
     * @param numEdges number of edges (not guaranteed to be all valid edges)
     */
    void generateRandomGraph(int numVertices = 15, int numEdges = 50) {
        
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, numVertices - 1);

        graph.resetGraph();
        // Add vertices
        for (int i = 0; i < numVertices; ++i) {
            graph.addVertex(i, static_cast<VertexType>(dis(gen) % 3));
        }

        // Add edges
        for (int i = 0; i < numEdges; ++i) {
            int source = dis(gen);
            int destination = dis(gen);

            // Ensure no self-loops and duplicate edges
            if (source != destination && !graph.edgeExists(source, destination)) {
                graph.addEdge(source, destination, 1);
            } else {
                --i; // Retry with a new pair if invalid
            }
        }

        //make sure each vertex has at least one edge
        for (int i = 0; i < numVertices; ++i) {
            if (graph.getEdgesFrom(i).size() == 0) {
                
                int destination = 0;
                do {
                    destination = dis(gen);
                }while(destination == i);
                
                graph.addEdge(i, destination, 1);
            }
        }
    }

    int loadGraphFromFile(std::string filename){
        graph.resetGraph();
        return graph.readFromFile(filename);
    }


    /**
     * @brief put random trains on the graph
     * 
     * @param numTrains number of trains(not guaranteed to be all valid trains)
     */
    void putRandomTrains(int numTrains = 15) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, graph.getVertices().size() - 1);

        for (int i = 0; i < numTrains; ++i) {
            int source = dis(gen);
            int destination = dis(gen);
            

            do{
                destination = dis(gen);
            }while(source == destination || graph.getVertexType(destination) != TERMINATOR);

            auto edges = graph.getEdgesFrom(source);
            int direction = edges.size() == 0 ? 0 : edges[0]->getDestID();

            trainsystem.addTrain(i, source, destination, direction);
        }
    }

    int addTrain(int id, int source, int destination, int direction){
        return trainsystem.addTrain(id, source, destination, direction);
    }

    void showInfo(){
        std::cout << "===========Graph Info===========\n";
        graph.printGraph();

        std::cout << "===========Trains Info===========\n";
        trainsystem.printTrains();
        
    }

    void start() {
        int time = 0;
        // trainsystem.printTrackOccupancy();
        while(1){
            trainsystem.setSignalWithTrainState(time);
            graph.displayGraph();
            if(trainsystem.checkIfAllTrainsArrived(time)){
                break;
            }
            time++;
        }
    }
};

std::unique_ptr<Simulator> Simulator::instance = nullptr;
#endif