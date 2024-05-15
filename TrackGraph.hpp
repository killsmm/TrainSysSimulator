#ifndef TRACK_GRAPH_HPP
#define TRACK_GRAPH_HPP

#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <climits>
#include <algorithm>
#include <list>

using namespace std;

typedef enum VertexType{
    TERMINATOR = 0,
    TRACK_SEGMENT,
    JUNCTION
} VertexType;

class Vertex{
private:
    int id;
    VertexType type;
public:
    Vertex(int id, VertexType type){
        this->id = id;
        this->type = type;
    }
    int getID(){
        return id;
    }
    VertexType getType(){
        return type;
    }
};

class Edge{
private:
    int source_id;
    int dest_id;
    int weight;
    bool is_passable;
public:
    Edge(int source, int dest, int weight){
        this->source_id = source;
        this->dest_id = dest;
        this->weight = weight;
        this->is_passable = true;
    }
    int getSourceID(){
        return source_id;
    }
    int getDestID(){
        return dest_id;
    }
    int getWeight(){
        return weight;
    }
    int setPassable(bool passable){
        is_passable = passable;
        return 0;
    }
    bool isPassable(){
        return is_passable;
    }
};

class TrackGraph {
private:
    int numVertices;
    int numEdges;
    vector<Vertex> vertices;
    vector<vector<Edge>> adjacentEdges;

    int addEdge(int source, int dest, int weight){
        if (source >= numVertices || dest >= numVertices){
            return -1;
        }

        /* check if it already contains the same edge*/
        for (int i = 0; i < adjacentEdges[source].size(); i++){
            if (adjacentEdges[source][i].getDestID() == dest){
                return -1;
            }
        }

        /* check if the vertices have too many edges*/
        if(vertices[source].getType() == TERMINATOR && adjacentEdges[source].size() > 0){
            return -1;
        }else if(vertices[source].getType() == TRACK_SEGMENT && adjacentEdges[source].size() > 1){
            return -1;
        }

        if(vertices[dest].getType() == TERMINATOR && adjacentEdges[dest].size() > 0){
            return -1;
        }else if(vertices[dest].getType() == TRACK_SEGMENT && adjacentEdges[dest].size() > 1){
            return -1;
        }


        adjacentEdges[source].push_back(Edge(source, dest, weight));
        adjacentEdges[dest].push_back(Edge(dest, source, weight));
        
        return 0;
    }

public:
    TrackGraph(){
    }
    int readFromFile(string filename){
        ifstream file(filename);
        if (!file.is_open()){
            cerr << "Error: could not open file " << filename << endl;
            return -1;
        }
        string line;
        getline(file, line);
        stringstream ss(line);

        ss >> numVertices;

        for(int i = 0; i < numVertices; i++){
            getline(file, line);
            stringstream ss(line);
            int id;
            string type_string;
            ss >> id >> type_string;
            if (type_string == "TERMINATOR"){
                vertices.push_back(Vertex(id, TERMINATOR));
            }else if (type_string == "TRACK_SEGMENT"){
                vertices.push_back(Vertex(id, TRACK_SEGMENT));
            }else if (type_string == "JUNCTION"){
                vertices.push_back(Vertex(id, JUNCTION));
            }else{
                cerr << "Error: unknown vertex type " << type_string << endl;
                return -1;
            }
            adjacentEdges.push_back(vector<Edge>());
        }

        getline(file, line);
        stringstream ss2(line);
        ss2 >> numEdges;

        for(int i = 0; i < numEdges; i++){
            getline(file, line);
            stringstream ss(line);
            int source, dest, weight;
            ss >> source >> dest >> weight;
            if(addEdge(source, dest, weight) != 0){
                cerr << "Error: failed add edge " << source << " " << dest << " " << weight << endl;
                return -1;
            }
        }
        return 0;
    }

    int findShortestPath(int source, int dest, list<int>& path){
        vector<int> dist(numVertices, INT_MAX);
        vector<int> prev(numVertices, -1);
        vector<bool> visited(numVertices, false);

        dist[source] = 0;
        for(int i = 0; i < numVertices; i++){
            int min_dist = INT_MAX;
            int min_index = -1;
            for(int j = 0; j < numVertices; j++){
                if (!visited[j] && dist[j] < min_dist){
                    min_dist = dist[j];
                    min_index = j;
                }
            }
            if (min_index == -1){
                break;
            }
            visited[min_index] = true;
            for(int j = 0; j < adjacentEdges[min_index].size(); j++){
                int dest = adjacentEdges[min_index][j].getDestID();
                int weight = adjacentEdges[min_index][j].getWeight();
                if (!visited[dest] && dist[min_index] + weight < dist[dest]){
                    dist[dest] = dist[min_index] + weight;
                    prev[dest] = min_index;
                }
            }
        }

        if (dist[dest] == INT_MAX){
            return -1;
        }

        int current = dest;
        while(current != -1){
            path.push_back(current);
            current = prev[current];
        }
        reverse(path.begin(), path.end());
        return dist[dest];
    }

    void printGraph(){
        cout << numVertices << " vertices" << endl;
        for(int i = 0; i < numVertices; i++){
            cout << "Vertex " << vertices[i].getID() << " ";
            if (vertices[i].getType() == TERMINATOR){
                cout << "TERMINATOR" << endl;
            }else if (vertices[i].getType() == TRACK_SEGMENT){
                cout << "TRACK_SEGMENT" << endl;
            }else if (vertices[i].getType() == JUNCTION){
                cout << "JUNCTION" << endl;
            }
            for(int j = 0; j < adjacentEdges[i].size(); j++){
                cout << "->" << adjacentEdges[i][j].getDestID() << "(" << adjacentEdges[i][j].getWeight() << ")" <<  endl;
            }
            cout << "---------------------" << endl;
        }
    }

    int setSignal(int source, int dest, bool passable){
        for(int i = 0; i < adjacentEdges[source].size(); i++){
            if (adjacentEdges[source][i].getDestID() == dest){
                adjacentEdges[source][i].setPassable(passable);
                return 0;
            }
        }
        return -1;
    }

    bool checkPassable(int source, int dest){
        for(int i = 0; i < adjacentEdges[source].size(); i++){
            if (adjacentEdges[source][i].getDestID() == dest){
                return adjacentEdges[source][i].isPassable();
            }
        }
        return false;
    }
};

#endif 