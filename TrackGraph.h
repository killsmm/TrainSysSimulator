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

#ifndef TRACK_GRAPH_H
#define TRACK_GRAPH_H

#include <vector>
#include <list>
#include <fstream>
#include <iostream>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

using namespace boost;

enum VertexType {
    TERMINATOR = 0,
    TRACK_SEGMENT,
    JUNCTION
};

struct VertexProperties {
    std::string name;
    VertexType type;
};

struct EdgeProperties {
    int weight;
};

typedef adjacency_list<vecS, vecS, directedS, VertexProperties, EdgeProperties> Graph;
typedef graph_traits<Graph>::vertex_descriptor Vertex;
typedef graph_traits<Graph>::edge_descriptor Edge;


class TrackGraph {

public:
    int readGraphFromFile(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error: could not open file " << filename << std::endl;
            return -1;
        }

        int num_vertices;
        file >> num_vertices;
        for (int i = 0; i < num_vertices; i++) {
            std::string name;
            std::string type_string;
            int type;
            file >> name >> type_string;
            if (type_string == "TERMINATOR") {
                type = TERMINATOR;
            } else if (type_string == "TRACK_SEGMENT") {
                type = TRACK_SEGMENT;
            } else if (type_string == "JUNCTION") {
                type = JUNCTION;
            } else {
                std::cerr << "Error: unknown vertex type " << type_string << std::endl;
                return -1;
            }
            addVertex(i, name, static_cast<VertexType>(type));
        }

        int num_edges;
        file >> num_edges;

        for (int i = 0; i < num_edges; i++) {
            int source, target, weight;
            file >> source >> target >> weight;
            addEdge(source, target, weight);
        }
        return 0;
    }

    int addTerminator(const std::string& name) {
        addVertex(num_vertices(graph), name, TERMINATOR);
        return 0;
    }

    int addTrackSegment(const std::string& name) {
        addVertex(num_vertices(graph), name, TRACK_SEGMENT);
        return 0;
    }

    int addJunction(const std::string& name) {
        addVertex(num_vertices(graph), name, JUNCTION);
        return 0;
    }

    bool checkGraphValidity() {
        //check if track segments only have two edges
        for (auto vd : make_iterator_range(vertices(graph))) {
            if (graph[vd].type == TRACK_SEGMENT) {
                int num_edges = out_degree(vd, graph);
                if (num_edges > 2) {
                    std::cerr << "Error: track segment " << graph[vd].name << " has " << num_edges << " edges" << std::endl;
                    return false;
                }
            }
        }
        //check if terminators have only one edge
        for (auto vd : make_iterator_range(vertices(graph))) {
            if (graph[vd].type == TERMINATOR) {
                int num_edges = out_degree(vd, graph);
                if (num_edges > 1) {
                    std::cerr << "Error: terminator " << graph[vd].name << " has " << num_edges << " edges" << std::endl;
                    return false;
                }
            }
        }
        return true;
    }

    void printGraph() const {


        std::cout << "Vertices:" << std::endl;
        for (auto vd : make_iterator_range(vertices(graph))) {
            std::cout << "Vertex " << vd << ": " << graph[vd].name << std::endl;
        }

        std::cout << "Edges:" << std::endl;
        for (auto ed : make_iterator_range(edges(graph))) {
            Vertex source = ed.m_source;
            Vertex target = ed.m_target;
            std::cout << "Edge (" << source << ", " << target << ") with weight " << graph[ed].weight << std::endl;
        }
    }

private:
    Graph graph;
    void addVertex(int index, const std::string& name, VertexType type) {
        VertexProperties vp;
        vp.name = name;
        vp.type = type;
        add_vertex(vp, graph);
    }

    void addEdge(int source, int target, int weight) {
        EdgeProperties ep;
        ep.weight = weight;
        add_edge(source, target, ep, graph);
    }

    std::vector<int> getShortestPath(std::string source_name, std::string target_name) {
        std::vector<Vertex> p(num_vertices(graph));
        std::vector<int> d(num_vertices(graph));

        //find the vertex index with its name
        Vertex source = -1;
        Vertex target = -1;
        for (auto vd : make_iterator_range(vertices(graph))) {
            if (graph[vd].name == source_name) {
                source = vd;
            }
            if (graph[vd].name == target_name) {
                target = vd;
            }
        }

        if (source == -1 || target == -1) {
            std::cerr << "Error: could not find source or target vertex" << std::endl;
            return std::vector<int>();
        }

        // dijkstra_shortest_paths(graph, source, predecessor_map(&p[0]).distance_map(&d[0]));
        std::list<int> path;
        for (Vertex v = target; v != source; v = p[v]) {
            path.push_front(v);
        }
        path.push_front(source);
        return std::vector<int>(path.begin(), path.end());
    }


};


#endif

/*

#define TERMINATOR_SEGMENT_LENGTH 0
#define TRACK_SEGMENT_LENGTH 1

typedef enum {
    TERMINATOR_SEGMENT = 0,
    SEGMENT_SEGMENT
} LinkType;


class NodeParent{
public:
    int id;

};

class Node{
public:
    bool has_light;
    bool light_state;
    NodeParent* parent;
};

typedef std::vector<Node*> Junction;


class Terminator: public NodeParent{
public:
    Terminator(int id);
    Node node;
};

class TrackSegment: public NodeParent{
public:
    TrackSegment(int id);
    Node nodeA;
    Node nodeB;
};

class SignalLight{
public:
    SignalLight();
    void setState(bool state);
private:
    bool state;
};

class Link{
public:
    Link(Node* nodeA, Node* nodeB, LinkType type);
    Node* nodeA;
    Node* nodeB;
    LinkType type;
};

class TrackGraph{
public:
    Terminator* addTerminator(int id);
    TrackSegment* addTrackSegment(int id);
    int addLink(Node* nodeA, Node* nodeB, LinkType type);
    Terminator* getTerminator(int id);
    TrackSegment* getTrackSegment(int id);
    int getShortestPath(Node* start, Node* end, std::list<Node*> *path);
    void printGraph();

// private:
    std::vector<Terminator*> terminators;
    std::vector<TrackSegment*> trackSegments;
    std::vector<Link*> links;
    std::vector<Junction*> junctions;
};

*/

