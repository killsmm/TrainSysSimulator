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
#include <unordered_map>
#include <unordered_set>
#include <opencv2/opencv.hpp>
#include <random>

/**
 * @brief generate a random color
 * 
 * @return std::string 
 */
static std::string randomColor() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 255);

    int r = dis(gen);
    int g = dis(gen);
    int b = dis(gen);

    std::stringstream ss;
    ss << "#" << std::hex << std::setfill('0') << std::setw(2) << r
       << std::setfill('0') << std::setw(2) << g
       << std::setfill('0') << std::setw(2) << b;

    return ss.str();
}

typedef enum VertexType{
    TERMINATOR = 0,
    TRACK_SEGMENT,
    JUNCTION,
    UNKNOWN = -1
} VertexType;

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
    std::unordered_map<int, VertexType> vertices;
    std::list<Edge> edges; // all the edges in the graph
    std::vector<std::vector<Edge*>> adjacentEdges; //edges that start from a vertex
    std::vector<std::vector<Edge*>> pointtedEdges; //edges that point to a vertex
    std::unordered_map<int, int> vertexHasTrain; //<vertex number, train id>
    std::unordered_map<int, std::string> trainColor;



    /**
     * @brief dfs to find all paths from source to dest
     * 
     * @param source start vertex
     * @param dest destination vertex
     * @param visited visited vertices
     * @param path a path from source to dest
     * @param paths the list of all paths
     */
    void findAllPathsUtil(int source, int dest, std::vector<bool>& visited, std::list<int>& path, std::list<std::list<int>>& paths){
        if (source == dest){
            paths.push_back(path);
            return;
        }
        visited[source] = true;
        for(size_t i = 0; i < adjacentEdges[source].size(); i++){
            int next = adjacentEdges[source][i]->getDestID();
            if (!visited[next]){
                path.push_back(next);
                findAllPathsUtil(next, dest, visited, path, paths); 
                path.pop_back();
            }
        }
        visited[source] = false;
    }



    std::string getVertexTypeString(int type){
        switch(type){
            case TERMINATOR:
                return "TERMINATOR";
            case TRACK_SEGMENT:
                return "TRACK_SEGMENT";
            case JUNCTION:
                return "JUNCTION";
            default:
                return "UNKNOWN";
        }
    }

    VertexType getVertexTypeFromString(std::string type){
        if (type == "TERMINATOR"){
            return TERMINATOR;
        }else if (type == "TRACK_SEGMENT"){
            return TRACK_SEGMENT;
        }else if (type == "JUNCTION"){
            return JUNCTION;
        }else{
            return UNKNOWN;
        }
    }



public:

    TrackGraph(){
        numVertices = 0;
        numEdges = 0;
    }

    int resetGraph(){
        vertices.clear();
        adjacentEdges.clear();
        pointtedEdges.clear();
        numEdges = 0;
        numVertices = 0;
        edges.clear();
        return 0;
    }

    std::vector<Edge*> getEdgesFrom(int source){
        return adjacentEdges[source];
    }

    std::vector<Edge*> getEdgesTo(int dest){
        return pointtedEdges[dest];
    }

    std::unordered_map<int, VertexType> &getVertices(){
        return vertices;
    }

    bool edgeExists(int source, int dest){
        for(size_t i = 0; i < adjacentEdges[source].size(); i++){
            if (adjacentEdges[source][i]->getDestID() == dest){
                return true;
            }
        }
        return false;
    }

    int addVertex(int id, VertexType type){
        if (vertices.find(id) != vertices.end()){
            return -1;
        }
        vertices[id] = type;
        adjacentEdges.push_back(std::vector<Edge*>());
        pointtedEdges.push_back(std::vector<Edge*>());
        numVertices++;
        return 0;
    }

    int addEdge(int source, int dest, int weight){
        if (vertices.find(source) == vertices.end() || vertices.find(dest) == vertices.end()){
            return -1;
        }

        /* check if it already contains the same edge*/
        for (size_t i = 0; i < adjacentEdges[source].size(); i++){
            if (adjacentEdges[source][i]->getDestID() == dest){
                return -1;
            }
        }

        if((vertices[source] == TERMINATOR && adjacentEdges[source].size() >= 1) || (vertices[source] == TRACK_SEGMENT && adjacentEdges[source].size() >= 2)){
            //terminator already has an edge or track segment already has 2 edges
            return -1;
        }

        if((vertices[dest] == TERMINATOR && adjacentEdges[dest].size() >= 1) || (vertices[dest] == TRACK_SEGMENT && adjacentEdges[dest].size() >= 2)){
            //terminator already has an edge or track segment already has 2 edges
            return -1;
        }

        edges.push_back(Edge(source, dest, weight));
        adjacentEdges[source].push_back(&edges.back());
        pointtedEdges[dest].push_back(&edges.back());

        edges.push_back(Edge(dest, source, weight));
        adjacentEdges[dest].push_back(&edges.back());
        pointtedEdges[source].push_back(&edges.back());

        numEdges += 2;

        return 0;
    }

    int getPathLength(std::list<int>& path){
        int length = 0;
        auto it = path.begin();
        int current = *it;
        it++;
        while(it != path.end()){
            for(size_t i = 0; i < adjacentEdges[current].size(); i++){
                if (adjacentEdges[current][i]->getDestID() == *it){
                    length += adjacentEdges[current][i]->getWeight();
                    break;
                }
            }
            current = *it;
            it++;
        }
        return length;
    }



    int readFromFile(std::string filename){
        //clean up the graph
        resetGraph();

        int vertices_num = 0; //read from file
        int edge_num = 0; //read from file

        int ret = 0;

        std::ifstream file(filename);
        if (!file.is_open()){
            std::cerr << "Error: could not open file " << filename << std::endl;
            return -1;
        }
        std::string line;
        std::getline(file, line);
        std::stringstream ss(line);

        ss >> vertices_num;

        for(int i = 0; i < vertices_num; i++){
            std::getline(file, line);
            std::stringstream ss(line);
            int id;
            std::string type_string;
            ss >> id >> type_string;

            VertexType type = getVertexTypeFromString(type_string);

            if(type == -1){
                std::cerr << "Error: unknown vertex type " << type_string << std::endl;
                ret = -1;
            }

            if(addVertex(id, type) != 0){
                std::cerr << "Error: failed to add vertex " << id << " " << type_string << std::endl;
                ret = -1;
            }
        }

        std::getline(file, line);
        std::stringstream ss2(line);
        ss2 >> edge_num;

        for(int i = 0; i < edge_num; i++){
            std::getline(file, line);
            std::stringstream ss(line);
            int source, dest, weight;
            ss >> source >> dest >> weight;
            if(addEdge(source, dest, weight) != 0){
                std::cerr << "Error: failed add edge " << source << " " << dest << " " << weight << std::endl;
                ret = -1;
            }
        }

        return ret;
    }

    int getVertexType(int id){
        if (vertices.find(id) == vertices.end()){
            return -1;
        }
        return vertices[id];
    }

    int findPreviousWaitPoint(std::list<int>& path, int current){
        int cur, next;
        auto it = std::find(path.begin(), path.end(), current);
        if (it == path.end()){
            return -1;
        }

        while(it != path.begin()){
            cur = *it;
            it--;
            next = *it;
            if (vertices[next] == JUNCTION){
                return cur;
            }    
        }
        return -1;
    }

    void generateDotFile(std::string filename) {
        std::ofstream dotFile(filename);

        dotFile << "digraph G {\n";
        dotFile << "node [style=filled];";
        for (auto it = vertices.begin(); it != vertices.end(); it++) {
            dotFile << "  " << it->first << " [label=\"" << it->first << "\"" << " fillcolor=\"" 
                    << (vertexHasTrain.find(it->first) != vertexHasTrain.end() ? trainColor[getTrainIDOnVertex(it->first)] : "white") << "\"];\n";
        }

        for (int i = 0; i < numVertices; i++) {
            for (size_t j = 0; j < adjacentEdges[i].size(); j++) {
                dotFile << "  " << i << " -> " << adjacentEdges[i][j]->getDestID() << " [label=\"" << adjacentEdges[i][j]->getWeight() << "\"" << " color=" << (adjacentEdges[i][j]->isPassable() ? "green" : "red") << "];\n";
            }
        }

        dotFile << "}\n";
        dotFile.close();
    }

    /**
     * @brief Get all paths and sort them by length
     * 
     * @param source (in)
     * @param dest (in)
     * @param paths (output) all paths from source to dest
     * @return 
     */
    void getAllPaths(int source, int dest, int nextPosition, std::list<std::list<int>>& paths){
        std::vector<bool> visited(numVertices, false);
        std::list<int> path;
        path.push_back(source);
        findAllPathsUtil(source, dest, visited, path, paths);

        /*delete path whose next position is not the nextPosition*/

        for(auto it = paths.begin(); it != paths.end(); it++){
            path = *it;
            if(*(std::next(path.begin())) != nextPosition){
                it = paths.erase(it);
            }
        }
        
        paths.sort([this](std::list<int>& path1, std::list<int>& path2){
            return getPathLength(path1) < getPathLength(path2);
        });
    }

    int findShortestPath(int source, int dest, std::list<int>& path){
        std::vector<int> dist(numVertices, INT_MAX);
        std::vector<int> prev(numVertices, -1);
        std::vector<bool> visited(numVertices, false);

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
            for(size_t j = 0; j < adjacentEdges[min_index].size(); j++){
                int dest = adjacentEdges[min_index][j]->getDestID();
                int weight = adjacentEdges[min_index][j]->getWeight();
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
        std::reverse(path.begin(), path.end());
        return dist[dest];
    }

    void printGraph(){
        std::cout << numVertices << " vertices in total:" << std::endl;
        for(auto it = vertices.begin(); it != vertices.end(); it++){
            std::cout << it->first << " " << getVertexTypeString(it->second) << " To:(";
            for(size_t i = 0; i < adjacentEdges[it->first].size(); i++){
                std::cout << " " << adjacentEdges[it->first][i]->getDestID();
            }
            std::cout << " )" << std::endl;
        }
    }
    
    void displayGraph(){
        generateDotFile("graph.dot");

        //convert the dot file to a png image
        std::string command = "dot -Tpng graph.dot -o graph.png";
        system(command.c_str());

        //display the image
        cv::Mat image = cv::imread("graph.png");
        cv::imshow("Graph", image);
        cv::waitKey(0);
    }

    void forbidEnter(int vertex){
        for(auto edge : pointtedEdges[vertex]){
            edge->setPassable(false);
        }
    }

    void exclusiveEnter(int source, int dest){
        for(auto edge : pointtedEdges[dest]){
            if (edge->getSourceID() != source){
                edge->setPassable(false);
            }else{
                edge->setPassable(true);
            }
        }
    }

    void resetSignal(){
        for(auto& edge : edges){
            edge.setPassable(true);
        }
    }

    void setVertexHasTrain(int vertex, int train_id){
        vertexHasTrain[vertex] = train_id;
        if(trainColor.find(train_id) == trainColor.end()){
            trainColor[train_id] = randomColor();
        }
    }

    void resetVertexHasTrain(){
        vertexHasTrain.clear();
    }

    int getTrainIDOnVertex(int vertex){
        if (vertexHasTrain.find(vertex) == vertexHasTrain.end()){
            return -1;
        }
        return vertexHasTrain[vertex];
    }

    bool checkPassable(int source, int dest){
        for(size_t i = 0; i < adjacentEdges[source].size(); i++){
            if (adjacentEdges[source][i]->getDestID() == dest){
                return adjacentEdges[source][i]->isPassable();
            }
        }
        return false;
    }
};

#endif // TRACK_GRAPH_HPP
