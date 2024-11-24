//
//  Graph.hpp
//  graph
//
//  Created by Teodor Calin on 24/11/24.
//

#ifndef Graph_hpp
#define Graph_hpp

#include <stdio.h>
#include <vector>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <fstream>
#include <sstream>
#include <limits>
#include <stdexcept>
#include <functional>
#include <Metal/Metal.hpp>

struct Edge {
    int source;
    int destination;
    float weight;
};

class Graph {
private:
    int vertices;
    std::vector<Edge> edges;

    // Metal setup
    MTL::Device* device;
    MTL::CommandQueue* commandQueue;
    MTL::Library* library;
    MTL::ComputePipelineState* dijkstraPipelineState;
    MTL::ComputePipelineState* aStarPipelineState;

    void setupMetal();
    void loadShaders();

public:
    // Constructor
    Graph(int v);

    // Destructor
    ~Graph();

    // Method to add an edge
    void addEdge(int src, int dest, float weight);

    // Method to get the number of vertices
    int getVertices() const;

    // Method to get the edges
    std::vector<Edge>& getEdges();

    // Method to display the graph
    void display() const;

    // Method to initialize distances
    std::vector<float> initializeDistances(int startVertex) const;

    // Method to export the graph to a CSV file
    void toCSV(const std::string& filename) const;

    // Method to import the graph from a CSV file
    void fromCSV(const std::string& filename);

    // Method to clear the graph
    void clear();

    // Method to check if the graph is empty
    bool isEmpty() const;

    // Dijkstra's algorithm
    std::vector<float> dijkstra(int startVertex, bool useMetal = false) const;

    // A* algorithm
    std::vector<int> aStar(int startVertex, int goalVertex, std::function<float(int, int)> heuristic, bool useMetal = false) const;
};

#endif /* Graph_hpp */
