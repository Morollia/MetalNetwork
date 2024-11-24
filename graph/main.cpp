//
//  main.cpp
//  graph
//
//  Created by Teodor Calin on 24/11/24.
//

#include <vector>
#include <iostream>
#include <string>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include "Graph.hpp"

bool compareResults(const std::vector<float>& a, const std::vector<float>& b, float epsilon = 1e-5) {
    if (a.size() != b.size()) {
        return false;
    }
    for (size_t i = 0; i < a.size(); ++i) {
        if (std::abs(a[i] - b[i]) > epsilon) {
            return false;
        }
    }
    return true;
}

float heuristic(int a, int b) {
    return std::abs(a - b);
}

void saveResultsToCSV(const std::string& filename, const std::vector<std::vector<std::string>>& data) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Unable to open file: " << filename << std::endl;
        return;
    }

    for (const auto& row : data) {
        for (size_t i = 0; i < row.size(); ++i) {
            file << row[i];
            if (i < row.size() - 1) {
                file << ",";
            }
        }
        file << "\n";
    }

    file.close();
}

int main() {
    const int baseNumVertices = 10000;
    const int baseNumEdges = 50000;
    const int maxMultiplier = 10;
    const std::string outputPath = "/Users/teodorcalin/Desktop/Morallia/graph/graph/results.csv";

    std::vector<std::vector<std::string>> results;
    results.push_back({"Multiplier", "Vertices", "Edges", "Graph Size (Bytes)", "Graph Size (KB)", "Graph Size (MB)", "Graph Size (GB)", "Dijkstra (Metal) Time (s)", "Dijkstra (CPU) Time (s)"});

    for (int multiplier = 1; multiplier <= maxMultiplier; multiplier+=1){
        int numVertices = baseNumVertices * multiplier;
        int numEdges = baseNumEdges * multiplier;

        // Initialize the graph
        Graph graph(numVertices);

        // Seed for random number generation
        std::srand(static_cast<unsigned>(std::time(0)));

        // Add random edges to the graph
        for (int i = 0; i < numEdges; ++i) {
            int src = std::rand() % numVertices;
            int dest = std::rand() % numVertices;
            float weight = static_cast<float>(std::rand() % 100 + 1); // Random weight between 1 and 100
            graph.addEdge(src, dest, weight);
        }
        
        std::cout<<"Completed Graph"<<std::endl;

        // Calculate graph size
        size_t edgeSize = graph.getEdges().size() * sizeof(Edge);
        size_t totalSize = edgeSize;

        std::cout << "Multiplier: " << multiplier << std::endl;
        std::cout << "Graph size:" << std::endl;
        std::cout << "Megabytes: " << totalSize / (1024.0 * 1024.0) << " MB" << std::endl;

        // Benchmark Dijkstra's algorithm with Metal
        clock_t start = clock();
        std::vector<float> distancesMetal = graph.dijkstra(0, true);
        clock_t end = clock();
        double dijkstraTimeMetal = static_cast<double>(end - start) / CLOCKS_PER_SEC;
        std::cout << "Dijkstra's algorithm (Metal) took " << dijkstraTimeMetal << " seconds." << std::endl;

        start = clock();
        std::vector<float> distancesCPU = graph.dijkstra(0, false);
        end = clock();
        double dijkstraTimeCPU = static_cast<double>(end - start) / CLOCKS_PER_SEC;
        std::cout << "Dijkstra's algorithm (CPU) took " << dijkstraTimeCPU << " seconds." << std::endl;

        // Compare results
        if (compareResults(distancesMetal, distancesCPU)) {
            std::cout << "Both implementations produced the same results." << std::endl;
        } else {
            std::cout << "The results differ between the implementations." << std::endl;
        }
        
        results.push_back({
            std::to_string(multiplier),
            std::to_string(numVertices),
            std::to_string(numEdges),
            std::to_string(totalSize),
            std::to_string(totalSize / 1024.0),
            std::to_string(totalSize / (1024.0 * 1024.0)),
            std::to_string(totalSize / (1024.0 * 1024.0 * 1024.0)),
            std::to_string(dijkstraTimeMetal),
            std::to_string(dijkstraTimeCPU),
        });
        
        std::cout << std::endl;
    }

    // Save results to CSV
    saveResultsToCSV(outputPath, results);

    std::cout << "Done" << std::endl;
    return 0;
}
