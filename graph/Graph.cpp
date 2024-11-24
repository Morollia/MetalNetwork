//
//  Graph.cpp
//  graph
//
//  Created by Teodor Calin on 24/11/24.
//

#define NS_PRIVATE_IMPLEMENTATION
#define CA_PRIVATE_IMPLEMENTATION
#define MTL_PRIVATE_IMPLEMENTATION
#include <Foundation/Foundation.hpp>
#include <Metal/Metal.hpp>
#include <QuartzCore/QuartzCore.hpp>

#include "Graph.hpp"
#include <simd/simd.h>

// Constructor
Graph::Graph(int v) : vertices(v), device(nullptr), commandQueue(nullptr), library(nullptr), dijkstraPipelineState(nullptr), aStarPipelineState(nullptr) {
    setupMetal();
    loadShaders();
}

// Destructor
Graph::~Graph() {
    if (dijkstraPipelineState) dijkstraPipelineState->release();
    if (aStarPipelineState) aStarPipelineState->release();
    if (library) library->release();
    if (commandQueue) commandQueue->release();
    if (device) device->release();
}

// Method to add an edge
void Graph::addEdge(int src, int dest, float weight) {
    if (src >= vertices || dest >= vertices) {
        throw std::out_of_range("Vertex index out of range");
    }
    edges.push_back({src, dest, weight});
}

// Method to get the number of vertices
int Graph::getVertices() const {
    return vertices;
}

// Method to get the edges
std::vector<Edge>& Graph::getEdges() {
    return edges;
}

// Method to display the graph
void Graph::display() const {
    for (const auto& edge : edges) {
        std::cout << "Edge from " << edge.source << " to " << edge.destination << " with weight " << edge.weight << std::endl;
    }
}

// Method to initialize distances
std::vector<float> Graph::initializeDistances(int startVertex) const {
    std::vector<float> distances(vertices, FLT_MAX);
    distances[startVertex] = 0.0f;
    return distances;
}

// Method to export the graph to a CSV file
void Graph::toCSV(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open file");
    }
    file << "source,destination,weight\n";
    for (const auto& edge : edges) {
        file << edge.source << "," << edge.destination << "," << edge.weight << "\n";
    }
    file.close();
}

// Method to import the graph from a CSV file
void Graph::fromCSV(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open file");
    }
    std::string line;
    std::getline(file, line); // Skip header
    edges.clear();
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string src, dest, weight;
        std::getline(ss, src, ',');
        std::getline(ss, dest, ',');
        std::getline(ss, weight, ',');
        addEdge(std::stoi(src), std::stoi(dest), std::stof(weight));
    }
    file.close();
}

// Method to clear the graph
void Graph::clear() {
    edges.clear();
}

// Method to check if the graph is empty
bool Graph::isEmpty() const {
    return edges.empty();
}

// Metal setup
void Graph::setupMetal() {
    device = MTL::CreateSystemDefaultDevice();
    if (!device) {
        std::cerr << "Metal is not supported on this device" << std::endl;
        return;
    }
    commandQueue = device->newCommandQueue();
}

// Load shaders
void Graph::loadShaders() {
    if (!device) return;

    NS::Error* error = nullptr;
    NS::String* shaderPath = NS::String::string("/Users/teodorcalin/Desktop/Morallia/graph/graph/Shaders.metallib", NS::UTF8StringEncoding);
    library = device->newLibrary(shaderPath, &error);
    if (!library) {
        std::cerr << "Failed to load library: " << error->localizedDescription()->utf8String() << std::endl;
        return;
    }

    MTL::Function* dijkstraFunction = library->newFunction(NS::String::string("dijkstra", NS::UTF8StringEncoding));
    dijkstraPipelineState = device->newComputePipelineState(dijkstraFunction, &error);
    if (!dijkstraPipelineState) {
        std::cerr << "Failed to create Dijkstra pipeline state: " << error->localizedDescription()->utf8String() << std::endl;
        return;
    }

    MTL::Function* aStarFunction = library->newFunction(NS::String::string("aStar", NS::UTF8StringEncoding));
    aStarPipelineState = device->newComputePipelineState(aStarFunction, &error);
    if (!aStarPipelineState) {
        std::cerr << "Failed to create A* pipeline state: " << error->localizedDescription()->utf8String() << std::endl;
        return;
    }
}

// Dijkstra's algorithm
std::vector<float> Graph::dijkstra(int startVertex, bool useMetal) const {
    if (useMetal && device) {
        // Metal-accelerated Dijkstra's algorithm
        std::vector<float> distances = initializeDistances(startVertex);
        MTL::Buffer* edgeBuffer = device->newBuffer(edges.data(), edges.size() * sizeof(Edge), MTL::ResourceStorageModeShared);
        MTL::Buffer* distanceBuffer = device->newBuffer(distances.data(), distances.size() * sizeof(float), MTL::ResourceStorageModeShared);
        int numEdges = static_cast<int>(edges.size());
        MTL::Buffer* numEdgesBuffer = device->newBuffer(&numEdges, sizeof(int), MTL::ResourceStorageModeShared);

        // Define the threadgroup size and grid size
        MTL::Size threadgroupSize = MTL::Size::Make(16, 1, 1); // 16 threads per threadgroup
        MTL::Size gridSize = MTL::Size::Make((numEdges + 15) / 16, 1, 1); // Number of threadgroups needed

        // Create a buffer for grid_size with correct alignment
        simd::uint3 gridSizeValue = simd_make_uint3(gridSize.width, gridSize.height, gridSize.depth);
        MTL::Buffer* gridSizeBuffer = device->newBuffer(&gridSizeValue, sizeof(simd::uint3), MTL::ResourceStorageModeShared);

        for (int i = 0; i < vertices; ++i) {
            MTL::CommandBuffer* commandBuffer = commandQueue->commandBuffer();
            MTL::ComputeCommandEncoder* computeEncoder = commandBuffer->computeCommandEncoder();

            computeEncoder->setComputePipelineState(dijkstraPipelineState);
            computeEncoder->setBuffer(edgeBuffer, 0, 0);
            computeEncoder->setBuffer(distanceBuffer, 0, 1);
            computeEncoder->setBuffer(numEdgesBuffer, 0, 2);
            computeEncoder->setBuffer(gridSizeBuffer, 0, 3);

            computeEncoder->dispatchThreads(gridSize, threadgroupSize);

            computeEncoder->endEncoding();
            commandBuffer->commit();
            commandBuffer->waitUntilCompleted();
        }

        float* resultPointer = static_cast<float*>(distanceBuffer->contents());
        std::vector<float> result(resultPointer, resultPointer + distances.size());

        edgeBuffer->release();
        distanceBuffer->release();
        numEdgesBuffer->release();
        gridSizeBuffer->release();

        return result;
    } else {
        // CPU-based Dijkstra's algorithm
        std::vector<float> distances = initializeDistances(startVertex);
        std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>, std::greater<>> pq;
        pq.push({0.0f, startVertex});

        while (!pq.empty()) {
            auto [dist, u] = pq.top();
            pq.pop();

            for (const auto& edge : edges) {
                if (edge.source == u) {
                    int v = edge.destination;
                    float weight = edge.weight;
                    if (distances[u] + weight < distances[v]) {
                        distances[v] = distances[u] + weight;
                        pq.push({distances[v], v});
                    }
                }
            }
        }

        return distances;
    }
}

// A* algorithm
std::vector<int> Graph::aStar(int startVertex, int goalVertex, std::function<float(int, int)> heuristic, bool useMetal) const {
    if (useMetal && device) {
        // Metal-accelerated A* algorithm
        std::vector<float> gScore(vertices, std::numeric_limits<float>::max());
        std::vector<float> fScore(vertices, std::numeric_limits<float>::max());
        std::vector<int> cameFrom(vertices, -1);
        gScore[startVertex] = 0.0f;
        fScore[startVertex] = heuristic(startVertex, goalVertex);

        MTL::Buffer* edgeBuffer = device->newBuffer(edges.data(), edges.size() * sizeof(Edge), MTL::ResourceStorageModeShared);
        MTL::Buffer* gScoreBuffer = device->newBuffer(gScore.data(), gScore.size() * sizeof(float), MTL::ResourceStorageModeShared);
        MTL::Buffer* fScoreBuffer = device->newBuffer(fScore.data(), fScore.size() * sizeof(float), MTL::ResourceStorageModeShared);
        MTL::Buffer* cameFromBuffer = device->newBuffer(cameFrom.data(), cameFrom.size() * sizeof(int), MTL::ResourceStorageModeShared);
        int numEdges = static_cast<int>(edges.size());
        MTL::Buffer* numEdgesBuffer = device->newBuffer(&numEdges, sizeof(int), MTL::ResourceStorageModeShared);

        for (int i = 0; i < vertices; ++i) {
            MTL::CommandBuffer* commandBuffer = commandQueue->commandBuffer();
            MTL::ComputeCommandEncoder* computeEncoder = commandBuffer->computeCommandEncoder();

            computeEncoder->setComputePipelineState(aStarPipelineState);
            computeEncoder->setBuffer(edgeBuffer, 0, 0);
            computeEncoder->setBuffer(gScoreBuffer, 0, 1);
            computeEncoder->setBuffer(fScoreBuffer, 0, 2);
            computeEncoder->setBuffer(cameFromBuffer, 0, 3);
            computeEncoder->setBuffer(numEdgesBuffer, 0, 4);

            MTL::Size gridSize = MTL::Size(vertices, 1, 1);
            MTL::Size threadGroupSize = MTL::Size(1, 1, 1);
            computeEncoder->dispatchThreads(gridSize, threadGroupSize);

            computeEncoder->endEncoding();
            commandBuffer->commit();
            commandBuffer->waitUntilCompleted();
        }

        int* cameFromPointer = static_cast<int*>(cameFromBuffer->contents());
        std::vector<int> path;
        for (int at = goalVertex; at != -1; at = cameFromPointer[at]) {
            path.push_back(at);
        }
        std::reverse(path.begin(), path.end());

        edgeBuffer->release();
        gScoreBuffer->release();
        fScoreBuffer->release();
        cameFromBuffer->release();
        numEdgesBuffer->release();

        return path;
    } else {
        // CPU-based A* algorithm
        std::vector<float> gScore(vertices, std::numeric_limits<float>::max());
        std::vector<float> fScore(vertices, std::numeric_limits<float>::max());
        std::vector<int> cameFrom(vertices, -1);
        gScore[startVertex] = 0.0f;
        fScore[startVertex] = heuristic(startVertex, goalVertex);

        std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>, std::greater<>> openSet;
        openSet.push({fScore[startVertex], startVertex});

        while (!openSet.empty()) {
            int current = openSet.top().second;
            openSet.pop();

            if (current == goalVertex) {
                std::vector<int> path;
                for (int at = goalVertex; at != -1; at = cameFrom[at]) {
                    path.push_back(at);
                }
                std::reverse(path.begin(), path.end());
                return path;
            }

            for (const auto& edge : edges) {
                if (edge.source == current) {
                    int neighbor = edge.destination;
                    float tentative_gScore = gScore[current] + edge.weight;
                    if (tentative_gScore < gScore[neighbor]) {
                        cameFrom[neighbor] = current;
                        gScore[neighbor] = tentative_gScore;
                        fScore[neighbor] = gScore[neighbor] + heuristic(neighbor, goalVertex);
                        openSet.push({fScore[neighbor], neighbor});
                    }
                }
            }
        }

        return {}; // Return an empty path if there is no path to the goal
    }
}
