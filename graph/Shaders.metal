//
//  Shaders.metal
//  graph
//
//  Created by Teodor Calin on 24/11/24.
//

#include <metal_stdlib>
using namespace metal;

struct Edge {
    int source;
    int destination;
    float weight;
};

// Custom atomic min function for floats
inline void atomic_min_float(device atomic_float* address, float value) {
    float old = atomic_load_explicit(address, memory_order_relaxed);
    while (value < old && !atomic_compare_exchange_weak_explicit(address, &old, value, memory_order_relaxed, memory_order_relaxed)) {}
}

// Dijkstra's kernel
kernel void dijkstra(device Edge* edges [[buffer(0)]], device atomic_float* distances [[buffer(1)]], constant int& numEdges [[buffer(2)]], constant uint3& grid_size [[buffer(3)]], uint id [[thread_position_in_grid]]) {
    uint numThreads = grid_size.x * grid_size.y * grid_size.z;
    for (int i = id; i < numEdges; i += numThreads) {
        int u = edges[i].source;
        int v = edges[i].destination;
        float weight = edges[i].weight;
        float newDist = atomic_load_explicit(&distances[u], memory_order_relaxed) + weight;

        // Atomic update for floating-point values
        atomic_min_float(&distances[v], newDist);
    }
}

// A* kernel
kernel void aStar(device Edge* edges [[buffer(0)]], device float* gScore [[buffer(1)]], device float* fScore [[buffer(2)]], device int* cameFrom [[buffer(3)]], constant int& numEdges [[buffer(4)]], uint id [[thread_position_in_grid]]) {
    for (int i = 0; i < numEdges; ++i) {
        int u = edges[i].source;
        int v = edges[i].destination;
        float weight = edges[i].weight;
        if (gScore[u] + weight < gScore[v]) {
            gScore[v] = gScore[u] + weight;
            fScore[v] = gScore[v]; // Simplified, normally you would add the heuristic here
            cameFrom[v] = u;
        }
    }
}
