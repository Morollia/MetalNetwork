# Metal-Accelerated Dijkstra's Algorithm

This repository contains an optimized implementation of Dijkstra's algorithm using Metal, Apple's framework for parallel programming on GPUs. The implementation demonstrates significant performance improvements over traditional CPU-based approaches, making it feasible to perform real-time graph analysis on portable devices.

## Table of Contents

- Introduction
- Features
- Installation
- Usage
- Performance Results
- Future Improvements
- Contributing
- License

## Introduction

Graph algorithms are essential tools in computer science, with applications in various fields such as network routing, social network analysis, and geographic information systems. This project leverages Metal's parallel processing capabilities to accelerate Dijkstra's algorithm, achieving substantial reductions in execution time.

## Features

- **Metal-Accelerated Dijkstra's Algorithm**: Significantly faster execution compared to CPU-based implementations.
- **Scalability**: Efficiently handles large graphs, making it suitable for real-time applications on portable devices.
- **Comparative Analysis**: Includes performance comparisons between Metal and CPU implementations.

## Installation

To clone and set up this repository, run the following commands:

```bash
git clone https://github.com/Morollia/MetalNetwork.git
cd metal-dijkstra
```

Ensure you have Xcode and the necessary Metal development tools installed on your macOS system. Follow the instructions for installing metal-cpp from Apple's Website. Also before running shaders have to be compiled into .metallib, from .metal. This can be done via:

```bash
xcrun -sdk macosx metal -c /path/to/Shaders.metal -o /path/to/Shaders.air
xcrun -sdk macosx metallib /path/to/Shaders.air -o /path/to/Shaders.metallib
```

## Usage

To run the Metal-accelerated Dijkstra's algorithm, follow these steps:

1. Open the project in Xcode.
2. Build and run the project.
3. (Optional) Create your own graphs.

Example usage:

```cpp
std::vector<float> distances = graph.dijkstra(startVertex, true); // Use Metal
std::vector<float> distancesCPU = graph.dijkstra(startVertex, false); // Use CPU
```

## Performance Results

The following table summarizes the performance improvements achieved with the Metal implementation:

| Multiplier | Vertices | Edges  | Graph Size (MB) | Dijkstra (Metal) Time (s) | Dijkstra (CPU) Time (s) |
| ---------- | -------- | ------ | --------------- | ------------------------- | ----------------------- |
| 1          | 10000    | 50000  | 0.572           | 0.853                     | 3.589                   |
| 2          | 20000    | 100000 | 1.144           | 1.664                     | 14.269                  |
| ...        | ...      | ...    | ...             | ...                       | ...                     |
| 10         | 100000   | 500000 | 5.722           | 8.066                     | 360.492                 |

## Future Improvements

Future work could focus on:

- Implementing the A\* algorithm using Metal.
- Exploring other graph algorithms such as BFS, DFS, and MST.
- Developing distributed computing approaches for even larger graphs(Maybe even with that Thunderbolt Bridge feature that nobody cared about).
- Further optimizing the Metal kernels for better performance.
-

## Contributing

Contributions are welcome! If you find any issues or have suggestions for improvements, please open an issue or submit a pull request.

## License

This project is licensed under the MIT License. See the LICENSE file for details.

## References

For more detailed information, please refer to the following paper:

Teodor Ioan Calin, VU Amsterdam, https://arxiv.org/submit/6021433/view
