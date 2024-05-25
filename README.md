# Min Cost Flow

Min cost flow implementation

## Features

- Bellman-Ford with Dijkstra or Edmonds-Karp algorithm

## Example
[Demo](./examples/main.cpp)
```cpp
#include <format>
#include <iostream>
#include <string_view>

#include <min_cost_flow/min_cost_flow.hpp>

using TFlow = int32_t;
using TCost = int32_t;

/*
(label)           <-- vertex
[capacity, cost]  <-- edge

               (1)
             ↗     ↘
       [2,1]    ↓    [1,1]
     ↗                     ↘
S(0)          [1,1]          T(3)
     ↘                     ↗
       [1,1]    ↓    [2,1]
             ↘     ↗
               (2)
*/

int main() {
  NMCF::TMinCostFlow<TFlow, TCost,
                     /*FindPathType=*/NMCF::FindPathType::kDijkstra>
      minCostFlowDijkstra(/*nVertices=*/4u);
  constexpr uint32_t kSource = 0u, kSink = 3u;
  minCostFlowDijkstra.AddEdge(kSource, 1u, 2u, 1u);
  minCostFlowDijkstra.AddEdge(kSource, 2u, 1u, 1u);
  minCostFlowDijkstra.AddEdge(1u, 2u, 1u, 1u);
  minCostFlowDijkstra.AddEdge(1u, kSink, 1u, 1u);
  minCostFlowDijkstra.AddEdge(2u, kSink, 2u, 1u);

  const auto &&[flow, cost] = minCostFlowDijkstra.Flow(kSource, kSink);
  using namespace std::string_view_literals;
  std::cout << std::format("Maximum flow: {}, minium cost: {}"sv, flow, cost);
}
```

## Requirements

|              | _Supported_ |
|--------------|-------------|
| Standard | C++23       |

### Third-party

- [googletest](https://github.com/google/googletest) - Testing framework
- [googlebenchmark](https://github.com/google/benchmark) – Microbenchmark library

## Build

```shell
# Clone repo
git clone https://github.com/aminjonshermatov/min_cost_flow.git 
cd min_cost_flow
mkdir build && cd build
# Generate build files
cmake -DMCF_TESTS=ON -DMCF_EXAMPLES=ON ..
# Build example
make libmcf_examples
# Run example
./examples/libmcf_examples
```

### CMake options

#### Flags

- `MCF_TESTS` – Enable tests
- `MCF_BENCHMARKS` - Run Benchmarks
- `MCF_EXAMPLES` – Enable examples
- `MCF_DISABLE_VERIFY` – Disable assert
