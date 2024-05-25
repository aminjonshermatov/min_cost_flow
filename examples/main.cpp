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