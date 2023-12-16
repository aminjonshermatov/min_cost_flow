#include <benchmark/benchmark.h>
#include <min_cost_flow/min_cost_flow.hpp>

#include "util/network_generator.hpp"

std::vector<int64_t> numberOfVertices{100, 250, 500, 750, 1000, 2000}, percentageOfDense{30, 40, 50, 60, 70, 80};

std::map<std::pair<int64_t, int64_t>, std::vector<NUtil::NNetwork::TEdge<int64_t>>> cache{};
auto getNetwork(int64_t nVertices, int64_t percentage) -> decltype(auto) {
    const auto key = std::pair(nVertices, percentage);
    if (!cache.contains(key)) {
      cache.emplace(key, NUtil::NNetwork::generateWeightedWithCost<int64_t>(nVertices, percentage, 1000, 1000));
    }
    return cache.at(key);
};

static void minCostFlowDijkstra(benchmark::State& state) {
  for (auto _ : state) {
    const auto n = state.range(0);
    NMinCostFlow::TMinCostFlow<int64_t, NMinCostFlow::FindPathType::kDijkstra> minCostFlow(n);
    const std::size_t source = 0, sink = n - 1;
    state.PauseTiming();
    for (const auto& [from, to, capacity, cost] : getNetwork(n, state.range(1))) {
      minCostFlow.addEdge(from, to, capacity, cost);
    }
    state.ResumeTiming();
    benchmark::DoNotOptimize(minCostFlow.flow(source, sink));
  }
}

static void minCostFlowEdmondsKarp(benchmark::State& state) {
  for (auto _ : state) {
    const auto n = state.range(0);
    NMinCostFlow::TMinCostFlow<int64_t, NMinCostFlow::FindPathType::kEdmondsKarp> minCostFlow(n);
    const std::size_t source = 0, sink = n - 1;
    state.PauseTiming();
    for (const auto& [from, to, capacity, cost] : getNetwork(n, state.range(1))) {
      minCostFlow.addEdge(from, to, capacity, cost);
    }
    state.ResumeTiming();
    benchmark::DoNotOptimize(minCostFlow.flow(source, sink));
  }
}

constexpr std::size_t Iterations = 15u;

BENCHMARK(minCostFlowDijkstra)
    ->ArgsProduct(std::vector{numberOfVertices, percentageOfDense})
    ->Unit(benchmark::kMillisecond)
    ->Iterations(Iterations);

BENCHMARK(minCostFlowEdmondsKarp)
    ->ArgsProduct(std::vector{numberOfVertices, percentageOfDense})
    ->Unit(benchmark::kMillisecond)
    ->Iterations(Iterations);

BENCHMARK_MAIN();