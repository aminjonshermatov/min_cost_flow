#include <benchmark/benchmark.h>
#include <min_cost_flow/min_cost_flow.hpp>
#include <util/network_generator.hpp>

std::vector<int64_t> numberOfVertices{100, 250, 500, 750, 1000, 2000},
    percentageOfDense{30, 40, 50, 60, 70, 80};

std::map<std::pair<int64_t, int64_t>,
         std::vector<NUtil::NNetwork::TEdge<int64_t, int64_t>>>
    cache{};

auto GetNetwork(uint32_t nVertices, uint32_t percentage) -> decltype(auto) {
  const auto key = std::pair(nVertices, percentage);
  if (!cache.contains(key)) {
    cache.emplace(key,
                  NUtil::NNetwork::GenerateWeightedWithCost<int64_t, int64_t>(
                      nVertices, percentage, int64_t{1000}, int64_t{1000}));
  }
  return cache.at(key);
}

static void MinCostFlowDijkstra(benchmark::State &state) {
  for ([[maybe_unused]] auto _ : state) {
    const auto nVertices = state.range(0);
    NMCF::TMinCostFlow<int64_t, int64_t, NMCF::FindPathType::kDijkstra>
        minCostFlow(nVertices);
    const uint32_t source{}, sink = nVertices - 1;
    state.PauseTiming();
    for (const auto &[from, to, capacity, cost] :
         GetNetwork(nVertices, state.range(1))) {
      minCostFlow.AddEdge(from, to, capacity, cost);
    }
    state.ResumeTiming();
    benchmark::DoNotOptimize(minCostFlow.Flow(source, sink));
  }
}

static void MinCostFlowEdmondsKarp(benchmark::State &state) {
  for ([[maybe_unused]] auto _ : state) {
    const auto nVertices = state.range(0);
    NMCF::TMinCostFlow<int64_t, int64_t, NMCF::FindPathType::kEdmondsKarp>
        minCostFlow(nVertices);
    const uint32_t source{}, sink = nVertices - 1;
    state.PauseTiming();
    for (const auto &[from, to, capacity, cost] :
         GetNetwork(nVertices, state.range(1))) {
      minCostFlow.AddEdge(from, to, capacity, cost);
    }
    state.ResumeTiming();
    benchmark::DoNotOptimize(minCostFlow.Flow(source, sink));
  }
}

constexpr uint32_t kIterations = 5u;

BENCHMARK(MinCostFlowDijkstra)
    ->ArgsProduct(std::vector{numberOfVertices, percentageOfDense})
    ->Unit(benchmark::kMillisecond)
    ->Iterations(kIterations);

BENCHMARK(MinCostFlowEdmondsKarp)
    ->ArgsProduct(std::vector{numberOfVertices, percentageOfDense})
    ->Unit(benchmark::kMillisecond)
    ->Iterations(kIterations);

BENCHMARK_MAIN();