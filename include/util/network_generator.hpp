#pragma once

#include <algorithm>
#include <cassert>
#include <concepts>
#include <functional>
#include <queue>
#include <ranges>

#include "min_cost_flow/verify.hpp"

#include "random.hpp"
#include "range_to_container.hpp"

namespace NUtil::NNetwork {

template <std::integral TFlow, std::integral TCost> struct TEdge {
  uint32_t from, to;
  TFlow capacity;
  TCost cost;
};

template <std::integral TFlow, std::integral TCost>
auto GenerateWeightedWithCost(const uint32_t nVertices,
                              const uint32_t percentageProbability,
                              const TFlow maxFlowValue,
                              const TCost maxCostValue) -> decltype(auto) {
  using namespace std::string_view_literals;
  MCF_VERIFY_MSG(nVertices > 2u,
                 "Except source and sink there must be at least one vertex"sv);
  MCF_VERIFY_MSG(std::clamp<uint32_t>(percentageProbability, 0u, 100u) ==
                     percentageProbability,
                 std::format("percentage must be between [0,100], but got {}",
                             percentageProbability));

  auto randomDAG = [](uint32_t nVertices,
                      uint32_t percentageProbability) -> decltype(auto) {
    return std::views::iota(decltype(nVertices){},
                            nVertices * (nVertices - 1) / 2) |
           std::views::transform([&percentageProbability](auto) {
             return NRandom::Range(uint32_t{1}, uint32_t{100}) <=
                    percentageProbability;
           }) |
           to<std::vector<bool>>();
  };

  auto indexDAG = [](uint32_t nVertices, uint32_t i,
                     uint32_t j) -> decltype(auto) {
    return nVertices * i + j - (i + 1) * (i + 2) / 2;
  };

  auto isConnected = [&indexDAG](uint32_t nVertices,
                                 const std::vector<bool> &dag,
                                 uint32_t source) -> decltype(auto) {
    std::vector<bool> isReached(nVertices, false);
    std::queue<uint32_t> queue;
    isReached[source] = true;
    queue.emplace(source);

    while (!queue.empty()) {
      auto &&vertex = queue.front();
      queue.pop();
      for (std::weakly_incrementable auto &&i :
           std::views::iota(0u, nVertices)) {
        if (isReached[i]) {
          continue;
        }
        const auto j = i < vertex ? indexDAG(nVertices, i, vertex)
                                  : indexDAG(nVertices, vertex, i);
        if (!dag[j]) {
          continue;
        }
        isReached[i] = true;
        queue.emplace(i);
      }
    }
    return std::ranges::all_of(isReached, std::identity());
  };

  auto complement = [](std::vector<bool> dag) -> decltype(auto) {
    return std::move(dag) |
           std::ranges::views::transform([](auto &&x) { return !x; }) |
           to<decltype(dag)>();
  };

  auto randomConnectedDAG =
      [&randomDAG, &isConnected,
       &complement](const std::size_t n,
                    std::size_t percentageProbability) -> decltype(auto) {
    const auto dag = randomDAG(n, percentageProbability);
    return isConnected(n, dag, 0u) ? dag : complement(dag);
  };

  std::vector<TEdge<TFlow, TCost>> edges;
  const auto dag = randomConnectedDAG(nVertices - 2, percentageProbability);

  MCF_VERIFY(isConnected(nVertices - 2, dag, 0u));

  std::vector<uint32_t> inDeg(nVertices - 2, 0u), outDeg(nVertices - 2, 0u);
  for (auto &&[i, j] :
       std::views::cartesian_product(std::views::iota(0u, nVertices - 2),
                                     std::views::iota(0u, nVertices - 2)) |
           std::views::filter(
               [](auto &&ij) { return std::get<0>(ij) < std::get<1>(ij); })) {
    const auto k = indexDAG(nVertices, i, j);
    if (dag[k]) {
      edges.emplace_back(i + 1, j + 1, NRandom::Range(TFlow{1}, maxFlowValue),
                         NRandom::Range(TCost{1}, maxCostValue));
      ++inDeg[j];
      ++outDeg[i];
    }
  }
  const uint32_t source{}, sink = nVertices - 1;
  for (std::weakly_incrementable auto &&i :
       std::views::iota(0u, nVertices - 2)) {
    if (inDeg[i] == 0u) {
      edges.emplace_back(source, i + 1, NRandom::Range(TFlow{1}, maxFlowValue),
                         NRandom::Range(TCost{1}, maxCostValue));
    }
    if (outDeg[i] == 0u) {
      edges.emplace_back(i + 1, sink, NRandom::Range(TFlow{1}, maxFlowValue),
                         NRandom::Range(TCost{1}, maxCostValue));
    }
  }

  return edges;
}

} // namespace NUtil::NNetwork
