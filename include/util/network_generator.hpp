#pragma once

#include <algorithm>
#include <concepts>
#include <functional>
#include <ranges>
#include <cassert>
#include <queue>

#include "random.hpp"
#include "range_to_container.hpp"

namespace NUtil::NNetwork {

  template<std::integral TUnit>
  struct TEdge {
    std::size_t from{}, to{};
    TUnit capacity{}, cost{};
  };

  template<std::integral TUnit>
  auto generateWeightedWithCost(const std::size_t verticesCount,
                                const std::size_t percentageProbability,
                                const std::size_t maxFlowValue,
                                const std::size_t maxCostValue) -> decltype(auto) {
    assert(verticesCount > 2);// source, sink, ...
    assert(std::clamp<std::size_t>(percentageProbability, 0u, 100u) == percentageProbability);
    auto randomDAG = [](const std::size_t n, std::size_t percentageProbability) -> decltype(auto) {
      return std::views::iota(decltype(n)(0), n * (n - 1) / 2) | std::views::transform([&](auto) { return NRandom::range(1, 100) <= percentageProbability; }) | to<std::vector<bool>>();
    };

    auto indexDAG = [](std::size_t n, std::size_t i, std::size_t j) -> decltype(auto) {
      return n * i + j - (i + 1) * (i + 2) / 2;
    };

    auto isConnected = [&indexDAG](const std::size_t n, const std::vector<bool> &dag, const std::size_t source) -> decltype(auto) {
      std::vector<bool> isReached(n, false);
      std::queue<std::size_t> queue;
      isReached[source] = true;
      queue.emplace(source);
      while (!queue.empty()) {
        const auto x = queue.front();
        queue.pop();
        for (auto i = 0u; i < n; ++i) {
          if (isReached[i]) continue;
          const auto j = i < x ? indexDAG(n, i, x) : indexDAG(n, x, i);
          if (dag[j] == 0) continue;
          isReached[i] = true;
          queue.emplace(i);
        }
      }
      return std::ranges::all_of(isReached, std::identity());
    };

    auto complement = [](std::vector<bool> dag) -> decltype(auto) {
      return std::move(dag) | std::ranges::views::transform([](auto x) { return !x; }) | to<decltype(dag)>();
    };

    auto randomConnectedDAG = [&randomDAG, &isConnected, &complement](const std::size_t n, std::size_t percentageProbability) -> decltype(auto) {
      const auto dag = randomDAG(n, percentageProbability);
      return isConnected(n, dag, 0u) ? dag : complement(dag);
    };

    std::vector<TEdge<TUnit>> edges;
    const auto dag = randomConnectedDAG(verticesCount - 2, percentageProbability);
    assert(isConnected(verticesCount - 2, dag, 0));
    std::vector<std::size_t> inDeg(verticesCount - 2, 0), outDeg(verticesCount - 2, 0);
    for (auto i = 0u; i < verticesCount - 2; i++) {
      for (auto j = i + 1; j < verticesCount - 2; j++) {
        const auto k = indexDAG(verticesCount, i, j);
        if (dag[k]) {
          edges.emplace_back(i + 1, j + 1, NRandom::range(1, maxFlowValue), NRandom::range(1, maxCostValue));
          ++inDeg[j];
          ++outDeg[i];
        }
      }
    }
    const std::size_t source = 0u, sink = verticesCount - 1;
    for (auto i = 0u; i < verticesCount - 2; ++i) {
      if (inDeg[i] == 0u) {
        edges.emplace_back(source, i + 1, NRandom::range(1, maxFlowValue), NRandom::range(1, maxCostValue));
      }
      if (outDeg[i] == 0u) {
        edges.emplace_back(i + 1, sink, NRandom::range(1, maxFlowValue), NRandom::range(1, maxCostValue));
      }
    }

    return edges;
  }

} // namespace NUtil::NNetwork
