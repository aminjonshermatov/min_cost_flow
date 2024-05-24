#pragma once

#include <cassert>
#include <concepts>
#include <cstdint>
#include <queue>
#include <ranges>

#include "verify.hpp"

namespace NMCF {

enum class FindPathType : uint8_t { kDijkstra, kEdmondsKarp };

template <std::integral TFlow, std::integral TCost>
struct TFlowEdge {
  uint32_t from, to;
  TFlow capacity, flow;
  TCost cost;
  uint32_t id;

  TFlowEdge(uint32_t from, uint32_t to, TFlow capacity, TCost cost, uint32_t id)
      : from(from), to(to), capacity(capacity), flow{}, cost(cost), id(id) {}
};

template <std::integral TFlow, std::integral TCost, FindPathType kFindPathType>
class TMinCostFlow {
  uint32_t nVertices_;
  std::vector<std::vector<uint32_t>> adjacentList_;
  std::vector<TFlowEdge<TFlow, TCost>> edges_;
  std::vector<TCost> potential_, distance_;
  std::vector<int32_t> predecessor_;

  public:
  static constexpr uint32_t kNpos = static_cast<uint32_t>(-1);
  static constexpr TFlow kInfinityFlow = std::numeric_limits<TFlow>::max();
  static constexpr TCost kInfinityCost = std::numeric_limits<TCost>::max();

  explicit TMinCostFlow(uint32_t nVertices) : nVertices_(nVertices) {
    adjacentList_.resize(nVertices);
    potential_.resize(nVertices);
    distance_.resize(nVertices);
    predecessor_.resize(nVertices);
  }

  auto AddEdge(uint32_t from, uint32_t to, TFlow capacity, TCost cost,
               int32_t id = kNpos) -> void {
    adjacentList_[from].emplace_back(edges_.size());
    edges_.emplace_back(from, to, capacity, cost, id);
    adjacentList_[to].emplace_back(edges_.size());
    edges_.emplace_back(to, from, TCost{}, -cost, kNpos);
  }

  auto Flow(uint32_t source, uint32_t sink) -> decltype(auto) {
    std::ranges::fill(potential_, TCost{});
    if constexpr (kFindPathType == FindPathType::kDijkstra) {
      // Bellman-Ford
      bool isUpdated = true;
      for ([[maybe_unused]] std::weakly_incrementable auto iteration :
           std::views::iota(uint32_t{}, nVertices_)) {
        isUpdated = false;
        for (const auto &edge : edges_) {
          if (edge.capacity == TFlow{}) {
            continue;
          }
          if (const auto newPotential = potential_[edge.from] + edge.cost;
              potential_[edge.to] > newPotential) {
            isUpdated |= true;
            potential_[edge.to] = newPotential;
          }
        }
      }
      MCF_VERIFY_MSG(!isUpdated, "Detected cycle with negative weight");
    }

    TFlow flow{};
    TCost cost{};
    while ((this->*GetFindPathAlgorithm())(source, sink)) {
      for (std::weakly_incrementable auto &&id :
           std::views::iota(uint32_t{}, nVertices_)) {
        potential_[id] += distance_[id];
      }

      std::integral auto newFlow = kInfinityFlow;
      for (auto vertex = sink; vertex != source;
           vertex = edges_[predecessor_[vertex]].from) {
        newFlow = std::min(newFlow, edges_[predecessor_[vertex]].capacity -
                                        edges_[predecessor_[vertex]].flow);
      }
      flow += newFlow;
      cost += (potential_[sink] - potential_[source]) * newFlow;
      for (auto v = sink; v != source; v = edges_[predecessor_[v]].from) {
        edges_[predecessor_[v]].flow += newFlow;
        edges_[predecessor_[v] ^ 1].flow -= newFlow;
      }
    }
    return std::tuple(flow, cost);
  }

  private:
  auto Dijkstra(const uint32_t source, const uint32_t sink) -> bool {
    distance_.assign(nVertices_, kInfinityCost);
    predecessor_.assign(nVertices_, kNpos);

    std::priority_queue<std::pair<TCost, uint32_t>,
                        std::vector<std::pair<TCost, uint32_t>>, std::greater<>>
        pq;
    pq.emplace(distance_[source] = TCost{}, source);
    while (!pq.empty()) {
      auto [distance, fromVertex] = pq.top();
      pq.pop();

      if (distance > distance_[fromVertex]) {
        continue;
      }
      for (const auto &edgeId : adjacentList_[fromVertex]) {
        const auto toVertex = edges_[edgeId].to;
        if (edges_[edgeId].capacity - edges_[edgeId].flow <= TFlow{}) {
          continue;
        }
        auto newDistance = distance + edges_[edgeId].cost +
                           potential_[fromVertex] - potential_[toVertex];
        if (newDistance < distance_[toVertex]) {
          distance_[toVertex] = newDistance;
          predecessor_[toVertex] = edgeId;
          pq.emplace(distance_[toVertex] = newDistance, toVertex);
        }
      }
    }
    return distance_[sink] != kInfinityCost;
  }

  auto EdmondsKarp(const uint32_t source, const uint32_t sink) -> bool {
    distance_.assign(nVertices_, kInfinityCost);
    predecessor_.assign(nVertices_, kNpos);

    std::queue<uint32_t> q;
    std::vector<bool> inQueue(nVertices_, false);

    q.emplace(source);
    inQueue[source] = true;
    distance_[source] = TCost{};

    while (!q.empty()) {
      auto fromVertex = q.front();
      q.pop();
      inQueue[fromVertex] = false;

      for (auto edgeId : adjacentList_[fromVertex]) {
        auto toVertex = edges_[edgeId].to;
        if (edges_[edgeId].capacity - edges_[edgeId].flow <= TFlow{}) {
          continue;
        }
        auto newDistance = distance_[fromVertex] + edges_[edgeId].cost +
                           potential_[fromVertex] - potential_[toVertex];
        if (newDistance < distance_[toVertex]) {
          distance_[toVertex] = newDistance;
          predecessor_[toVertex] = edgeId;
          if (!inQueue[toVertex]) {
            q.emplace(toVertex);
            inQueue[toVertex] = true;
          }
        }
      }
    }
    return distance_[sink] != kInfinityCost;
  }

  static constexpr auto GetFindPathAlgorithm() -> decltype(auto) {
    return kFindPathType == FindPathType::kDijkstra
               ? &TMinCostFlow::Dijkstra
               : &TMinCostFlow::EdmondsKarp;
  }
};

} // namespace NMCF