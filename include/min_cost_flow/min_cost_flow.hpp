#pragma once

#include "common.hpp"

#include <cassert>
#include <cstdint>
#include <queue>

namespace NMinCostFlow {

  enum class FindPathType : uint8_t {
    kDijkstra,
    kEdmondsKarp
  };

  template<std::integral T, FindPathType findPathType>
  class TMinCostFlow {

    std::size_t n;
    std::vector<std::vector<std::size_t>> g;
    std::vector<NMinCostFlow::TFlowEdge<T>> edges;
    std::vector<T> pot, dist;
    std::vector<int32_t> pre;

  public:
    explicit TMinCostFlow(std::size_t n_) : n(n_) {
      g.resize(n);
      pot.resize(n);
      dist.resize(n);
      pre.resize(n);
    }

    auto addEdge(std::size_t from, std::size_t to, T capacity, T cost, int32_t id = -1) -> void {
      g[from].emplace_back(edges.size());
      edges.emplace_back(from, to, capacity, cost, id);
      g[to].emplace_back(edges.size());
      edges.emplace_back(to, from, 0, -cost, -1);
    }

    auto flow(const std::size_t source, const std::size_t sink) -> decltype(auto) {
      std::fill(pot.begin(), pot.end(), 0);
      if constexpr (findPathType == FindPathType::kDijkstra) {
        bool any = true;
        for (int _ = 0; _ < n && any; ++_) {
          any = false;
          for (const auto &edge: edges) {
            if (edge.capacity == 0) continue;
            auto npot = pot[edge.from] + edge.cost;
            if (pot[edge.to] > npot) {
              any |= true;
              pot[edge.to] = npot;
            }
          }
        }
        assert(!any);// cycle of negative weight
      }

      T flow{}, cost{};
      while ((this->*getFindPathAlgorithm())(source, sink)) {
        if constexpr (findPathType == FindPathType::kDijkstra) {
          for (int v = 0; v < n; ++v) {
            pot[v] += dist[v];
          }
        }

        auto nf = std::numeric_limits<T>::max();
        for (auto v = sink; v != source; v = edges[pre[v]].from) {
          nf = std::min(nf, edges[pre[v]].capacity - edges[pre[v]].flow);
        }
        flow += nf;
        cost += (pot[sink] - pot[source]) * nf;
        for (auto v = sink; v != source; v = edges[pre[v]].from) {
          edges[pre[v]].flow += nf;
          edges[pre[v] ^ 1].flow -= nf;
        }
      }
      return std::tuple(flow, cost);
    }

  private:
    auto dijkstra(const std::size_t source, const std::size_t sink) -> bool {
      dist.assign(n, std::numeric_limits<T>::max());
      pre.assign(n, -1);

      std::priority_queue<std::pair<T, std::size_t>, std::vector<std::pair<T, std::size_t>>, std::greater<>> pq;
      pq.emplace(dist[source] = 0, source);
      while (!pq.empty()) {
        auto [d, from] = pq.top();
        pq.pop();
        if (d > dist[from]) continue;
        for (auto id: g[from]) {
          auto to = edges[id].to;
          if (edges[id].capacity - edges[id].flow <= T{}) continue;
          auto nd = d + edges[id].cost + pot[from] - pot[to];
          if (nd < dist[to]) {
            dist[to] = nd;
            pre[to] = id;
            pq.emplace(dist[to] = nd, to);
          }
        }
      }
      return dist[sink] != std::numeric_limits<T>::max();
    }

    auto edmondsKarp(const std::size_t source, const std::size_t sink) -> bool {
      dist.assign(n, std::numeric_limits<T>::max());
      pre.assign(n, -1);

      std::queue<std::size_t> q;
      std::vector<bool> inQueue(n, false);
      q.emplace(source);
      inQueue[source] = true;
      dist[source] = 0;
      while (!q.empty()) {
        const auto from = q.front();
        q.pop();
        inQueue[from] = false;
        for (const auto id: g[from]) {
          auto to = edges[id].to;
          if (edges[id].capacity - edges[id].flow <= T{}) continue;
          auto nd = dist[from] + edges[id].cost + pot[from] - pot[to];
          if (nd < dist[to]) {
            dist[to] = nd;
            pre[to] = id;
            if (!inQueue[to]) {
              q.emplace(to);
              inQueue[to] = true;
            }
          }
        }
      }
      return dist[sink] != std::numeric_limits<T>::max();
    }

    static constexpr auto getFindPathAlgorithm() -> decltype(auto) {
      return findPathType == FindPathType::kDijkstra ? &TMinCostFlow::dijkstra : &TMinCostFlow::edmondsKarp;
    }
  };

} // namespace NMinCostFlow