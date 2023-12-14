#pragma once

#include "common.hpp"

#include <queue>

namespace NMinCostFlow {

    enum class FindPathType : uint8_t { kDijkstra, kEdmondsKarp };

    template <typename T, FindPathType findPathType> class TMinCostFlow {

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

        auto add_edge(std::size_t from, std::size_t to, T capacity, T cost, int32_t id) -> void {
            g[from].emplace_back(edges.size());
            edges.emplace_back(from, to, capacity, cost, id);
            g[to].emplace_back(edges.size());
            edges.emplace_back(to, from, 0, -cost, -1);
        }

        auto flow(const std::size_t source, const std::size_t sink) -> decltype(auto) {
            std::fill(pot.begin(), pot.end(), T{});
            bool any = true;
            for (std::size_t turn = 0u; turn < n && any; ++turn) {
                any = false;
                for (const auto &edge : edges) {
                    if (edge.capacity == T{}) continue;
                    auto newPot = pot[edge.from] + edge.cost;
                    if (pot[edge.to] > newPot) {
                        any |= true;
                        pot[edge.to] = newPot;
                    }
                }
            }
            assert(!any); // cycle of negative weight

            T flow{}, cost{};
            while ((this->*getFindPathAlgorithm())(source, sink)) {
                for (auto v = 0u; v < n; ++v) {
                    pot[v] += dist[v];
                }
                auto newFlow = std::numeric_limits<T>::max();
                for (auto v = sink; v != source; v = edges[pre[v]].from) {
                    newFlow = std::min(newFlow, edges[pre[v]].capacity - edges[pre[v]].flow);
                }
                flow += newFlow;
                cost += (pot[sink] - pot[source]) * newFlow;
                for (auto v = sink; v != source; v = edges[pre[v]].from) {
                    edges[pre[v]].flow += newFlow;
                    edges[pre[v] ^ 1].flow -= newFlow;
                }
            }
            return std::tuple(flow, cost);
        }
    private:
        auto Dijkstra(const std::size_t source, const std::size_t sink) -> bool {
            dist.assign(n, std::numeric_limits<T>::max());
            pre.assign(n, -1);

            std::priority_queue<std::tuple<T, int>, std::vector<std::tuple<T, int>>, std::greater<>> pq;
            pq.emplace(dist[source] = 0, source);
            while (!pq.empty()) {
                auto [d, from] = pq.top();
                pq.pop();
                if (d > dist[from]) continue;
                for (auto id : g[from]) {
                    auto to = edges[id].to;
                    if (edges[id].capacity - edges[id].flow <= T{}) continue;
                    auto newDist = d + edges[id].cost + pot[from] - pot[to];
                    if (newDist < dist[to]) {
                        dist[to] = newDist;
                        pre[to] = id;
                        pq.emplace(dist[to] = newDist, to);
                    }
                }
            }
            return dist[sink] != std::numeric_limits<T>::max();
        }

        auto EdmondsKarp(const std::size_t source, const std::size_t sink) -> bool {
            dist.assign(n, std::numeric_limits<T>::max());
            pre.assign(n, -1);

            std::queue<std::size_t> q;
            std::vector<bool> inQueue(n, false);
            q.emplace(source);
            inQueue[source] = true;
            dist[source] = 0;
            while (!q.empty()) {
                auto from = q.front();
                q.pop();
                inQueue[from] = false;
                for (auto id : g[from]) {
                    auto to = edges[id].to;
                    if (edges[id].capacity - edges[id].flow <= T{}) continue;
                    if (dist[from] + edges[id].cost < dist[to]) {
                        dist[to] = dist[from] + edges[id].cost;
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
            return findPathType == FindPathType::kDijkstra ? &TMinCostFlow::Dijkstra : &TMinCostFlow::EdmondsKarp;
        }
    };

}