#pragma once

#include "common.hpp"

namespace MinCostFlow {

    template <typename T> class MinCostFlowBase {

        std::size_t n;
        std::vector<std::vector<std::size_t>> g;
        std::vector<MinCostFlow::FlowEdge<T>> edges;
        std::vector<T> pot, dist;
        std::vector<int> pre;
    public:
        explicit MinCostFlowBase(std::size_t n_) : n(n_) {
            g.resize(n);
            pot.resize(n);
            dist.resize(n);
            pre.resize(n);
        }

        auto add_edge(int u, int v, T cap, T cost, int id) -> void {
            g[u].emplace_back(edges.size());
            edges.emplace_back(u, v, cap, cost, id);
            g[v].emplace_back(edges.size());
            edges.emplace_back(v, u, 0, -cost, id + n);
        }

        bool dijkstra(std::size_t source, std::size_t sink) {
            dist.assign(n, std::numeric_limits<T>::max());
            pre.assign(n, -1);


            std::priority_queue<U, std::vector<U>, std::greater<>> pq;
            pq.emplace(dist[S] = 0, S);
            while (!pq.empty()) {
                auto [d, u] = pq.top();
                pq.pop();
                if (d > dist[u]) continue;
                for (auto id : g[u]) {
                    auto v = edges[id].v;
                    if (edges[id].cap - edges[id].flow <= 0) continue;
                    auto nd = d + edges[id].cost + pot[u] - pot[v];
                    if (nd < dist[v]) {
                        dist[v] = nd;
                        pre[v] = id;
                        pq.emplace(dist[v] = nd, v);
                    }
                }
            }
            return dist[T] != inf;
        }

        decltype(auto) flow(int S, int T) {
            fill(pot.begin(), pot.end(), 0);
            bool any = true;
            for (int _ = 0; _ < n && any; ++_) {
                any = false;
                for (const auto &edge : edges) {
                    if (edge.cap == 0) continue;
                    auto npot = pot[edge.u] + edge.cost;
                    if (pot[edge.v] > npot) {
                        any |= true;
                        pot[edge.v] = npot;
                    }
                }
            }
            assert(!any); // cycle of negative weight

            i64 flow = 0, cost = 0;
            while (dijkstra(S, T)) {
                for (int v = 0; v < n; ++v) {
                    pot[v] += dist[v];
                }
                auto nf = inf;
                for (int v = T; v != S; v = edges[pre[v]].u) {
                    nf = std::min(nf, edges[pre[v]].cap - edges[pre[v]].flow);
                }
                flow += nf;
                cost += (pot[T] - pot[S]) * nf;
                for (int v = T; v != S; v = edges[pre[v]].u) {
                    edges[pre[v]].flow += nf;
                    edges[pre[v] ^ 1].flow -= nf;
                }
            }
            const auto mx_id = max_element(edges.begin(), edges.end(), [](const FlowEdge &lhs, const FlowEdge &rhs) {
                return lhs.id < rhs.id;
            })->id;
            std::vector<i64> fall_through(mx_id + 1, 0);
            //std::map<int, i64> fall_through;
            for (const auto &edge : edges) {
                if (edge.id != -1 && edge.flow > 0) {
                    fall_through[edge.id] = edge.flow;
                }
            }
            return std::tuple{flow, cost, std::move(fall_through)};
        }
    };

}
