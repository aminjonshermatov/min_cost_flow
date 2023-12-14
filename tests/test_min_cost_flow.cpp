#include <gtest/gtest.h>

#include <min_cost_flow/min_cost_flow.hpp>

#include <random>
#include <chrono>

namespace {

    constexpr std::size_t MAXIMUM_NUMBER_OF_VERTEX_IN_TIER = 10u;

    std::mt19937 generator(std::chrono::steady_clock::now().time_since_epoch().count());
    auto randRange(const int32_t min, const int32_t max) -> decltype(auto) { // both inclusive, i.e [min, max]
        return std::uniform_int_distribution<>{min, max}(generator);
    }

}

class TRandomNetworkFixture : public ::testing::Test {
protected:
    using TUnit = int;
    std::size_t n{}, source, sink;
    struct TEdge {
        std::size_t from{}, to{};
        TUnit capacity{}, cost{};
        TEdge(std::size_t from_, std::size_t to_, TUnit capacity_, TUnit cost_) : from(from_), to(to_), capacity(capacity_), cost(cost_) { }
    };
    std::vector<TEdge> edges;

    void SetUp() override {
        std::array<std::size_t, 3> tiers{};
        std::size_t total{};
        for (auto& tier : tiers) {
            tier = randRange(1, MAXIMUM_NUMBER_OF_VERTEX_IN_TIER);
            total += tier;
        }
        source = 0u, sink = total + 1;
        n = sink + 1;
        std::cerr << "tiers: " << tiers[0] << ' ' << tiers[1] << ' ' << tiers[2] << std::endl;
        edges.reserve(tiers[0] * tiers[1] * tiers[2] + tiers[0] + tiers[2]);

        for (auto a = 1u; a <= tiers[0]; ++a) {
            edges.emplace_back(source, a, randRange(2, 5), randRange(1, 100));
        }
        for (auto a = 1u; a <= tiers[0]; ++a) {
            for (auto b = 1u; b <= tiers[1]; ++b) {
                for (auto c = 1u; c <= tiers[2]; ++c) {
                    edges.emplace_back(a, tiers[0] + b, randRange(1, 5), randRange(1, 100));
                    edges.emplace_back(tiers[0] + b, tiers[0] + tiers[2] + c, randRange(1, 5), randRange(1, 100));
                }
            }
        }
        for (auto c = 1u; c <= tiers[2]; ++c) {
            edges.emplace_back(tiers[0] + tiers[1] + c, sink, randRange(2, 5), randRange(1, 100));
        }
    }
};

TEST_F(TRandomNetworkFixture, MinCostFlow) {
    NMinCostFlow::TMinCostFlow<TUnit, NMinCostFlow::FindPathType::kDijkstra> mcf(n);
    std::cerr << n << std::endl;
    for (auto& [from, to, capacity, cost] : edges) {
        std::cerr << from << ' ' << to << ' ' << capacity << ' ' << cost << std::endl;
        mcf.add_edge(from, to, capacity, cost, -1);
    }
    const auto [flow, cost] = mcf.flow(source, sink);
    std::cerr << flow << ' ' << cost << std::endl;
}

int main(int argc, char* argv[]) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}