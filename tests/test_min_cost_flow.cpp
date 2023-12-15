#include <gtest/gtest.h>

#include <min_cost_flow/min_cost_flow.hpp>
#include <util/network_generator.hpp>

class TRandomNetworkFixture : public ::testing::Test {
protected:
  using TUnit = int;
  std::size_t n{}, source{}, sink{};

  std::vector<NUtil::NNetwork::TEdge<TUnit>> edges;

  void SetUp() override {
    n = NUtil::NRandom::range(3, 1000);
    source = 0u;
    sink = n - 1;
    edges = NUtil::NNetwork::generateWeightedWithCost<TUnit>(n, 60, 1000, 1000);
  }
};

TEST_F(TRandomNetworkFixture, MinCostFlow) {
  NMinCostFlow::TMinCostFlow<TUnit, NMinCostFlow::FindPathType::kDijkstra> mcfDijkstra(n);
  NMinCostFlow::TMinCostFlow<TUnit, NMinCostFlow::FindPathType::kEdmondsKarp> mcfEdmondsKarp(n);
  for (auto &[from, to, capacity, cost]: edges) {
    mcfDijkstra.addEdge(from, to, capacity, cost);
    mcfEdmondsKarp.addEdge(from, to, capacity, cost);
  }
  ASSERT_EQ(mcfDijkstra.flow(source, sink), mcfEdmondsKarp.flow(source, sink));
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}