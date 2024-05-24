#include <gtest/gtest.h>

#include <min_cost_flow/min_cost_flow.hpp>
#include <util/network_generator.hpp>

class TRandomNetworkFixture : public ::testing::Test {
  protected:
  using TFlow = int32_t;
  using TCost = int32_t;
  uint32_t nVertices{}, source{}, sink{};

  std::vector<NUtil::NNetwork::TEdge<TFlow, TCost>> edges;

  void SetUp() override {
    nVertices = NUtil::NRandom::Range(uint32_t{3}, uint32_t{1000});
    source = 0u;
    sink = nVertices - 1;
    edges = NUtil::NNetwork::GenerateWeightedWithCost<TFlow, TCost>(
        nVertices, 60u, TFlow{1000}, TCost{1000});
  }
};

TEST_F(TRandomNetworkFixture, MinCostFlow) {
  NMCF::TMinCostFlow<TFlow, TCost, NMCF::FindPathType::kDijkstra> mcfDijkstra(
      nVertices);
  NMCF::TMinCostFlow<TFlow, TCost, NMCF::FindPathType::kEdmondsKarp>
      mcfEdmondsKarp(nVertices);
  for (const auto &[from, to, capacity, cost] : edges) {
    mcfDijkstra.AddEdge(from, to, capacity, cost);
    mcfEdmondsKarp.AddEdge(from, to, capacity, cost);
  }
  ASSERT_EQ(mcfDijkstra.Flow(source, sink), mcfEdmondsKarp.Flow(source, sink));
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}