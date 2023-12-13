#include <gtest/gtest.h>

#include <min_cost_flow/min_cost_flow.hpp>

TEST(MinCostFlow, BasicTest) {
    EXPECT_EQ(flow_sum(1, 2), 3);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}