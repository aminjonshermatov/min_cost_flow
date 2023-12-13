#pragma once

#include <cstddef>

namespace MinCostFlow {

    template <typename T> struct FlowEdge {
        std::size_t from, to;
        T capacity, flow, cost;
        std::size_t id;
        FlowEdge(std::size_t from_, std::size_t to_,
                 T capacity_, T flow_, T cost_,
                 std::size_t id_)
                : from(from_), to(to_), capacity(capacity_), flow(T{}), cost(cost_), id(id_) { }
    };

}
