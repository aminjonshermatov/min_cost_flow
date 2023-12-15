#pragma once

#include <cstddef>

namespace NMinCostFlow {

  template<typename TUnit>
  struct TFlowEdge {
    std::size_t from, to;
    TUnit capacity, flow, cost;
    int32_t id;

    TFlowEdge(std::size_t from_, std::size_t to_,
              TUnit capacity_, TUnit cost_,
              int32_t id_)
        : from(from_), to(to_), capacity(capacity_), flow(TUnit{}), cost(cost_), id(id_) {}
  };

} // namespace NMinCostFlow
