#pragma once

#include <chrono>
#include <random>

namespace NUtil::NRandom {

  std::mt19937 generator(std::chrono::steady_clock::now().time_since_epoch().count());

  auto range(const int32_t min, const int32_t max) -> decltype(auto) {// both inclusive, i.e [min, max]
    return std::uniform_int_distribution<>{min, max}(generator);
  }

} // namespace NUtil::NRandom
