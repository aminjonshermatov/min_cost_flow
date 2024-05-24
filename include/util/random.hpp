#pragma once

#include <random>

namespace NUtil::NRandom {

std::random_device rd{};
std::mt19937 generator(rd());

// both inclusive/closed range, i.e [min, max]
auto Range(auto &&min, auto &&max) -> decltype(auto) {
  return std::uniform_int_distribution<std::remove_reference_t<decltype(min)>>{
      min, max}(generator);
}

} // namespace NUtil::NRandom
