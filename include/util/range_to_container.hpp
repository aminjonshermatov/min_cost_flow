#include <ranges>

// https://stackoverflow.com/questions/58808030/range-view-to-stdvector
namespace detail {
  template<typename C>
  struct to_helper {};

  template<typename Container, std::ranges::range R>
    requires std::convertible_to<std::ranges::range_value_t<R>, typename Container::value_type>
  Container operator|(R &&r, to_helper<Container>) {
    return Container(r.begin(), r.end());
  }
} // namespace detail

template<std::ranges::range Container>
  requires(!std::ranges::view<Container>)
auto to() { return detail::to_helper<Container>{}; }
