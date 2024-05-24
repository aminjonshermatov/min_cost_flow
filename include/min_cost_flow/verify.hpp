#pragma once

#if !MCF_DISABLE_VERIFY

#include <format>
#include <iostream>
#include <string_view>

namespace NMCF::NVerify {

auto VerifyFailed(const char *expression, const char *function,
                  const char *file, int64_t line) {
  std::cerr << std::format(
      "Expression: {} failed.\nAt function: {} in file: {}, line: {}\n",
      expression, function, file, line);
  std::abort();
}
auto VerifyFailedWithMessage(const char *expression, const char *function,
                             const char *file, int64_t line,
                             std::string_view message) {
  std::cerr << std::format(
      "{}\nExpression: `{}` failed\nAt function: {}\nIn file: {}\nLine: {}\n",
      message, expression, function, file, line);
  std::abort();
}

} // namespace NMCF::NVerify

#define MCF_VERIFY(expression)                                                 \
  ((!!(expression))                                                            \
       ? ((void)0)                                                             \
       : ::NMCF::NVerify::VerifyFailed(#expression, __PRETTY_FUNCTION__,       \
                                       __FILE__, __LINE__))

#define MCF_VERIFY_MSG(expression, message)                                    \
  ((!!(expression))                                                            \
       ? ((void)0)                                                             \
       : ::NMCF::NVerify::VerifyFailedWithMessage(                             \
             #expression, __PRETTY_FUNCTION__, __FILE__, __LINE__, message))

#else

#define MCF_VERIFY(expression) ((void)(expression))
#define MCF_VERIFY_MSG(expression, message) ((void)(expression))

#endif
