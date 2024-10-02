#pragma once

// A wrapper around ros logging to be able to use fmt formatting directly in the macros
// use like this: GSL_INFO("Value: {}", value);

#include <fmt/color.h>
#include <fmt/format.h>
#include <rclcpp/rclcpp.hpp>

#define GSL_INFO(...) RCLCPP_INFO(rclcpp::get_logger("GSL"), "%s", fmt::format(__VA_ARGS__).c_str())
#define GSL_INFO_COLOR(color, ...) RCLCPP_INFO(rclcpp::get_logger("GSL"), "%s", fmt::format(fmt::fg(color), __VA_ARGS__).c_str())

#define GSL_WARN(...) RCLCPP_WARN(rclcpp::get_logger("GSL"), "%s", fmt::format(fmt::fg(fmt::terminal_color::yellow), __VA_ARGS__).c_str())

#define GSL_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("GSL"), "%s", fmt::format(fmt::fg(fmt::terminal_color::red), __VA_ARGS__).c_str())

#if GSL_TRACING
    #define GSL_TRACE(...) RCLCPP_INFO(rclcpp::get_logger("GSL - Trace"), "%s", fmt::format(fmt::fg(fmt::terminal_color::green), __VA_ARGS__).c_str())
#else
    #define GSL_TRACE(...)
#endif

#define GSL_DEBUG 1
// Asserts will raise SIGTRAP if condition fails. If you have a debugger, that will stop it in the appropriate line. Otherwise, the program ends.
#if GSL_DEBUG
    #define GSL_ASSERT_MSG(cnd, ...)                                                                                                                 \
        {                                                                                                                                            \
            if (!(cnd))                                                                                                                              \
            {                                                                                                                                        \
                GSL_ERROR("{0}:     At {1}",                                                                                                         \
                          fmt::format(fmt::bg(fmt::terminal_color::red) | fmt::fg(fmt::terminal_color::white) | fmt::emphasis::bold,                 \
                                      fmt::format(__VA_ARGS__)),                                                                                     \
                          fmt::format(fmt::emphasis::bold, "{0}:{1}", __FILE__, __LINE__));                                                          \
                raise(SIGTRAP);                                                                                                                      \
            }                                                                                                                                        \
        }

#else

    #define GSL_ASSERT_MSG(cnd, msg)

#endif

#define GSL_ASSERT(cnd) GSL_ASSERT_MSG(cnd, "")