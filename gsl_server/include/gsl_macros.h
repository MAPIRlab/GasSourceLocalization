#pragma once
#include <spdlog/spdlog.h>
#include <fmt/color.h>
#include <fmt/format.h>
#include <signal.h>

#ifdef GSL_DEBUG
#define GSL_ASSERT_MSG(cnd, msg)                                                                                                                        \
    {                                                                                                                                                   \
        if (!(cnd))                                                                                                                                     \
        {                                                                                                                                               \
            spdlog::error("{0}:     At {1}",                                                                                                            \
                          fmt::format(fmt::bg(fmt::terminal_color::red) | fmt::fg(fmt::terminal_color::white) | fmt::emphasis::bold, "ERROR: {}", msg), \
                          fmt::format(fmt::emphasis::bold, "{0}:{1}", __FILE__, __LINE__));                                                             \
            raise(SIGTRAP);                                                                                                                             \
        }                                                                                                                                               \
    }

#else

#define GSL_ASSERT_MSG(cnd, msg)

#endif

#define GSL_ASSERT(cnd) GSL_ASSERT_MSG(cnd, "")
