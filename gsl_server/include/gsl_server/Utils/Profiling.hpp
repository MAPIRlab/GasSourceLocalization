#pragma once

#if defined TRACY_ENABLE && defined TRACY_INSTRUMENTATION
#include <tracy/Tracy.hpp>
#else

#define ZoneScoped

#endif