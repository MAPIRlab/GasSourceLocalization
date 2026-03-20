#pragma once
#include "Logging.hpp" // IWYU pragma: export
#include "ConditionalMacros.hpp" // IWYU pragma: export

#define CLOSE_PROGRAM raise(SIGTRAP)