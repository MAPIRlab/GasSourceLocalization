#pragma once

//Slight convenience macros to avoid having #if #endif everywhere (specially inside function signatures, yuck)

#if USE_GUI
#define IF_GUI(...) __VA_ARGS__
#else
#define IF_GUI(...)
#endif

#if USE_GADEN
#define IF_GADEN(...) __VA_ARGS__
#else
#define IF_GADEN(...)
#endif