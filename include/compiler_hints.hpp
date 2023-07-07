
#define USE_COMPILER_HINTS

#if defined(USE_COMPILER_HINTS)
#define assume_hint(cond) do { if (!(cond))__builtin_unreachable(); } while(0)
#elif defined(DEBUG)
#include <cassert>
#define assume_hint(cond) assert(cond)
#else
#define assume_hint(cond)
#endif
