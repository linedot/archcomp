#ifndef ARCHCOMP_SIMD_UTIL
#define ARCHCOMP_SIMD_UTIL

#include <utility>

namespace archcomp
{

//@SIMD_SIZE_FUNCTION@//

#if !defined(GENERATED_SIMD_SIZE)
#if defined(__AVX512__)
constexpr std::size_t simd_size()
{
    return 512/8;
}
#elif defined(__AVX2__)
constexpr std::size_t simd_size()
{
    return 256/8;
}
#elif defined(__ARM_NEON)
constexpr std::size_t simd_size()
{
    return 128/8;
}
#else
// cppcheck-suppress preprocessorErrorDirective
#error simd_size() not implemented for the target Architecture
#endif // defined(<Arch specific SIMD/Vector macros>)
#endif // !defined(GENERATED_SIMD_SIZE)

} //namespace archcomp

#endif // ARCHCOMP_SIMD_UTIL
