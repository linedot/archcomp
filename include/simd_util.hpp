#ifndef ARCHCOMP_SIMD_UTIL
#define ARCHCOMP_SIMD_UTIL

#include <utility>
#include <compiler_hints.hpp>

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
#elif defined(__ARM_SVE)
std::size_t simd_size()
{
    std::size_t  byte_size = 0;
    __asm__ volatile(
            "incb %[byte_size]\n\t"
            : [byte_size] "=r" (byte_size)
            :
            :
            );
    assume_hint(byte_size >= 128/8);
    return byte_size;
}
#elif defined(__riscv_vector)
std::size_t simd_size()
{
    std::size_t  byte_size = 0;
    __asm__ volatile(
#if defined(RVV_OVERRIDE_VLEN)
#define SPSP(x) #x
#define SPS(x) SPSP(x)
            "mov t0, " SPS(RVV_OVERRIDE_VLEN) "\n\t"
#undef SPS
#undef SPSP
            "vsetvli %[byte_size], t0, e8, m1, ta, ma\n\t"
#else
            "vsetvli %[byte_size], zero, e8, m1, ta, ma\n\t"
#endif
            : [byte_size] "=r" (byte_size)
            :
            :
#if defined(RVV_OVERRIDE_VLEN)
            "t0"
#endif
            );
    assume_hint(byte_size >= 128/8);
    return byte_size;
}
#else
// cppcheck-suppress preprocessorErrorDirective
#error simd_size() not implemented for the target Architecture
#endif // defined(<Arch specific SIMD/Vector macros>)
#endif // !defined(GENERATED_SIMD_SIZE)

} //namespace archcomp

#endif // ARCHCOMP_SIMD_UTIL
