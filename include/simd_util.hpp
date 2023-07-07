#include <utility>

namespace archcomp
{

//@SIMD_SIZE_FUNCTION@//

#if !defined(GENERATED_SIMD_SIZE)
#if defined(__AVX2__)
constexpr std::size_t simd_size()
{
    return 256/8;
}
#elif defined(__AVX512__)
constexpr std::size_t simd_size()
{
    return 512/8;
}
#else
#error simd_size() not implemented for the target Architecture
#endif // defined(<Arch specific SIMD/Vector macros>)
#endif // !defined(GENERATED_SIMD_SIZE)

} //namespace archcomp
