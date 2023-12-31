#ifndef ARCHCOMP_DYNAMIC_ALIGNED_ALLOCATOR
#define ARCHCOMP_DYNAMIC_ALIGNED_ALLOCATOR

#include <cstdlib>
#include <limits>
#include <new>
#include <stdexcept>


namespace archcomp
{
/** \brief aligned allocator with runtime alignment
 *  \tparam T underlying element type
 */
template<class T>
struct dynamic_aligned_allocator
{
    typedef T value_type;
 
    dynamic_aligned_allocator (std::size_t alignment)
        : alignment(alignment)
    {
        if ((0 == alignment) || !((alignment & (alignment - 1)) == 0))
        {
            throw std::invalid_argument("Alignment not a power of 2");
        }
    }
 
    template<class U>
    constexpr dynamic_aligned_allocator (const dynamic_aligned_allocator <U>& other) noexcept 
        : alignment(other.alignment)
    {
    }
    template<class U>
    constexpr dynamic_aligned_allocator (const dynamic_aligned_allocator <U>&& other) noexcept 
        : alignment(other.alignment)
    {
    }
 
    [[nodiscard]] T* allocate(std::size_t n)
    {
        if (n > std::numeric_limits<std::size_t>::max() / sizeof(T))
            throw std::bad_array_new_length();
 
        if (auto p = static_cast<T*>(std::aligned_alloc(alignment, n * sizeof(T))))
        {
            return p;
        }
 
        throw std::bad_alloc();
    }
 
    void deallocate(T* p, std::size_t n) noexcept
    {
        std::free(p);
    }
private:
    std::size_t alignment;

    template<class U>
    friend bool operator==(const dynamic_aligned_allocator <T>& a1, const dynamic_aligned_allocator <U>& a2);

    template<class U>
    friend bool operator!=(const dynamic_aligned_allocator <T>& a1, const dynamic_aligned_allocator <U>& a2);
};
 
template<class T, class U>
bool operator==(const dynamic_aligned_allocator <T>& a1, const dynamic_aligned_allocator <U>& a2) 
{ 
    return a1.alignment == a2.alignment; 
}
 
template<class T, class U>
bool operator!=(const dynamic_aligned_allocator <T>& a1, const dynamic_aligned_allocator <U>& a2)
{
    return a1.alignment != a2.alignment;
}
 
} // namespace archcomp

#endif // ARCHCOMP_DYNAMIC_ALIGNED_ALLOCATOR
