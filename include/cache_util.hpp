#ifndef ARCHCOMP_CACHE_UTIL
#define ARCHCOMP_CACHE_UTIL

#include <cstdint>
#include <string>
#include <stdexcept>

namespace archcomp
{

/** \brief Provides basic information about the cache of the running system
 *
 **/
class cache_info
{
public:
    cache_info();

    std::uint64_t get_cache_line_size() const { return cache_line_size; }
    std::uint64_t get_l1d_size() const { return l1d_size; }
    std::uint64_t get_l1d_assoc() const { return l1d_assoc; }
    std::uint64_t get_l2_size() const { return l2_size; }
    std::uint64_t get_l2_assoc() const { return l2_assoc; }
    std::uint64_t get_l2_per_core() const { return l2_per_core; }
    std::uint64_t get_l3_size() const { return l3_size; }
    std::uint64_t get_l3_assoc() const { return l3_assoc; }
    std::uint64_t get_l3_per_core() const { return l3_per_core; }


private:

    // Sensible/conservative defaults
    std::uint64_t cache_line_size = 64;
    std::uint64_t l1d_size = 32*1024;
    std::uint64_t l1d_assoc = 4;
    std::uint64_t l2_size = 256*1024;
    std::uint64_t l2_assoc = 8;
    std::uint64_t l2_per_core = 256*1024;
    std::uint64_t l3_size = 4*1024*1024;
    std::uint64_t l3_assoc = 8;
    std::uint64_t l3_per_core = 1024*1024;
};

} //namespace archcomp

#endif // ARCHCOMP_CACHE_UTIL
