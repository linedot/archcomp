#include <cache_util.hpp>
#include <fmt/core.h>

auto main() -> int
{
    try
    {
        const archcomp::cache_info cinfo;

        fmt::print("Cache line size:   {}\n", cinfo.get_cache_line_size());
        fmt::print("L1D size:          {}\n", cinfo.get_l1d_size());
        fmt::print("L1D associativity: {}\n", cinfo.get_l1d_assoc());
        fmt::print("L2 size:           {}\n", cinfo.get_l2_size());
        fmt::print("L2 size per core:  {}\n", cinfo.get_l2_per_core());
        fmt::print("L2 associativity:  {}\n", cinfo.get_l2_assoc());
        fmt::print("L3 size:           {}\n", cinfo.get_l3_size());
        fmt::print("L3 size per core:  {}\n", cinfo.get_l3_per_core());
        fmt::print("L3 associativity:  {}\n", cinfo.get_l3_assoc());
    }
    catch(...)
    {
        return -2;
    }
    return 0;
}
