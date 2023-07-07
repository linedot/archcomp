#include <cache_util.hpp>
#include <fmt/format.h>

int main()
{
    const archcomp::cache_info ci;

    fmt::print("Cache line size:   {}\n", ci.get_cache_line_size());
    fmt::print("L1D size:          {}\n", ci.get_l1d_size());
    fmt::print("L1D associativity: {}\n", ci.get_l1d_assoc());
    fmt::print("L2 size:           {}\n", ci.get_l2_size());
    fmt::print("L2 size per core:  {}\n", ci.get_l2_per_core());
    fmt::print("L2 associativity:  {}\n", ci.get_l2_assoc());
    fmt::print("L3 size:           {}\n", ci.get_l3_size());
    fmt::print("L3 size per core:  {}\n", ci.get_l3_per_core());
    fmt::print("L3 associativity:  {}\n", ci.get_l3_assoc());
}
