#include <cache_util.hpp>

#include <hwloc.h>

archcomp::cache_info::cache_info()
{
    hwloc_topology_t topo;

    auto res = hwloc_topology_init(&topo);
    if ( 0 != res) throw std::runtime_error("Failed to initialize hwloc topology");

    res = hwloc_topology_load(topo);
    if ( 0 != res) throw std::runtime_error("Failed to load hwloc topology");

    auto core_count = hwloc_get_nbobjs_by_type(topo, HWLOC_OBJ_CORE);
    auto pu_count = hwloc_get_nbobjs_by_type(topo, HWLOC_OBJ_PU);

    // Not sure this is the right way to do this
    auto smt_level = pu_count/core_count;

    auto l1d_depth = hwloc_get_cache_type_depth(topo, 1, 
            HWLOC_OBJ_CACHE_DATA);
    if ( -1 == l1d_depth) throw std::runtime_error("Failed to get L1D topology depth");

    // We're assuming all LxDs look the same so we're just getting data from the first one
    // Probably wrong for processors with P/E cores
    auto l1d = hwloc_get_obj_by_depth(topo, l1d_depth, 0);


    // Assuming all caches have the same CL size
    cache_line_size = l1d->attr->cache.linesize;
    l1d_size = l1d->attr->cache.size;
    l1d_assoc = l1d->attr->cache.associativity;

    auto l2_depth = hwloc_get_cache_type_depth(topo, 2, 
            HWLOC_OBJ_CACHE_DATA);
    if ( -1 == l2_depth) throw std::runtime_error("Failed to get L2 topology depth");

    auto l2 = hwloc_get_obj_by_depth(topo, l2_depth, 0);

    l2_size = l2->attr->cache.size;
    l2_assoc = l2->attr->cache.associativity;
    auto l2_cpuset = l2->cpuset;
    l2_per_core = l2_size/(hwloc_bitmap_weight(l2_cpuset)/smt_level);


    auto l3_depth = hwloc_get_cache_type_depth(topo, 3, 
            HWLOC_OBJ_CACHE_DATA);
    if ( -1 == l3_depth) throw std::runtime_error("Failed to get L3 topology depth");

    auto l3 = hwloc_get_obj_by_depth(topo, l3_depth, 0);

    l3_size = l3->attr->cache.size;
    l3_assoc = l3->attr->cache.associativity;
    auto l3_cpuset = l3->cpuset;
    l3_per_core = l3_size/(hwloc_bitmap_weight(l3_cpuset)/smt_level);

    hwloc_topology_destroy(topo);
}
