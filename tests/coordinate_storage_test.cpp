#include <coordinate_storage.hpp>
#include <tuple>

//#include <fmt/format.h>

constexpr std::uint64_t dims = 3;
constexpr std::uint64_t count = 6400;


constexpr std::uint64_t coord_offset = 0;

using tidx = archcomp::transformation_index;

// I've thought about it and came to the conclustion that 
// these short var names are better in this case
// NOLINTBEGIN(readability-identifier-length)
constexpr auto adder1 = []([[maybe_unused]] const tidx index, 
        const float x1, const float x2, float& xout)
{
    xout = x1+x2;
};

constexpr auto adder3 = []([[maybe_unused]] const tidx index,
        const float x1, const float y1, const float z1, 
        const float x2, const float y2, const float z2,
        float& xout, float& yout, float& zout)
{
    xout = x1 + x2;
    yout = y1 + y2;
    zout = z1 + z2;
};

constexpr auto euclidean3 = []([[maybe_unused]] const tidx index,
        const float x1, const float y1, const float z1, 
        const float x2, const float y2, const float z2,
        float& xout, [[maybe_unused]] float& yout, [[maybe_unused]] float& zout)
{
    xout = x1 * x2 + y1 * y2 + z1 * z2;
};

constexpr auto compute3 = []([[maybe_unused]] const tidx index,
        const float x1, const float y1, const float z1, 
        const float x2, const float y2, const float z2,
        float& xout, float& yout, float& zout)
{

    auto dx = x1 - x2;
    auto dy = y1 - y2;
    auto dz = z1 - z2;

    xout = dy * zout - dz * yout;
    yout = dz * xout - dx * zout;
    zout = dx * yout - dy * xout;
};
// NOLINTEND(readability-identifier-length)

constexpr auto csrw = archcomp::make_rw_coord_spec;
constexpr auto csro = archcomp::make_ro_coord_spec;

void test_dim_1(auto& storage)
{

    storage.transform(adder1,
            csro(0,0),
            csro(1,0),
            csrw(2,0));
}

void test_dim_3(auto& storage)
{
    storage.transform(adder3, 
            csro(0,0),csro(0,1),csro(0,2),
            csro(1,0),csro(1,1),csro(1,2),
            csrw(2,0),csrw(2,1),csrw(2,2));

    storage.transform(euclidean3, 
            csro(0,0),csro(0,1),csro(0,2),
            csro(1,0),csro(1,1),csro(1,2),
            csrw(2,0),csrw(2,1),csrw(2,2));

    storage.transform(compute3, 
            csro(0,0),csro(0,1),csro(0,2),
            csro(1,0),csro(1,1),csro(1,2),
            csrw(2,0),csrw(2,1),csrw(2,2));
}


auto main() -> int
{

    using vspec = archcomp::vector_spec;
    using archcomp::coordinate_storage;
    using archcomp::align_to;
    using archcomp::access_type;

    try
    {
    const archcomp::cache_info cinfo;
    coordinate_storage<float, access_type::interleaved> storage_interleaved(
            align_to::l1bank,
            cinfo,
            vspec{dims, count},
            vspec{dims, count},
            vspec{dims, count});

    test_dim_1(storage_interleaved);
    test_dim_3(storage_interleaved);

    coordinate_storage<float, access_type::interleaved_simd1> storage_simd1(
            align_to::l1bank,
            cinfo,
            vspec{dims, count},
            vspec{dims, count},
            vspec{dims, count});

    test_dim_1(storage_simd1);
    test_dim_3(storage_simd1);

    coordinate_storage<float, access_type::interleaved_simd4> storage_simd4(
            align_to::l1bank,
            cinfo,
            vspec{dims, count},
            vspec{dims, count},
            vspec{dims, count});

    test_dim_1(storage_simd4);
    test_dim_3(storage_simd4);


    coordinate_storage<float, access_type::interleaved_simd8> storage_simd8(
            align_to::l1bank,
            cinfo,
            vspec{dims, count},
            vspec{dims, count},
            vspec{dims, count});

    test_dim_1(storage_simd8);
    test_dim_3(storage_simd8);


    coordinate_storage<float, access_type::separate> storage_separate(
            align_to::l1bank,
            cinfo,
            vspec{dims, count},
            vspec{dims, count},
            vspec{dims, count});
    
    test_dim_1(storage_separate);
    test_dim_3(storage_separate);


// TODO: implement outer_transform interface
//     constexpr std::size_t smallv_size = 2688;
//     constexpr std::size_t bigv_size = 1<<10;
//     constexpr std::size_t result_size = bigv_size*smallv_size;
//     coordinate_storage<float, access_type::interleaved_simd8> cs_for_outer(
//             align_to::l1bank,
//             ci,
//             vspec{3, smallv_size},
//             vspec{1, bigv_size},
//             vspec{3, result_size});
// 
//     cs_for_outer.outer_transform(doathing,
//             3, csro(0,0), csro(0,1), csro(0,2),
//             1, csro(1,0),
//             3, csro(2,0), csro(2,1), csro(2,2));

    }
    catch(std::exception& e)
    {
        try
        {
            fmt::print("Exception thrown during tests: {}\n", e.what());
        }
        catch(...)
        {
            return -3;
        }
        return -1;
    }
    catch(...)
    {
        try
        {
            fmt::print("Exception thrown during tests.\n");
        }
        catch(...)
        {
            return -3;
        }
        return -2;
    }


    return 0;
}
