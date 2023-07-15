#include <boost/ut.hpp>

#include <cache_util.hpp>
#include <coordinate_storage.hpp>

#include <utility>

#include "common.hpp"

using tidx = archcomp::transformation_index;


constexpr auto csrw = archcomp::make_rw_coord_spec;
constexpr auto csro = archcomp::make_ro_coord_spec;

auto main() -> int
{
    using namespace boost::ext::ut;
    using archcomp::cache_info;
    using archcomp::coordinate_storage;
    using archcomp::access_type;
    using archcomp::align_to;

    using vspec = archcomp::vector_spec;

    using test_cs::all_access_types;
    using test_cs::all_align_types;
    using test_cs::data_types;
    using test_cs::with_access_type;
    using test_cs::with_align_type;
    using test_cs::with_data_type;



    "coordinate_storage_dim3_transform"_test = []()
    {

        cache_info cinfo;


        auto test_transformations = [&cinfo]
            <typename scalar>()
        {

            constexpr vspec small_vector_spec{3,4096};
            // I've thought about it and came to the conclustion that 
            // these short var names are better in this case
            // NOLINTBEGIN(readability-identifier-length)

            constexpr auto adder3 = []([[maybe_unused]] const tidx index,
                    archcomp::argument_pack<scalar,
                          const scalar, const scalar, const scalar,
                          const scalar, const scalar, const scalar,
                          scalar&, scalar&, scalar&> inputs)
            {
                auto& [x1, y1, z1, x2, y2, z2, xout, yout, zout] = inputs.args;

                xout = x1 + x2;
                yout = y1 + y2;
                zout = z1 + z2;
            };

            constexpr auto euclidean3 = []([[maybe_unused]] const tidx index,
                    archcomp::argument_pack<scalar,
                          const scalar, const scalar, const scalar,
                          const scalar, const scalar, const scalar,
                          scalar&, scalar&, scalar&> inputs)
            {
                auto& [x1, y1, z1, x2, y2, z2, xout, yout, zout] = inputs.args;

                xout = x1 * x2 + y1 * y2 + z1 * z2;
            };

            constexpr auto compute3 = []([[maybe_unused]] const tidx index,
                    archcomp::argument_pack<scalar,
                          const scalar, const scalar, const scalar,
                          const scalar, const scalar, const scalar,
                          scalar&, scalar&, scalar&> inputs)
            {
                auto& [x1, y1, z1, x2, y2, z2, xout, yout, zout] = inputs.args;

                auto dx = x1 - x2;
                auto dy = y1 - y2;
                auto dz = z1 - z2;

                xout = dy * zout - dz * yout;
                yout = dz * xout - dx * zout;
                zout = dx * yout - dy * xout;
            };
            // NOLINTEND(readability-identifier-length)

            auto test_adder_euclidean_compute = [&cinfo,small_vector_spec,adder3,euclidean3,compute3]
                <access_type access, align_to align>()
            {
                expect(nothrow([&cinfo,small_vector_spec,adder3,euclidean3,compute3]
                    {
                        auto storage = coordinate_storage<scalar, access>(align, cinfo, 
                                small_vector_spec, 
                                small_vector_spec, 
                                small_vector_spec);

                        storage.transform(adder3,
                                archcomp::make_coord_spec_pack(
                                csro(0,0),csro(0,1),csro(0,2),
                                csro(1,0),csro(1,1),csro(1,2),
                                csrw(2,0),csrw(2,1),csrw(2,2)));

                        storage.transform(euclidean3, 
                                archcomp::make_coord_spec_pack(
                                csro(0,0),csro(0,1),csro(0,2),
                                csro(1,0),csro(1,1),csro(1,2),
                                csrw(2,0),csrw(2,1),csrw(2,2)));

                        storage.transform(compute3, 
                                archcomp::make_coord_spec_pack(
                                csro(0,0),csro(0,1),csro(0,2),
                                csro(1,0),csro(1,1),csro(1,2),
                                csrw(2,0),csrw(2,1),csrw(2,2)));
                    }))
                    << "scalar: " << typeid(scalar).name()
                    << "; align: " << static_cast<std::uint64_t>(align)
                    << "; access: " << static_cast<std::uint64_t>(access);
            };

            with_access_type(
                    std::make_index_sequence<
                    std::tuple_size_v<decltype(all_access_types)>>{},
                    test_adder_euclidean_compute);
        };
        
        with_data_type(std::make_index_sequence<
                std::tuple_size_v<decltype(data_types)>>{},
                test_transformations);

    };
}
