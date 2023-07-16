#include "common.hpp"

#include <boost/ut.hpp>

#include <cache_util.hpp>
#include <coordinate_storage.hpp>

#include <cstddef>
#include <cstdint>
#include <exception>
#include <tuple>
#include <typeinfo>
#include <utility>
#include <vector>


// ut's examples don't try-catch anything, not sure if doing so would break anything
// NOLINTNEXTLINE(bugprone-exception-escape)
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


    // shouldn't be able to create empty storages
    "coordinate_storage_create_empty"_test = []()
    {
        cache_info cinfo;


        // Test creating empty storage
        auto test_empty_storage_creation = [&cinfo]
            <typename scalar>()
        {

            auto create_empty_storage = [&cinfo]
                <access_type access, align_to align>()
            {
                expect(throws([&cinfo]{auto storage =
                            coordinate_storage<scalar, access>(align, cinfo,vspec{0,0});}))
                    << "scalar: " << typeid(scalar).name()
                    << "; align: " << static_cast<std::uint64_t>(align)
                    << "; access: " << static_cast<std::uint64_t>(access);
            };

            with_access_type(
                    std::make_index_sequence<
                    std::tuple_size_v<decltype(all_access_types)>>{},
                    create_empty_storage);
        };
        
        with_data_type(std::make_index_sequence<
                std::tuple_size_v<decltype(data_types)>>{},
                test_empty_storage_creation);

    };

    "coordinate_storage_create_small"_test = []()
    {
        cache_info cinfo;

        // Test creating empty storage
        auto test_empty_storage_creation = [&cinfo]
            <typename scalar>()
        {
            constexpr vspec small_vector_spec{1,1024};

            auto create_empty_storage = [&cinfo,small_vector_spec]
                <access_type access, align_to align>()
            {
                expect(nothrow([&cinfo,small_vector_spec]{auto storage =
                            coordinate_storage<scalar, access>(align, cinfo, small_vector_spec);}))
                    << "scalar: " << typeid(scalar).name()
                    << "; align: " << static_cast<std::uint64_t>(align)
                    << "; access: " << static_cast<std::uint64_t>(access);
            };

            with_access_type(
                    std::make_index_sequence<
                    std::tuple_size_v<decltype(all_access_types)>>{},
                    create_empty_storage);
        };
        
        with_data_type(std::make_index_sequence<
                std::tuple_size_v<decltype(data_types)>>{},
                test_empty_storage_creation);

    };
}
