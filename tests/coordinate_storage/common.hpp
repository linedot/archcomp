#include <coordinate_storage.hpp>
#include <cache_util.hpp>

#include <utility>

namespace test_cs
{

    // Boilerplate to iterate through access and align types
    // =====================================================
    constexpr auto all_access_types = std::make_tuple(
            archcomp::access_type::interleaved,
            archcomp::access_type::interleaved_simd1,
            archcomp::access_type::interleaved_simd2,
            archcomp::access_type::interleaved_simd4,
            archcomp::access_type::interleaved_simd8,
            archcomp::access_type::separate
            );
    constexpr auto all_align_types = std::make_tuple(
            archcomp::align_to::simd,
            archcomp::align_to::cache_line,
            archcomp::align_to::l1bank,
            archcomp::align_to::l1
            );

    auto with_align_type = []
    <archcomp::access_type access, std::size_t ... align_index>
    (std::index_sequence<align_index...>, auto do_this)
    {
        (do_this.template operator()< 
         access, 
         std::get<align_index>(all_align_types)>(), ...) ;
    };

    auto with_access_type = []
    <std::size_t ... access_idx>
    (std::index_sequence<access_idx...>, auto do_this)
    {
        (with_align_type.template operator()<
         std::get<access_idx>(all_access_types)>(
             std::make_index_sequence<
             std::tuple_size_v<decltype(all_align_types)>>{},
             do_this
             ), ...);
    };
    // End of boilerplate to iterate through access and align types
    // =====================================================
    

    // Boilerplate to iterate over relevant data types
    // =====================================================
    constexpr auto data_types = std::tuple<double, float, 
         std::uint64_t, std::uint32_t, std::uint16_t, std::uint8_t,
         std::int64_t, std::int32_t, std::int16_t, std::int8_t>{};

    // std::remove_const is in type_traits but it's a heavy header, so let's avoid it here
    template<typename T> struct remove_const {typedef T type;};
    template<typename T> struct remove_const<const T> {typedef T type;};
    
    auto with_data_type = []
    <std::size_t ... data_type_idx>
    (std::index_sequence<data_type_idx...>, auto do_this)
    {
        (do_this.template operator()<typename remove_const<
             typename std::tuple_element<
                 data_type_idx,decltype(data_types)>::type>::type>(), ...);
    };
    // End of boilerplate to iterate over relevant data types
    // =====================================================

}
