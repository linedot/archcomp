// TODO: Consider exposing some sort of iterators so the storage becomes usable with smth. like std::transform? Or at least investigate if this is a good idea

#ifndef ARCHCOMP_COORDINATE_STORAGE
#define ARCHCOMP_COORDINATE_STORAGE

#include <cache_util.hpp>
#include <compiler_hints.hpp>
#include <dynamic_aligned_allocator.hpp>
#include <simd_util.hpp>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

namespace archcomp
{

/** \brief coordinate access type
 *
 **/
enum class access_type : std::uint64_t
{
    interleaved,         //!< values will be interleaved (AoS) (x1,y1,z1,x2,y2,z2,x3,...)
    interleaved_simd1,   //!< multiples of simd_size bytes of values will be interleaved
    interleaved_simd2,   //!< multiples of 2*simd_size bytes of values will be interleaved
    interleaved_simd4,   //!< multiples of 4*simd_size bytes of values will be interleaved
    interleaved_simd8,   //!< multiples of 8*simd_size bytes of values will be interleaved
    interleaved_simd_cl, //!< interleave n*simd_size bytes of values so that m cache lines 
                         //!< are filled, minimizing both n and m, i.e 
                         //!< `if (2*simd_size == cache_line_size)`, then 2*simd_size bytes 
                         //!< of values will be interleaved. Or - for a slightly weirder example:
                         //!< `if (3*simd_size == 2*cache_line_size)`, then 3*simd_size bytes 
                         //!< of values will be interleaved
    separate             //!< separate memory pool for each coordinate (SoA)
};

/** \brief alignment type
 *
 **/
enum class align_to : std::uint64_t
{
    simd,       //!< align to simd_size
    cache_line, //!< align to cache_line_size
    l1bank,     //!< align to cache_line_size*l1d_assoc
    l1          //!< align to l1d_size. This will fail if l1d_size != 2^n
};




/** \brief vector (coordinate array) specification
 *
 *  \details first entry is number of dimensions, second entry is number of elements
 */
typedef std::tuple<std::size_t, std::size_t> vector_spec;


/** \brief coordinate specification
 *
 */
template<bool rw>
struct coord_spec
{
    std::size_t vector_id; ///< id of vector in the coordinate storage
    std::size_t coord;     ///< coordinate in the vector (x=0,y=1,...)
    constexpr static bool writeable = rw; ///< Whether this will be a writeable parameter
};


template<typename first, typename second>
struct first_type
{
    typedef first type;
};

/** \brief arguments to be passed to a tranformation function
 */
template<typename scalar, typename  ... argument_types>
struct argument_pack
{
    std::tuple<argument_types... > args;
};

/** \brief pack of coordinate specification
 *
 *  \tparam coord_spec_types types of coordinate specs, must be coord_spec<false> or 
 *          coord_spec<true> and shouldn't be specified explicitly
 */
template<typename ... coord_spec_types>
  requires(
      (std::is_same_v<coord_spec<false>,coord_spec_types> ||
       std::is_same_v<coord_spec<true>,coord_spec_types>) && ... )
struct coord_spec_pack
{
    typedef std::tuple<coord_spec_types...> coord_specs;

    coord_specs values;


    template<typename scalar, bool rw>
    struct ref_or_value;

    template<typename scalar>
    struct ref_or_value<scalar, true>
    {
        typedef scalar& type;
    };

    template<typename scalar>
    struct ref_or_value<scalar, false>
    {
        typedef const scalar type;
    };

    // TODO: I'm not sure this is the best name for it
    template<typename scalar>
    struct scalar_args 
    {
        /** \brief argument pack type corresponding to coordinate specification
         */
        typedef argument_pack<scalar,
            typename ref_or_value<scalar,coord_spec_types::writeable>::type ...>
            pack_type;

        /** \brief constructs argument pack with the help of a pointer generator
         *
         *  \details The argument passed to the transformation function is constructed
         *           with this function
         *
         *  \tparam ptr_tuple_type type of tuple of base pointers to be used with
         *          ptr_generator
         *
         *  \param[in] ptr_generator callable that will generate pointers to values
         *             to be packed into the argument pack.
         *             must accept 2 parameters - a coord_spec and a scalar* base
         *             pointer and must return a scalar* pointer to the appropriate value
         *  \param[in] specs tuple of coord_spec objects
         *  \param[in] ptr_tuple tuple of base pointers to be used with ptr_generator
         *
         * \return argument_pack to be passed to the transformation function
         */
        template<typename ptr_tuple_type>
        static pack_type construct(auto ptr_generator,
                            coord_specs specs,
                            ptr_tuple_type ptr_tuple)
        {
            auto get_refs = [&] (auto&& ... specs)
                -> decltype(pack_type::args)
            {
                return std::apply([&](auto&& ... ptr)
                -> decltype(pack_type::args)
                    {
                        return std::tuple<typename ref_or_value<scalar,
                            std::remove_reference<decltype(specs)>::type::writeable>::type ...>
                        {
                            std::forward<typename ref_or_value<scalar,
                            std::remove_reference<decltype(specs)>::type::writeable>::type
                            >(*ptr_generator(specs, ptr)) ...
                        };
                    }, std::forward<ptr_tuple_type>(ptr_tuple));
            };
            return {.args = std::apply(get_refs, 
                    std::forward<coord_specs>(specs))};
        }
    };
};

/** \brief create a pack of coordinate specifications
 */
template<typename ... coord_spec_types>
coord_spec_pack<coord_spec_types ...> make_coord_spec_pack(coord_spec_types ... coord_specs)
{
    return coord_spec_pack<coord_spec_types ...>
    {
        .values = std::make_tuple(coord_specs ...)
    };
};


/** \brief creates a writeable coordinate specification
 *
 *  \param[in] vector_id id of the vector for this coordinate
 *  \param[in] coord which coordinate within the vector to use. the vector
 *             must have been specified with enough dimensions
 */
constexpr auto make_rw_coord_spec (std::size_t vector_id, std::size_t coord)
{
    return coord_spec<true>{.vector_id=vector_id, .coord=coord};
};

/** \brief creates a read-only coordinate specification
 *
 *  \param[in] vector_id id of the vector for this coordinate
 *  \param[in] coord which coordinate within the vector to use. the vector
 *             must have been specified with enough dimensions
 */
constexpr auto make_ro_coord_spec (std::size_t vector_id, std::size_t coord)
{
    return coord_spec<false>{.vector_id=vector_id, .coord=coord};
};


/** \brief index for simple linear transformations
 *
 *  \details (for strong typing)
 */
struct transformation_index
{
    std::size_t idx; ///< actual index
};

/** \brief concept for a valid transformation function
 *  
 *  \tparam T function type the concept is being applied to
 *  \tparam scalar type of scalar being transformed
 *  \tparam coord_spec_pack_type type of the coordinate spec pack defining which
 *          vectors and coordinates will be transformed
 */
template<typename T, typename scalar,  typename coord_spec_pack_type>
concept transformer_func = requires(T func, transformation_index tidx, 
        coord_spec_pack_type coord_spec_pack,
        typename coord_spec_pack_type::template scalar_args<scalar>::pack_type arg_pack)
{

    func(tidx, arg_pack);
};

/** \brief storage for multidimensional coordinate arrays/vectors (spatial coordinates, velocities, forces, etc...)
 *  \tparam scalar underlying datatype of stored elements
 *  \tparam coordinate_access_type how the elements of the vectors will be accessed
 *
 *  \see access_type
 **/
template<typename scalar, access_type coordinate_access_type>
class coordinate_storage
{
public:


    /** \brief Create storage and allocate mempools 
     *
     *  \tparam vector_spec_types types of vector specs. Must be vector_spec and 
     *          shouldn't be provided explicitly
     *
     *  \param[in] align_mempools_to specifies how to align the memory pools
     *  \param[in] ci Reference to a cache_info instance. If not provided a new 
     *             one will be created
     *  \param[in] vector_specs specifications of vectors. The first value in the pair is the number of dimensions ( i.e 3 for x,y,z; 2 for x,y, etc...), the second is the number of Elements to reserve.
     *
     *  \sa align_to,access_type,cache_info
     */
    template<typename... vector_spec_types>
    explicit coordinate_storage(
            align_to   align_mempools_to = align_to::l1bank,
            cache_info ci = cache_info(),
            vector_spec_types... vector_specs)
        requires (std::is_same_v<vector_spec,vector_spec_types> && ...)
                 && (0 != sizeof...(vector_spec_types))
        : 
            ci(ci),
            allocator(get_alignment(align_mempools_to, ci)),
            dims{{std::get<0>(vector_specs)...}}
    {
        if ( (... || (0 == std::get<0>(vector_specs) || 0 == std::get<1>(vector_specs) )))
        {
            throw std::invalid_argument("Vector spec can't have 0 dimensions or 0 elements");
        }
        auto create_vector = [this](std::tuple<std::size_t, std::size_t> spec)
        {
            const auto& vector_dims = std::get<0>(spec);
            const auto& vector_size = std::get<1>(spec);

            auto dynamic_divisible_by = get_block_size(vector_size);

            // Let's assume element count is divisible by some small power of 2 so we
            // vectorize more easily
            if (0 != (vector_size % divisible_by))
            {
                throw std::invalid_argument(
                        "Element count is not a multiple of divisible_by");
            }

            if (0 != (vector_size % dynamic_divisible_by))
            {
                throw std::invalid_argument(
                        "Element count is not a multiple of dynamic_divisible_by");
            }

            switch(coordinate_access_type)
            {
                // one memory pool per coordinate
                case access_type::separate:
                {
                    for(std::size_t i = 0 ; i< vector_dims; i++)
                    {
                        std::vector<scalar, decltype(allocator)> mempool(allocator);
                        mempool.resize(vector_size);
                        memory_pools.push_back(std::move(mempool));
                    }

                    std::size_t n = 0;
                    break;
                }
                // one memory pool per vector
                case access_type::interleaved:
                case access_type::interleaved_simd1:
                case access_type::interleaved_simd2:
                case access_type::interleaved_simd4:
                case access_type::interleaved_simd8:
                case access_type::interleaved_simd_cl:
                {
                    std::vector<scalar, decltype(allocator)> mempool(allocator);
                    mempool.resize(vector_dims*vector_size);
                    memory_pools.push_back(std::move(mempool));

                    std::size_t n = 0;

                    break;
                }
                default:
                    throw std::invalid_argument("Invalid access type");
            }
        };
        (create_vector(vector_specs),...);
    }

    /** \brief get a reference to a specific memory pool directly
     *
     *  \param[in] id id of the memory pool
     *
     *  \return reference to the memory pool with that id
     *  \throw std::invalid_argument if id >= memory_pools.size()
     */
    std::vector<scalar,dynamic_aligned_allocator<scalar>>& get_mempool(const std::size_t id)
    {
        if (memory_pools.size() <= id)
        {
            throw std::invalid_argument("Invalid memory pool id");
        }
        return memory_pools[id];
    }

    /** \brief Transforms multiple vectors using the provided function
     *
     *  \tparam coord_spec_types types of specified coordinates. Must be coord_spec 
     *          and shouldn't be specified explicitly by user
     *  \tparam func_type type of the function that will be called, \see func
     *
     *  \param[in] func function that will be called for all elements. Must
     *             take one std::size_t index and as many "scalar"s or "scalar&"s
     *             as coordinate specs specified
     *  \param[in] coord_spec_pack pack containing coord_spec that specify where to take the 
     *             parameters to the function from
     *
     *  \sa coord_spec,coord_speck_pack,make_coord_spec_pack
     */
    template<typename coord_spec_pack_type,
             transformer_func<scalar, coord_spec_pack_type> func_type>
    inline void transform(func_type func, coord_spec_pack_type&& coord_spec_pack )
    {
        using scalar_args_type = typename coord_spec_pack_type::template scalar_args<scalar>;
        using value_pack_type = typename scalar_args_type::pack_type;
        using coord_spec_tuple_type = typename coord_spec_pack_type::coord_specs;

        std::vector<std::reference_wrapper<memory_pool_type>> pools;

        std::apply([&pools,this](auto&&... specs)
                {
                    pools.insert(pools.end(),{
                            get_mempool(
                                pool_idx_calc(
                                    specs.vector_id,
                                    specs.coord)) ... });
                }, coord_spec_pack.values);

        std::size_t element_count = pools[0].get().size();

        auto elements_divide_by = 1;

        // NOTE: keep an eye on this when modifying anything fundamental
        if (access_type::separate != coordinate_access_type)
        {
            elements_divide_by = dims[0];
        }

        element_count /= elements_divide_by;
        

        for (auto& pool : pools)
        {
            // separate will have same element_count, but interleaved will
            // have dims x element_count
            if (pool.get().size()/elements_divide_by != element_count)
            {
                throw std::runtime_error("memory pools have unequal size");
            }
        }

        // This might help vectorization?
        assume_hint(0 == element_count % divisible_by);

        auto block_count = get_block_count(element_count, dims[0]);
        auto block_size  = get_block_size(element_count);

        // This might help vectorization?
        assume_hint(0 == block_size % block_divisible_by());


        constexpr auto block_start = [](const std::size_t block_id,
                const std::size_t block_size,
                const std::size_t dims)
            -> std::size_t
            {
                return block_id*block_size*dims;
            };

        static_assert(divisible_by == 32, "divisible_by is not 32 and a loop below depending on it being so is unrolled 32 times");

        // For performance it's probably a good idea to
        // spend some time trying to get the compiler to cooperate
        switch(coordinate_access_type)
        {
        case access_type::interleaved:
        {
            // Adapted from https://stackoverflow.com/questions/10604794/convert-stdtuple-to-stdarray-c11
            // (the C++17 answer)
            auto get_pointers = [&pools,this] (auto&& ... specs)
            {
                return std::tuple{std::forward<scalar *>(
                        &pools[
                        pool_idx_calc(
                            specs.vector_id,
                            specs.coord)].get()
                        [specs.coord]) ...};
            };
            
            auto pointers = std::apply(get_pointers, 
                    std::forward<coord_spec_tuple_type>(coord_spec_pack.values));


            for(std::size_t i = 0; i < element_count; i+= divisible_by)
            {
                #pragma omp unroll
                for(std::size_t j = 0; j < divisible_by; j++)
                {
                    auto pointer_generator = [i,j,this](auto spec, scalar * ptr)
                        -> scalar *
                    {
                        return (ptr+(i+j)*dims[spec.vector_id]);
                    };
                    func(transformation_index{i+j},
                            coord_spec_pack_type::template scalar_args<scalar>::construct(
                            pointer_generator,
                            coord_spec_pack.values,
                            pointers
                            ));
                }
            }
            break;
        }
        case access_type::interleaved_simd1:
        case access_type::interleaved_simd2:
        case access_type::interleaved_simd4:
        case access_type::interleaved_simd8:
        case access_type::interleaved_simd_cl:
        {

            for(std::size_t block_id = 0; block_id < block_count; block_id++)
            {
                auto get_pointers = [&pools,block_id,block_size,block_start,this]
                    (auto&& ... specs)
                {
                    return std::tuple{std::forward<scalar * __restrict__>(
                            &pools[
                            pool_idx_calc(
                                specs.vector_id,
                                specs.coord)].get()
                            [
                            (block_start(block_id, block_size,
                                         dims[specs.vector_id])+
                             specs.coord*block_size)
                            ]) ...};
                };
                
                auto pointers = std::apply(get_pointers, 
                        std::forward<coord_spec_tuple_type>(coord_spec_pack.values));
                // Just makes things worse actually
                //#pragma omp for simd order(concurrent)
                for(std::size_t i = 0; i < block_size; i++)
                {

                    auto pointer_generator = [i,this](auto spec, scalar * __restrict__ ptr)
                        -> scalar * __restrict__
                    {
                        return (ptr+i);
                    };
                    value_pack_type args = scalar_args_type::construct(
                            pointer_generator,
                            coord_spec_pack.values,
                            pointers
                            );

                    func(transformation_index{i}, args);
                }
            }
            break;
        }
        case access_type::separate:
        {
            auto get_pointers = [&pools,this] (auto&& ... specs)
            {
                return std::tuple{std::forward<scalar * __restrict__>(
                        &pools[
                        pool_idx_calc(
                            specs.vector_id,
                            specs.coord)].get()
                        [0]) ...};
            };
            
            auto pointers = std::apply(get_pointers, 
                    std::forward<coord_spec_tuple_type>(coord_spec_pack.values));


            for(std::size_t i = 0; i < element_count; i+= divisible_by)
            {
#if defined(__INTEL_LLVM_COMPILER)
                #pragma omp simd 
#elif defined(__llvm__)
                #pragma omp simd
#elif defined(__GNUC__)
                #pragma omp unroll
#endif
                for(std::size_t j = i; j < i+divisible_by; j++)
                {

                    auto pointer_generator = [j,this](auto spec, scalar * __restrict__ ptr)
                        -> scalar * __restrict__
                    {
                        return (ptr+j);
                    };
                    value_pack_type args = scalar_args_type::construct(
                            pointer_generator,
                            coord_spec_pack.values,
                            pointers
                            );

                    func(transformation_index{i}, args);
                }
            }
            break;
        }
        }

    }

private:

    /// Stored vectors must have multiple of this elements
    constexpr static std::size_t divisible_by = 32;

    /** \brief calculate memory pool id from vector id and coordinate
     *  
     *  \param[in] vector_id id of stored vector
     *  \param[in] coord coordinate to access
     *
     *  \return id of memory pool that can be used with get_mempool();
     **/
    constexpr std::size_t pool_idx_calc(std::size_t vector_id, std::size_t coord)
    {
        switch(coordinate_access_type)
        {
            case access_type::interleaved:
            case access_type::interleaved_simd1:
            case access_type::interleaved_simd2:
            case access_type::interleaved_simd4:
            case access_type::interleaved_simd8:
            case access_type::interleaved_simd_cl:
                return vector_id;
            case access_type::separate:
                return vector_id+coord;
            default:
                throw std::runtime_error("pool_idx_calc not implemented for access typ");
        }
    }

    /** \brief Calculate alignment from align_to type and cache info
     *
     *  \param[in] align_mempools_to specifies how to align the memory
     *  \param[in] ci reference to a cache_info instance
     **/
    static std::size_t get_alignment(align_to align_mempools_to, const cache_info& ci)
    {
        switch(align_mempools_to)
        {
            case align_to::simd:
                return simd_size();
            case align_to::cache_line:
                return ci.get_cache_line_size();
            case align_to::l1bank:
                return ci.get_cache_line_size()*ci.get_l1d_assoc();
            case align_to::l1:
                return ci.get_l1d_size();
        }
        throw std::invalid_argument("Invalid align type");
    }

    constexpr std::size_t get_block_count(std::size_t element_count, std::size_t dims)
    {
        switch(coordinate_access_type)
        {
            case access_type::interleaved:
                return element_count/dims;
            case access_type::interleaved_simd1:
                return element_count/dims/(simd_size()/sizeof(scalar));
            case access_type::interleaved_simd2:
                return element_count/dims/(2*simd_size()/sizeof(scalar));
            case access_type::interleaved_simd4:
                return element_count/dims/(4*simd_size()/sizeof(scalar));
            case access_type::interleaved_simd8:
                return element_count/dims/(8*simd_size()/sizeof(scalar));
            case access_type::separate:
                return 1;
            default:
                throw std::runtime_error("access type not supported yet");
        }
    }

    constexpr std::size_t get_block_size(std::size_t element_count)
    {
        switch(coordinate_access_type)
        {
            case access_type::interleaved:
                return 1;
            case access_type::interleaved_simd1:
                return (simd_size()/sizeof(scalar));
            case access_type::interleaved_simd2:
                return (2*simd_size()/sizeof(scalar));
            case access_type::interleaved_simd4:
                return (4*simd_size()/sizeof(scalar));
            case access_type::interleaved_simd8:
                return (8*simd_size()/sizeof(scalar));
            case access_type::separate:
                return element_count;
            default:
                throw std::runtime_error("access type not supported yet");
        }
    }
    
    constexpr static std::size_t block_divisible_by()
    {
        switch(coordinate_access_type)
        {
            case access_type::interleaved:
                return 1;
            case access_type::interleaved_simd1:
                return simd_size()/sizeof(scalar);
            case access_type::interleaved_simd2:
                return 2*simd_size()/sizeof(scalar);
            case access_type::interleaved_simd4:
                return 4*simd_size()/sizeof(scalar);
            case access_type::interleaved_simd8:
                return 8*simd_size()/sizeof(scalar);
            case access_type::separate:
                return divisible_by;
            default:
                throw std::runtime_error("access type not implemented");
        }
    }

    cache_info ci;

    std::vector<std::size_t> dims;

    dynamic_aligned_allocator<scalar> allocator;

    typedef std::vector<scalar, dynamic_aligned_allocator<scalar>> memory_pool_type;

    std::vector<memory_pool_type> memory_pools;
};


} // namespace archcomp
  
#endif // ARCHCOMP_COORDINATE_STORAGE
