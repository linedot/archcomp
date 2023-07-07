#include <cstdint>
#include <functional>
#include <initializer_list>
#include <type_traits>
#include <numeric>
#include <utility>
#include <vector>
#include <map>

#include <cache_util.hpp>
#include <simd_util.hpp>
#include <dynamic_aligned_allocator.hpp>
#include <compiler_hints.hpp>

#include <fmt/format-inl.h>

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
 *  \details first entry is the vector id, second entry is the coordinate 
 *           within that vector, third entry is whether the coordinate will
 *           be written to
 */
typedef std::tuple<std::size_t, std::size_t, bool> coord_spec;


/** \brief creates a writeable coordinate specification
 *
 *  \param[in] vector_id id of the vector for this coordinate
 *  \param[in] coord which coordinate within the vector to use. the vector
 *             must have been specified with enough dimensions
 */
constexpr auto make_rw_coord_spec (std::size_t vector_id, std::size_t coord)
{
    return std::make_tuple(vector_id, coord, true);
};

/** \brief creates a read-only coordinate specification
 *
 *  \param[in] vector_id id of the vector for this coordinate
 *  \param[in] coord which coordinate within the vector to use. the vector
 *             must have been specified with enough dimensions
 */
constexpr auto make_ro_coord_spec (std::size_t vector_id, std::size_t coord)
{
    return std::make_tuple(vector_id, coord, false);
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
 */
template<typename T, typename scalar,  typename... Args>
concept transformer_func = requires(T func, transformation_index tidx, Args... args)
{
    func(tidx, args...);
}  // TODO: maybe it's somehow possible to check whether a parameter is ro/rw and then
   // check the type accordingly?
  && ((std::is_same_v<scalar,Args> || std::is_same_v<scalar&,Args>) && ... );

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

    /** \brief provide a member type that is always the first type
     *
     *  \details using this with parameter packs to specify specific parameter types to
     *           variadic templates from parameter packs of other types
     */
    template<typename T, typename U>
    struct first_type_of_two
    {
        using type = T;
    };


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
    coordinate_storage(
            align_to    align_mempools_to = align_to::l1bank,
            cache_info ci = cache_info(),
            vector_spec_types... vector_specs)
        requires (std::is_same_v<vector_spec,vector_spec_types> && ...)
        : 
            ci(ci),
            allocator(get_alignment(align_mempools_to, ci)),
            dims{{std::get<0>(vector_specs)...}}
    {
        auto create_vector = [this](std::tuple<std::size_t, std::size_t> spec)
        {
            auto dynamic_divisible_by = get_block_size(std::get<1>(spec));

            // Let's assume element count is divisible by some small power of 2 so we
            // vectorize more easily
            if (0 != (std::get<1>(spec) % divisible_by))
            {
                throw std::invalid_argument(
                        fmt::format(
                        "Element count is not a multiple of {}", divisible_by));
            }

            if (0 != (std::get<1>(spec) % dynamic_divisible_by))
            {
                throw std::invalid_argument(
                        fmt::format(
                        "Element count is not a multiple of {}", dynamic_divisible_by));
            }

            switch(coordinate_access_type)
            {
                // one memory pool per coordinate
                case access_type::separate:
                {
                    auto dims = std::get<0>(spec);
                    for(std::size_t i = 0 ; i< dims; i++)
                    {
                        std::vector<scalar, decltype(allocator)> mempool(allocator);
                        mempool.resize(sizeof(scalar)*std::get<1>(spec));
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
                    mempool.resize(std::get<0>(spec)*sizeof(scalar)*std::get<1>(spec));
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
     *  \param[in] coords objects of type coord_spec that specify where to take the 
     *             parameters to the function from
     */
    template<
        typename... coord_spec_types,
        transformer_func<
            scalar, 
            typename first_type_of_two<scalar&, coord_spec_types>::type ...
            > func_type>
    inline void transform(func_type func, coord_spec_types... coords)
        requires (std::is_same_v<coord_spec,coord_spec_types> && ...)
    {
        using tidx = transformation_index;
        std::vector<
            std::reference_wrapper<
                std::vector<scalar,dynamic_aligned_allocator<scalar>>>> pools;

        std::size_t element_count;

        pools.insert(pools.end(),{
                get_mempool(
                        pool_idx_calc(std::get<0>(coords), std::get<1>(coords)))...});

        element_count = pools[0].get().size();

        auto elements_divide_by = 1;

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
        //assume_hint(0 == element_count % divisible_by);

        auto block_count = get_block_count(element_count, dims[0]);
        auto block_size  = get_block_size(element_count);

        // This might help vectorization?
        //assume_hint(0 == block_size % block_divisible_by());


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


            std::array<scalar * __restrict__, sizeof...(coords)> pointers =
            {
                {
                     &pools[
                         pool_idx_calc(
                         std::get<0>(coords), 
                         std::get<1>(coords))].get()[
                    std::get<1>(coords)]...
                }
            };

            for(std::size_t i = 0; i < element_count; i+= divisible_by)
            {
                #pragma omp unroll
                for(std::size_t j = 0; j < divisible_by; j++)
                {
                    std::size_t vi = 0;
                    func(tidx{i+j},
                        *(pointers[pack_incrementer<coord_spec_types>(vi)] + 
                        (i+j)*dims[std::get<0>(coords)])...);
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
                std::array<scalar * __restrict__, sizeof...(coords)> pointers =
                {
                    {
                         &pools[
                             pool_idx_calc(
                             std::get<0>(coords), 
                             std::get<1>(coords))].get()[
                        (block_start(block_id,block_size,
                                    dims[std::get<0>(coords)]) +
                        std::get<1>(coords)*block_size)]...
                    }
                };
                // Just makes things worse actually
                //#pragma omp for simd order(concurrent)
                for(std::size_t i = 0; i < block_size; i++)
                {

                    std::size_t vi = 0;
                    func(tidx{i+0}, *(pointers[pack_incrementer<coord_spec_types>(vi)]+i+0)...);
                }
            }
            break;
        }
        case access_type::separate:
        {
            std::array<scalar * __restrict__, sizeof...(coords)> pointers =
            {
                {
                     &pools[
                         pool_idx_calc(
                         std::get<0>(coords), 
                         std::get<1>(coords))].get()[0]...
                }
            };
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
                    std::size_t vi = 0;
                    func(tidx{j},
                     *(pointers[
                         pack_incrementer<coord_spec_types>(vi)]+j)...);
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



    template<typename T>
    constexpr std::size_t pack_incrementer(std::size_t& index)
    {
        auto result = index;
        index++;
        return result;
    }

    cache_info ci;

    std::vector<std::size_t> dims;

    dynamic_aligned_allocator<scalar> allocator;

    std::vector<
        std::vector<scalar,dynamic_aligned_allocator<scalar>>>
            memory_pools;
};


} // namespace archcomp
