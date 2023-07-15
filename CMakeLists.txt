cmake_minimum_required(VERSION 3.25)

if (NOT DEFINED CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
endif ()

if (NOT DEFINED CMAKE_CXX_EXTENSIONS)
    set(CMAKE_CXX_EXTENSIONS OFF)
endif ()

project(archcomp
    VERSION 0.1.0
    LANGUAGES CXX C)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

find_package(fmt REQUIRED)
# FindHWLOC.cmake from libeigen, could also use pkg-config
find_package(HWLOC REQUIRED)


# I actually wanted to just add some reasonable defaults to CMAKE_CXX_FLAGS,
# but the way I'm doing this here doesn't make the flags visible in the cache.
# However - now that I'm thinking about it - these should be reasonable defaults
# and if other flags get added on top (ARM SVE, AVX512), this doesn't break anything
set(architecture_flags)
message(STATUS "Architecture: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "Compiler ID:  ${CMAKE_CXX_COMPILER_ID}")
set(x86_64_list "x86_64" "AMD64")
set(arm64_list "AArch64" "aarch64" "ARM64")
set(rv64_list "riscv64")
if ( "${CMAKE_SYSTEM_PROCESSOR}" IN_LIST x86_64_list )
    set(architecture_flags $<$<CXX_COMPILER_ID:Clang,GNU,Intel>:-mavx2 -mfma>
                           $<$<CXX_COMPILER_ID:MSVC>:/arch:AVX2>)

elseif ( "${CMAKE_SYSTEM_PROCESSOR}" IN_LIST arm64_list )
    set(architecture_flags $<$<CXX_COMPILER_ID:"Clang","GNU","Intel">:-mneon>)
elseif ( "${CMAKE_SYSTEM_PROCESSOR}" IN_LIST rv64_list )
    # I think targeting RVA23U64 would be reasonable and I assume V will be part of the profile
    # This might be problematic as default on some non-public in-development chips
    set(architecture_flags $<$<CXX_COMPILER_ID:"Clang","GNU","Intel">:-march=rv64imafdcv>)
endif()
message(STATUS "Default architecture flags: ${architecture_flags}")


add_library(archcomp STATIC src/cache_util.cpp)

set(project_include_dirs
    ${CMAKE_SOURCE_DIR}/include 
    ${HWLOC_INCLUDE_DIRS})
set(project_library_dirs
    ${HWLOC_LIBRARY_DIRS})
set(project_libraries
    fmt::fmt
    ${HWLOC_LIBRARIES})
set(project_cxx_flags
    ${architecture_flags})

target_compile_options(archcomp PUBLIC ${project_cxx_flags})
target_include_directories(archcomp PUBLIC ${project_include_dirs})
target_link_directories(archcomp PUBLIC ${project_link_dirs})
target_link_libraries(archcomp PUBLIC ${project_libraries})

# Tests
enable_testing()
add_subdirectory(tests)

# Documentation
include (cmake/Doxygen.cmake)
