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

set(project_include_dirs
    ${CMAKE_SOURCE_DIR}/include 
    ${HWLOC_INCLUDE_DIRS})
set(project_library_dirs
    ${HWLOC_LIBRARY_DIRS})
set(project_libraries
    fmt::fmt
    ${HWLOC_LIBRARIES})
enable_testing()
add_subdirectory(tests)

include (cmake/Doxygen.cmake)
