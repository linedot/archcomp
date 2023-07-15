find_package(ut REQUIRED)


set(coordinate_storage_tests 
        coordinate_storage/basics
        coordinate_storage/transform)


set(tests cache_info_printall ${coordinate_storage_tests})

foreach(test ${tests})
    set(test_source ${test}.cpp)
    string(REGEX REPLACE "\/" "-" test ${test})
    add_executable(${test} ${test_source})
    target_compile_options(${test} PRIVATE ${project_cxx_flags})
    target_include_directories(${test} PRIVATE ${project_include_dirs})
    target_link_directories(${test} PRIVATE ${project_library_dirs})
    target_link_libraries(${test} PRIVATE ${project_libraries} Boost::ut archcomp)
    add_test(NAME ${test}
        COMMAND ${test})
endforeach()
