foreach(test cache_info_printall coordinate_storage_test)
    add_executable(${test} ${test}.cpp)
    target_include_directories(${test} PRIVATE ${project_include_dirs})
    target_link_directories(${test} PRIVATE ${project_library_dirs})
    target_link_libraries(${test} PRIVATE ${project_libraries})
    add_test(NAME ${test}
        COMMAND ${test})
endforeach()
