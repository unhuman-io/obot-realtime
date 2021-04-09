#add_test(NAME ik_analytic COMMAND test)

function(freebot_test name)
    add_executable(
        ${name}_test
        ${name}_test.cpp
        ../src/${name}.cpp
    )
    target_link_libraries(
        ${name}_test
        gtest_main
    )
    target_include_directories(${name}_test PUBLIC ../include ${RBDL_PATH}/include ${EIGEN3_INCLUDE_DIR})
    gtest_discover_tests(${name}_test)
endfunction()

include(GoogleTest)
freebot_test(freebot_arm_ik_analytic_control)