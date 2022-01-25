add_executable(obot-realtime obot-realtime.cpp obot_arm_ik_control.cpp obot_arm_ik_analytic_control.cpp obot_base_control.cpp)
target_link_libraries(obot-realtime rt ${RBDL_LIBRARY} ${RBDL_URDFReader_LIBRARY} yaml-cpp motor_manager)
target_include_directories(obot-realtime PUBLIC ../include ${RBDL_PATH}/include ${EIGEN3_INCLUDE_DIR})

add_executable(obot-test obot-test.cpp)
target_link_libraries(obot-test rt yaml-cpp motor_manager)
target_include_directories(obot-test PUBLIC ../include)
