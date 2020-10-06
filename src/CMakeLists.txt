add_executable(freebot-realtime freebot-realtime.cpp)
target_link_libraries(freebot-realtime rt ${RBDL_LIBRARY} ${RBDL_URDFReader_LIBRARY} yaml-cpp)
target_include_directories(freebot-realtime PUBLIC ../include ${RBDL_PATH}/include ${EIGEN3_INCLUDE_DIR})
