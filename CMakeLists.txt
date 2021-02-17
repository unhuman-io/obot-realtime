cmake_minimum_required(VERSION 3.0)
project(freebot-realtime)

set(RBDL_PATH CACHE DIRECTORY /usr)
FIND_LIBRARY (RBDL_LIBRARY NAMES rbdl
	PATHS
	${RBDL_PATH}
    ${RBDL_PATH}/lib)
FIND_LIBRARY (RBDL_URDFReader_LIBRARY NAMES rbdl_urdfreader
	PATHS
	${RBDL_PATH}
	${RBDL_PATH}/lib
	)

FIND_PACKAGE (Eigen3 REQUIRED)

add_subdirectory(src)