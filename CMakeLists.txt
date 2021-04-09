cmake_minimum_required(VERSION 3.14)
project(freebot-realtime)
enable_testing()

set(CMAKE_CXX_STANDARD 11)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
)
FetchContent_MakeAvailable(googletest)

set(RBDL_PATH CACHE PATH /usr)
FIND_LIBRARY (RBDL_LIBRARY NAMES rbdl
	PATHS
	${RBDL_PATH}/lib)
FIND_LIBRARY (RBDL_URDFReader_LIBRARY NAMES rbdl_urdfreader
	PATHS
	${RBDL_PATH}/lib
	)

FIND_PACKAGE (Eigen3 REQUIRED)

add_subdirectory(src)
add_subdirectory(test)