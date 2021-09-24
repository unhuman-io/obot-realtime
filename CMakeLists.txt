cmake_minimum_required(VERSION 3.10)
project(freebot-realtime)
enable_testing()

set(CMAKE_CXX_STANDARD 11)
option(ENABLE_GTEST "enable gtest" ON)

if(${CMAKE_VERSION} VERSION_LESS "3.14.0")
	message("cmake version to low for gtest")
	set(ENABLE_GTEST OFF)
endif()

if(ENABLE_GTEST)
	include(FetchContent)
	FetchContent_Declare(
	googletest
	URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
	)
	FetchContent_MakeAvailable(googletest)
endif()

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

if(ENABLE_GTEST)
	add_subdirectory(test)
endif()