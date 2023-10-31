cmake_minimum_required(VERSION 3.2)
project(UserCode_lib CXX)

set(LIBRARY_NAME UserCode_lib)

file(GLOB_RECURSE LIB_SOURCES
	"source/*.cpp"
)

add_library(${LIBRARY_NAME} SHARED
	${LIB_SOURCES}
)

include_directories(
    ${PROJECT_SOURCE_DIR}/source
)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -Wno-write-strings)

set_target_properties( ${LIBRARY_NAME} PROPERTIES OUTPUT_NAME_DEBUG ${LIBRARY_NAME}D )
set_target_properties( ${LIBRARY_NAME} PROPERTIES OUTPUT_NAME_RELEASE ${LIBRARY_NAME})
