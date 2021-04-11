cmake_minimum_required(VERSION 3.13)
project(linchpin)
set(CMAKE_CXX_STANDARD 14)


# https://cliutils.gitlab.io/modern-cmake/chapters/testing/googletest.html
include(FetchContent)
include(GoogleTest)

FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        release-1.8.0
)

FetchContent_GetProperties(googletest)
if(NOT googletest_POPULATED)
  FetchContent_Populate(googletest)
  add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR})
endif()

# message("googletest_SOURCE_DIR: ${googletest_SOURCE_DIR}")
# message("googletest_BINARY_DIR: ${googletest_BINARY_DIR}")

enable_testing()

set(SOURCES
	src/test_base64.cpp
	src/test_proto.cpp
	src/test_rle.cpp
	src/test_bitstream.cpp
	../src/linchpin_state.c
	../src/usnprintf.c
	../src/linchpin_error.c
)

add_executable(linchpin-tests ${SOURCES})
target_compile_options(linchpin-tests PRIVATE -Wall -Wextra)
target_include_directories(
	linchpin-tests PRIVATE
	${googletest_SOURCE_DIR}/googletest/include
	inc
	../inc)
target_link_libraries(linchpin-tests gtest_main gmock)

gtest_add_tests(TARGET linchpin-tests)
