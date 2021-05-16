cmake_minimum_required(VERSION 3.10)
project(linchpin_python)


add_library(rlew32 MODULE rlew32.c)
target_compile_options(rlew32 PRIVATE -Wall -Wextra)
target_include_directories(
	rlew32 PRIVATE
	../inc)
#set_target_properties(tardy_gnutls_connection_provider PROPERTIES OUTPUT_NAME gnutls)
set_target_properties(rlew32 PROPERTIES PREFIX "")

#add_executable(rlew32_dec rlew32_dec.c)
#target_include_directories(rlew32_dec PRIVATE ../inc)
