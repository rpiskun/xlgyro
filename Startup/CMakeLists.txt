cmake_minimum_required(VERSION 3.0)
project(startup ASM)

set(LOCAL_SRCS "")
set(LOCAL_HDRS "")

add_definitions(-DUSE_HAL_DRIVER)
add_definitions(-DDEBUG)
add_definitions(-DSTM32F407xx)

set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -x assembler-with-cpp")

file(GLOB LOCAL_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/*.s")

add_library(startup ${LOCAL_SRCS})

