cmake_minimum_required(VERSION 3.0)
project(lsm9ds1 NONE)

set(LOCAL_SRCS "")
set(LOCAL_HDRS "")

add_definitions(-DUSE_HAL_DRIVER)
add_definitions(-DDEBUG)
add_definitions(-DSTM32F407xx)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m4 -std=gnu11 -g3 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb")

file(GLOB LOCAL_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/*.c")

file(GLOB LOCAL_HDRS "${CMAKE_CURRENT_SOURCE_DIR}/../inc/*.h")

include_directories(${CMAKE_CURRENT_SOURCE_DIR} "../../CMSIS/Device/ST/STM32F4xx/Include")
include_directories(${CMAKE_CURRENT_SOURCE_DIR} "../../CMSIS/Include")
include_directories(${CMAKE_CURRENT_SOURCE_DIR} "../../STM32F4xx_HAL_Driver/Inc")
include_directories(${CMAKE_CURRENT_SOURCE_DIR} "../inc")
include_directories(${CMAKE_CURRENT_SOURCE_DIR} "../../../Inc")

add_library(lsm9ds1 ${LOCAL_SRCS} ${LOCAL_HDRS})

