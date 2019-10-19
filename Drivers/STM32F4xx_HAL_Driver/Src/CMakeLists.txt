cmake_minimum_required(VERSION 3.0)
project(stm32f407hal C)

set(LOCAL_SRCS "")
set(LOCAL_HDRS "")

set(CMAKE_C_COMPILER "$ENV{CROSS_COMPILE}gcc") 

add_definitions(-DUSE_HAL_DRIVER)
add_definitions(-DDEBUG)
add_definitions(-DSTM32F407xx)

set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/STM32F407VGTX_FLASH.ld")

set_target_properties(${TARGET_NAME} PROPERTIES LINK_DEPENDS ${LINKER_SCRIPT})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m4 -std=gnu11 -g3 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o")

file(GLOB LOCAL_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/*.c")
file(GLOB LOCAL_HDRS "${CMAKE_CURRENT_SOURCE_DIR}/../Inc/*.h")

include_directories(${CMAKE_SOURCE_DIR} "Drivers/CMSIS/Device/ST/STM32F4xx/Include")
include_directories(${CMAKE_SOURCE_DIR} "Drivers/CMSIS/Include")
include_directories(${CMAKE_CURRENT_SOURCE_DIR} "../Inc")

add_library(stm32f407hal ${LOCAL_SRCS} ${LOCAL_HDRS})

