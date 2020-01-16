cmake_minimum_required(VERSION 3.0)
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")
project(xlgyro.elf NONE)

set(LOCAL_SRCS "")
set(LOCAL_HDRS "")

add_definitions(-DUSE_HAL_DRIVER)
add_definitions(-DDEBUG)
add_definitions(-DSTM32F407xx)

set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/../STM32F407VGTX_FLASH.ld")
#set(LINKER_SCRIPT "/home/piskun/projects/glstk/STM32CubeIDE/workspace/xlgyro/STM32F407VGTX_FLASH.ld")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m4 -std=gnu11 -g3 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb")

file(GLOB LOCAL_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/*.c")

file(GLOB LOCAL_HDRS "${CMAKE_CURRENT_SOURCE_DIR}/../Inc/*.h")

include_directories(${CMAKE_CURRENT_SOURCE_DIR} "../Drivers/CMSIS/Device/ST/STM32F4xx/Include")
include_directories(${CMAKE_CURRENT_SOURCE_DIR} "../Drivers/CMSIS/Include")
include_directories(${CMAKE_CURRENT_SOURCE_DIR} "../Drivers/STM32F4xx_HAL_Driver/Inc")
include_directories(${CMAKE_CURRENT_SOURCE_DIR} "../Drivers/LSM9DS1/inc")
include_directories(${CMAKE_CURRENT_SOURCE_DIR} "../Inc")

set (CMAKE_EXE_LINKER_FLAGS "-T${LINKER_SCRIPT} -Wl,-Map=${PROJECT_NAME}.map -Wl,--gc-sections -Wl,--start-group -Wl,-lc -Wl,-lm -Wl,--end-group")

add_executable(${PROJECT_NAME} ${LOCAL_SRCS} ${LOCAL_HDRS})

target_link_libraries(${PROJECT_NAME} lsm9ds1 stm32f407hal startup)

