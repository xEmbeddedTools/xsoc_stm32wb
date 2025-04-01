#
# Copyright (c) xEmbeddedTools team and contributors.
# Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
#

cmake_minimum_required(VERSION 3.19.0)

if (XMCU_SOC_MODEL STREQUAL "XMCU_SOC_MODEL_STM32WB35CEU6A")
    file(GLOB_RECURSE XMCU_SOC_SRC 
            "${CMAKE_CURRENT_LIST_DIR}/rm0434/*.cpp" 
            "${CMAKE_CURRENT_LIST_DIR}/rm0434/*.c" 
            "${CMAKE_CURRENT_LIST_DIR}/rm0434/*.asm" 
            "${CMAKE_CURRENT_LIST_DIR}/rm0434/*.hpp"
            "${CMAKE_CURRENT_LIST_DIR}/rm0434/*.h"
            "${CMAKE_CURRENT_LIST_DIR}/CMSIS/Device/ST/STM32WBxx/Source/Templates/gcc/startup_stm32wb35xx_cm4.s"
            "${CMAKE_CURRENT_LIST_DIR}/CMSIS/Device/ST/STM32WBxx/Source/Templates/system_stm32wbxx.c")
    set(XMCU_LD_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/CMSIS/Device/ST/STM32WBxx/Source/Templates/gcc/linker/stm32wb35xx_flash_cm4.ld")
    add_library(XMCU_SOC OBJECT ${XMCU_SOC_SRC})
    add_compile_definitions(XMCU_SOC PUBLIC "XMCU_SOC_MODEL_STM32WB35CEU6A" "STM32WB35xx")
endif()

set(XMCU_COMPILE_FLAGS ${XMCU_COMPILE_FLAGS} -mcpu=cortex-m4 -mthumb)
set(XMCU_LINKER_FLAGS ${XMCU_LINKER_FLAGS} -mcpu=cortex-m4 -mthumb)

if (${XMCU_NOSTDLIB} STREQUAL "ON")
    message("XMCU: no stdlib")
    set(XMCU_COMPILE_FLAGS ${XMCU_COMPILE_FLAGS} -nostdlib -nostartfiles)
    set(XMCU_LINKER_FLAGS ${XMCU_LINKER_FLAGS} -nostdlib -nostartfiles --specs=nosys.specs -lc_nano)
endif()

include_directories(XMCU_SOC PUBLIC
                    ${XMCU_PATH}
                    ${CMAKE_CURRENT_LIST_DIR}
                    "${XMCU_PATH}/soc/st/arm/CMSIS/Include"
                    "${CMAKE_CURRENT_LIST_DIR}/CMSIS/Device/ST/STM32WBxx/Include")
