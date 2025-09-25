#
# Copyright (c) xEmbeddedTools team and contributors.
# Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
#

cmake_minimum_required(VERSION 3.19.0)

set(SOC_MODEL_FAMILY "")
set(STARTUP_FILE "")

if (XMCU_SOC_MODEL STREQUAL "XMCU_SOC_MODEL_STM32WB35CEU6A")
    set(SOC_MODEL_FAMILY "STM32WB35xx")
    set(STARTUP_FILE "${CMAKE_CURRENT_LIST_DIR}/CMSIS/Device/ST/STM32WBxx/Source/Templates/gcc/startup_stm32wb35xx_cm4.s")
    set(XMCU_LD_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/CMSIS/Device/ST/STM32WBxx/Source/Templates/gcc/linker/stm32wb35xx_flash_cm4.ld")
elseif (XMCU_SOC_MODEL STREQUAL "XMCU_SOC_MODEL_STM32WB55CGU6")
    set(SOC_MODEL_FAMILY "STM32WB55xx")
    set(STARTUP_FILE "${CMAKE_CURRENT_LIST_DIR}/CMSIS/Device/ST/STM32WBxx/Source/Templates/gcc/startup_stm32wb55xx_cm4.s")
    set(XMCU_LD_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/CMSIS/Device/ST/STM32WBxx/Source/Templates/gcc/linker/stm32wb55xx_flash_cm4.ld")
else()
    message(FATAL_ERROR "Unsupported XMCU_SOC_MODEL: ${XMCU_SOC_MODEL}")
endif()

file(GLOB_RECURSE XMCU_SOC_SRC
    "${CMAKE_CURRENT_LIST_DIR}/rm0434/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/rm0434/*.c"
    "${CMAKE_CURRENT_LIST_DIR}/rm0434/*.asm"
    "${CMAKE_CURRENT_LIST_DIR}/rm0434/*.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/rm0434/*.h"
    "${STARTUP_FILE}"
    "${CMAKE_CURRENT_LIST_DIR}/CMSIS/Device/ST/STM32WBxx/Source/Templates/system_stm32wbxx.c")

add_library(XMCU_SOC OBJECT ${XMCU_SOC_SRC})

add_compile_definitions("${XMCU_SOC_MODEL}" "${SOC_MODEL_FAMILY}")

set(XMCU_COMPILE_FLAGS ${XMCU_COMPILE_FLAGS} -mcpu=cortex-m4 -mthumb)
set(XMCU_LINKER_FLAGS ${XMCU_LINKER_FLAGS} -mcpu=cortex-m4 -mthumb)

if (${XMCU_NOSTDLIB} STREQUAL "ON")
    message("XMCU: no stdlib")
    set(XMCU_COMPILE_FLAGS ${XMCU_COMPILE_FLAGS} -nostdlib -nostartfiles)
    set(XMCU_LINKER_FLAGS ${XMCU_LINKER_FLAGS} -nostdlib -nostartfiles --specs=nosys.specs -lc_nano)
endif()

include_directories(
    ${XMCU_PATH}
    ${CMAKE_CURRENT_LIST_DIR}
    "${XMCU_PATH}/soc/st/arm/CMSIS/Include"
    "${CMAKE_CURRENT_LIST_DIR}/CMSIS/Device/ST/STM32WBxx/Include")

