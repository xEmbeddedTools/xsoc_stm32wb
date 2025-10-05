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
    add_library(XMCU_SOC OBJECT ${XMCU_SOC_SRC})
    target_compile_definitions(XMCU_SOC PUBLIC "XMCU_SOC_MODEL_STM32WB35CEU6A" "STM32WB35xx")
elseif (XMCU_SOC_MODEL STREQUAL "XMCU_SOC_MODEL_STM32WB55CGU6")
    file(GLOB_RECURSE XMCU_SOC_SRC
                      "${CMAKE_CURRENT_LIST_DIR}/rm0434/*.cpp"
                      "${CMAKE_CURRENT_LIST_DIR}/rm0434/*.c"
                      "${CMAKE_CURRENT_LIST_DIR}/rm0434/*.asm"
                      "${CMAKE_CURRENT_LIST_DIR}/rm0434/*.hpp"
                      "${CMAKE_CURRENT_LIST_DIR}/rm0434/*.h"
                      "${CMAKE_CURRENT_LIST_DIR}/CMSIS/Device/ST/STM32WBxx/Source/Templates/gcc/startup_stm32wb55xx_cm4.s"
                      "${CMAKE_CURRENT_LIST_DIR}/CMSIS/Device/ST/STM32WBxx/Source/Templates/system_stm32wbxx.c")
    add_library(XMCU_SOC OBJECT ${XMCU_SOC_SRC})
    target_compile_definitions(XMCU_SOC PUBLIC "XMCU_SOC_MODEL_STM32WB55CGU6" "STM32WB55xx")
else()
    message(FATAL_ERROR "Unsupported XMCU_SOC_MODEL: ${XMCU_SOC_MODEL}")
endif()

target_compile_features(XMCU_SOC PUBLIC cxx_std_23)
target_compile_features(XMCU_SOC PUBLIC c_std_11)
target_include_directories(XMCU_SOC PUBLIC
                                    ${XMCU_PATH}
                                    ${CMAKE_CURRENT_LIST_DIR}
                                    "${CMAKE_CURRENT_LIST_DIR}/CMSIS/Device/ST/STM32WBxx/Include/"
                                    "${XMCU_PATH}/soc/st/arm/CMSIS/Include")

target_compile_options(XMCU_SOC PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-mcpu=cortex-m4 -mthumb>
                                       $<$<COMPILE_LANGUAGE:C>:-mcpu=cortex-m4 -mthumb>
                                       $<$<COMPILE_LANGUAGE:ASM>:-mcpu=cortex-m4 -mthumb>)

if (${XMCU_NOSTDLIB} STREQUAL "ON")
    message("XMCU: no stdlib")
    target_compile_options(XMCU_SOC PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-nostdlib -nostartfiles>
                                           $<$<COMPILE_LANGUAGE:C>:-nostdlib -nostartfiles>
                                           $<$<COMPILE_LANGUAGE:ASM>:-nostdlib -nostartfiles>)
endif()

include_directories(
    ${XMCU_PATH}
    ${CMAKE_CURRENT_LIST_DIR}
    "${XMCU_PATH}/soc/st/arm/CMSIS/Include"
    "${CMAKE_CURRENT_LIST_DIR}/CMSIS/Device/ST/STM32WBxx/Include")

