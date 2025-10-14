#
# Copyright (c) xEmbeddedTools team and contributors.
# Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
#

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

target_compile_features(XMCU_SOC PRIVATE cxx_std_23)
target_compile_features(XMCU_SOC PRIVATE c_std_11)
target_include_directories(XMCU_SOC PUBLIC
                                    ${XMCU_PATH}
                                    ${CMAKE_CURRENT_LIST_DIR}
                                    "${CMAKE_CURRENT_LIST_DIR}/CMSIS/Device/ST/STM32WBxx/Include/"
                                    "${XMCU_PATH}/soc/st/arm/CMSIS/Include")

if (DEFINED XMCU_NOSTDLIB)
    if (${XMCU_NOSTDLIB} STREQUAL "ON")
        target_compile_definitions(XMCU_SOC PUBLIC XMCU_NOSTDLIB)
    endif()
    message(STATUS "XMCU_NOSTDLIB: ${XMCU_NOSTDLIB}")
else()
    message(FATAL_ERROR "XMCU_NOSTDLIB is not set (expected values: ON/OFF)")
endif()

include_directories(
    ${XMCU_PATH}
    ${CMAKE_CURRENT_LIST_DIR}
    "${XMCU_PATH}/soc/st/arm/CMSIS/Include"
    "${CMAKE_CURRENT_LIST_DIR}/CMSIS/Device/ST/STM32WBxx/Include")

