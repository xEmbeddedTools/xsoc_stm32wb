#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// CMSIS
#include <stm32wbxx.h>

// soc
#include <rm0434/peripherals/USART/base.hpp>

// xmcu
#include <xmcu/Limited.hpp>
#include <xmcu/Non_copyable.hpp>
#include <xmcu/non_constructible.hpp>

/**
  * @brief Universal Synchronous Asynchronous Receiver Transmitter
  */
//typedef struct
//{
//    __IO uint32_t CR1;   /*!< USART Control register 1,                 Address offset: 0x00 */
//    __IO uint32_t CR2;   /*!< USART Control register 2,                 Address offset: 0x04 */
//    __IO uint32_t CR3;   /*!< USART Control register 3,                 Address offset: 0x08 */
//    __IO uint32_t BRR;   /*!< USART Baud rate register,                 Address offset: 0x0C */
//    __IO uint32_t GTPR;  /*!< USART Guard time and prescaler register,  Address offset: 0x10 */
//    __IO uint32_t RTOR;  /*!< USART Receiver Time Out register,         Address offset: 0x14 */
//    __IO uint32_t RQR;   /*!< USART Request register,                   Address offset: 0x18 */
//    __IO uint32_t ISR;   /*!< USART Interrupt and status register,      Address offset: 0x1C */
//    __IO uint32_t ICR;   /*!< USART Interrupt flag Clear register,      Address offset: 0x20 */
//    __IO uint32_t RDR;   /*!< USART Receive Data register,              Address offset: 0x24 */
//    __IO uint32_t TDR;   /*!< USART Transmit Data register,             Address offset: 0x28 */
//    __IO uint32_t PRESC; /*!< USART Prescaler register,                 Address offset: 0x2C */
//} USART_TypeDef;


namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals::ll {
struct usart : public usart_base
{

};
} // namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals::ll
