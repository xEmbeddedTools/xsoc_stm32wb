/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// this
#include "stm32wb55xx.h"
#include <rm0434/peripherals/AES/AES.hpp>

// xmcu
#include <xmcu/bit.hpp>



namespace {

using namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals;

void enable(AES_TypeDef* a_p_registers)
{
    bit::flag::clear(&(a_p_registers->CR), AES_CR_EN_Pos);
        // a_p_registers->CR
}

void disable(AES_TypeDef* a_p_registers)
{

}

} // namespace

namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals {

using namespace utils;
using namespace xmcu;

void AES::enable()
{
    ::enable(this->p_registers);
}

void AES::disable()
{
    ::disable(this->p_registers);
}


} // namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals

namespace xmcu::soc::st::arm::m4::wb::rm0434 {

void rcc<AES, AES::_1>::enable()
{
    bit::flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_AES1EN);
    bit::flag::is(RCC->AHB2ENR, RCC_AHB2ENR_AES1EN);
}

void rcc<AES, AES::_2>::disable()
{
    bit::flag::set(&(RCC->AHB3ENR), RCC_AHB3ENR_AES2EN);
    bit::flag::is(RCC->AHB3ENR, RCC_AHB3ENR_AES2EN);
}

} // namespace xmcu::soc::st::arm::m4::wb::rm0434


