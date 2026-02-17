/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// this
#include <rm0434/peripherals/AES/AES.hpp>

namespace {

using namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals;

} // namespace


namespace xmcu::soc::st::arm::m4::wb::rm0434 {

void rcc<AES>::enable()
{
    bit::flag::set(&(RCC->AHB3ENR), RCC_AHB3ENR_PKAEN);
    bit::flag::is(RCC->AHB3ENR, RCC_AHB3ENR_PKAEN);
}

void rcc<AES>::disable()
{
    bit::flag::clear(&(RCC->AHB3ENR), RCC_AHB3ENR_PKAEN);
    bit::flag::is(RCC->AHB3ENR, RCC_AHB3ENR_PKAEN);
}

} // namespace xmcu::soc::st::arm::m4::wb::rm0434


