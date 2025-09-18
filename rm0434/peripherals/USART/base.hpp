#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// soc
#include <rm0434/config.hpp>

// xmcu
#include <xmcu/non_constructible.hpp>

namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals::ll {
#if defined(XMCU_SOC_MODEL_STM32WB35CEU6A) || defined(XMCU_SOC_MODEL_STM32WB55CGU6)
struct usart_base : private non_constructible
{
    enum class _1 : std::uint32_t;
};
#endif
} // namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals::ll
