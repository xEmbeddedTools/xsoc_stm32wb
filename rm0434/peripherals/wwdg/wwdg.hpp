#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// std
#include <stdint.h>

// CMSIS
#include <stm32wbxx.h>

// xmcu
#include <rm0434/rcc.hpp>
#include <xmcu/hal/IRQ_config.hpp>
#include <xmcu/Limited.hpp>
#include <xmcu/non_constructible.hpp>

namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals {
class wwdg : private xmcu::non_constructible
{
public:
    static constexpr uint32_t window_min = 0x40;
    static constexpr uint32_t window_max = 0x7F;
    using window_t = xmcu::Limited<uint32_t, window_min, window_max>;

    static void enable(window_t a_window = window_max);

    static void feed();
    static void feed(window_t reload);
    static bool is_active();

protected:
    static void interrupt_enable(const xmcu::hal::IRQ_config& a_config);
    static void feed_int();
};

} // namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals

namespace xmcu::soc::st::arm::m4::wb::rm0434 {
template<> class rcc<peripherals::wwdg> : private non_constructible
{
public:
    static void enable(bool a_enable_in_lp)
    {
        bit::flag::set(&RCC->APB1ENR1, RCC_APB1ENR1_WWDGEN);

        if (true == a_enable_in_lp)
        {
            bit::flag::set(&RCC->APB1SMENR1, RCC_APB1SMENR1_WWDGSMEN);
        }
        else
        {
            bit::flag::clear(&RCC->APB1SMENR1, RCC_APB1SMENR1_WWDGSMEN);
        }
    }
    static void disable()
    {
        bit::flag::clear(&RCC->APB1ENR1, RCC_APB1ENR1_WWDGEN);
    }
};

} // namespace xmcu::soc::st::arm::m4::wb::rm0434
