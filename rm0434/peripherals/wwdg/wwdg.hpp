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
    struct s : xmcu::non_constructible
    {
        static constexpr uint32_t min = 0x40;
        static constexpr uint32_t max = 0x7F;
    };
    using Window = xmcu::Limited<uint32_t, s::min, s::max>;
    using Interrupt_Callback_Function = void (*)();
    static Interrupt_Callback_Function p_callback;

    static void enable(Window a_window = s::max);

    static void feed();
    static void feed(Window a_reload);
    static bool is_active();

    static void interrupt_enable(const xmcu::hal::IRQ_config& a_config, Interrupt_Callback_Function a_p_callback);
    static void interrupt_disable();
    static void feed_int();
    static void feed_int(const Window& a_reload);
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
