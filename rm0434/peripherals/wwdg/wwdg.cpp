/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// this
#include <rm0434/peripherals/wwdg/wwdg.hpp>

// using namespace xmcu::soc::st::arm::m4::wb::rm0434;
using namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals;

void wwdg::enable(wwdg::window_t a_window)
{
    WWDG->CFR = WWDG_CFR_WDGTB | a_window;
    WWDG->CR = WWDG_CR_WDGA | WWDG_CR_T;
}

void wwdg::feed()
{
    static constexpr uint32_t reload = WWDG_CR_T;
    static_assert(reload <= window_max && reload >= window_min, "invalid reload value");
    WWDG->CR = WWDG_CR_WDGA | reload;
}

void wwdg::feed(wwdg::window_t a_reload)
{
    WWDG->CR = a_reload | WWDG_CR_WDGA;
}

bool wwdg::is_active()
{
    // xmcu::bit::is(WWDG->CR, WWDG_CR_WDGA_Pos);
    return xmcu::bit::is_any(WWDG->CR, WWDG_CR_WDGA);
}

void wwdg::interrupt_enable(const xmcu::hal::IRQ_config& a_config)
{
    bit::flag::set(&WWDG->CFR, WWDG_CFR_EWI);

    NVIC_SetPriority(WWDG_IRQn,
                     NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_config.preempt_priority, a_config.sub_priority));
    NVIC_EnableIRQ(WWDG_IRQn);
}

void wwdg::feed_int()
{
    // This write operation must be performed a few instructions before the end of the ISR.
    // If it is the final instruction before exiting, it could cause an immediate re-entry into this same IRQ.
    WWDG->SR = 0; // clear interrupt flag
    wwdg::feed();
}
