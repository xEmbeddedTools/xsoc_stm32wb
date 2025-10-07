/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// this
#include <rm0434/peripherals/wwdg/wwdg.hpp>

// using namespace xmcu::soc::st::arm::m4::wb::rm0434;
using namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals;

volatile uint32_t wwdg_repetable::wwdg_mid_counter = wwdg_mid_reload;

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

void wwdg_repetable::interrupt_enable(const xmcu::hal::IRQ_config& a_config)
{
    bit::flag::set(&WWDG->CFR, WWDG_CFR_EWI);

    NVIC_SetPriority(WWDG_IRQn,
                     NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_config.preempt_priority, a_config.sub_priority));
    NVIC_EnableIRQ(WWDG_IRQn);
}

void wwdg_repetable::feed()
{
    wwdg_mid_counter = wwdg_mid_reload;
}

void wwdg_repetable::feed_hw()
{
    // This write operation must be performed a few instructions before the end of the ISR.
    // If it is the final instruction before exiting, it causes an immediate re-entry into this same IRQ.
    WWDG->SR = 0; // clear interrupt flag
    wwdg::feed();
}

void wwdg_repetable::interrupt()
{
    if (0 < wwdg_mid_counter)
    {
        // threatsafe by design
        // ... until main thread in only writing (not read modify write) and only interrupt make decrementation.
        feed_hw();
        if constexpr (1 == wwdg_repetable::wwdg_mid_step)
        {
            wwdg_repetable::wwdg_mid_counter = wwdg_repetable::wwdg_mid_counter - 1;
        }
        else
        {
            if (wwdg_repetable::wwdg_mid_counter > wwdg_repetable::wwdg_mid_step)
            {
                wwdg_repetable::wwdg_mid_counter = wwdg_repetable::wwdg_mid_counter - wwdg_repetable::wwdg_mid_step;
            }
            else
            {
                wwdg_repetable::wwdg_mid_counter = 0;
            }
        }
        // reset is caused only if counter is set 0 and until next interrupt anyone don't set a greather value.
        // altenatively when this interrupt cannot be handled.
    }

    // TODO don't put interrupt flag clearing at lest
}

extern "C" {
void WWDG_IRQHandler()
{
    // TODO try to make some magic to disable unussed code?
    return wwdg_repetable::interrupt();
}
}
