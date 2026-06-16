/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// this
#include <rm0434/utils/tick_counter.hpp>

// soc
#include <rm0434/clocks/hclk.hpp>

// debug
#include <xmcu/assertion.hpp>

namespace xmcu::soc::st::arm::m4::wb::rm0434::utils {
using namespace xmcu::soc::st::arm::m4::wb::rm0434::clocks;
using namespace xmcu::soc::st::arm::m4::wb::rm0434::system;

tick_counter<Milliseconds>::Callback tick_counter<Milliseconds>::callback;
std::uint16_t auto_reload = 0x0u;

std::uint64_t tick_counter<Milliseconds>::get()
{
    // Last / most significant word. These lines do not emit any instructions.
    volatile std::uint32_t& cnt_lsw = reinterpret_cast<volatile std::uint32_t*>(&cnt)[0];
    volatile std::uint32_t& cnt_msw = reinterpret_cast<volatile std::uint32_t*>(&cnt)[1];

    for (std::uint32_t i = 0; true; ++i)
    {
        auto m1 = cnt_msw;
        auto l1 = cnt_lsw;
        auto m2 = cnt_msw;
        if (m1 == m2)
        {
            // Usually just performs the return. The returned value is already prepared during assignment.
            return std::uint64_t { m1 } << 32 | l1;
        }
        if (i > 3) // non-standard break condition to optimize the fast path of the loop
        {
            break;
        }
    }
    return 0; // what can I do?
}

void tick_counter<Milliseconds>::register_callback(const Callback& a_callback)
{
    callback = a_callback;
}

void tick_counter<Milliseconds>::update(void*)
{
    cnt = cnt + 1u;

    if (nullptr != callback.function)
    {
        callback.function(cnt, callback.p_user_data);
    }
}

template<> void tick_counter<Milliseconds>::enable<Systick>(Systick* a_p_timer,
                                                            const IRQ_config& a_irq_config,
                                                            std::uint64_t a_start_cnt)
{
    a_p_timer->enable((hclk<1u>::get_frequency_Hz() / 1000u) - 1, Systick::Prescaler::_1);
    a_p_timer->interrupt.enable(a_irq_config);
    a_p_timer->interrupt.register_callback({ tick_counter::update, nullptr });
    p_timer = a_p_timer;
    cnt = a_start_cnt;
}

template<> void tick_counter<Milliseconds>::disable<Systick>()
{
    (reinterpret_cast<Systick*>(p_timer))->disable();
    p_timer = nullptr;
    cnt = 0u;
}

template<> void tick_counter<Milliseconds>::start<Systick>(bool a_call_handler_on_start)
{
    reinterpret_cast<Systick*>(p_timer)->start();

    if (true == a_call_handler_on_start && nullptr != callback.function)
    {
        callback.function(cnt, callback.p_user_data);
    }
}

template<> void tick_counter<Milliseconds>::stop<Systick>()
{
    reinterpret_cast<Systick*>(p_timer)->stop();
}

template<> void tick_counter<Milliseconds>::update_and_resume<Systick>()
{
    hkm_assert(nullptr != p_timer);
    auto systick = reinterpret_cast<Systick*>(p_timer);
    systick->update_and_reload_value(hclk<1u>::get_frequency_Hz() / 1000u - 1);
}
} // namespace xmcu::soc::st::arm::m4::wb::rm0434::utils
