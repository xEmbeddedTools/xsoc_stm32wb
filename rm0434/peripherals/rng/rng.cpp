/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// this
#include <rm0434/peripherals/rng/rng.hpp>

// xmcu
#include <rm0434/system/hsem/hsem.hpp>
#include <rm0434/utils/tick_counter.hpp>
#include <rm0434/utils/wait_until.hpp>
#include <soc/st/arm/m4/nvic.hpp>
#include <xmcu/bit.hpp>
#include <xmcu/various.hpp>

// debug
#include <xmcu/assertion.hpp>

namespace {
using namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals;

// sic! Here becouse of 'static inline' gcc bug:
// https://gcc.gnu.org/bugzilla/show_bug.cgi?id=88165
rng::interrupt::Callback callback;
} // namespace

extern "C" {
using namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals;

void RNG_IRQHandler()
{
    RNG_interrupt_handler();
}
}

namespace {
using namespace xmcu;
using namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals;
using namespace xmcu::soc::st::arm::m4::wb::rm0434::utils;

rng::Event_flag get_Event_flag(std::uint32_t a_SR, std::uint32_t a_CR)
{
    rng::Event_flag ret = rng::Event_flag::none;

    if (true == bit::flag::is(a_SR, RNG_SR_SEIS))
    {
        ret |= rng::Event_flag::seed_error;
    }

    if (true == bit::flag::is(a_SR, RNG_SR_CEIS) && false == bit::flag::is(a_CR, RNG_CR_CED))
    {
        ret |= rng::Event_flag::clock_error;
    }

    return ret;
}
} // namespace

namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals {
using namespace xmcu;
using namespace xmcu::soc::st::arm::m4::wb::rm0434::system;

void RNG_interrupt_handler()
{
    std::uint32_t SR = RNG->SR;
    std::uint32_t CR = RNG->CR;

    std::uint32_t data = 0x0;

    if (true == bit::flag::is(SR, RNG_SR_DRDY))
    {
        data = RNG->DR;
    }

    callback.function(data, get_Event_flag(SR, CR), callback.p_user_data);
}

rng::polling::Result rng::polling::get()
{
    Scoped_guard<hsem::_1_step> sem0_guard(0x0u);

    wait_until::all_bits_are_set(RNG->SR, RNG_SR_DRDY);
    return { get_Event_flag(RNG->SR, RNG->CR), RNG->DR };
}

rng::polling::Result rng::polling::get(Milliseconds a_timeout)
{
    const std::uint64_t start = tick_counter<Milliseconds>::get();

    Scoped_guard<hsem::_1_step> sem0_guard(0x0u, a_timeout.get() - (tick_counter<Milliseconds>::get() - start));

    if (false == wait_until::all_bits_are_set(
                     RNG->SR, RNG_SR_DRDY, a_timeout.get() - (tick_counter<Milliseconds>::get() - start)))
    {
        return { get_Event_flag(RNG->SR, RNG->CR), 0x0u };
    }
    return { get_Event_flag(RNG->SR, RNG->CR), RNG->DR };
}

void rng::interrupt::enable(const IRQ_config& a_config)
{
    NVIC_SetPriority(IRQn_Type::RNG_IRQn,
                     NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_config.preempt_priority, a_config.sub_priority));
    NVIC_EnableIRQ(IRQn_Type::RNG_IRQn);
}
void rng::interrupt::disable()
{
    stop();

    NVIC_DisableIRQ(IRQn_Type::RNG_IRQn);
}

void rng::interrupt::start(const Callback& a_callback)
{
    hkm_assert(nullptr != a_callback.function);

    Scoped_guard<nvic> interrupt_guard;

    callback = a_callback;

    bit::flag::set(&(RNG->CR), RNG_CR_IE);
}

void rng::interrupt::stop()
{
    bit::flag::clear(&(RNG->CR), RNG_CR_IE);

    callback = { nullptr, nullptr };
}

void rng::enable()
{
    Scoped_guard<hsem::_1_step> sem0_guard(0x0u);

    bit::flag::set(&(RNG->CR), RNG_CR_CED | RNG_CR_RNGEN, RNG_CR_RNGEN);
}
bool rng::enable(Milliseconds a_timeout)
{
    Scoped_guard<hsem::_1_step> sem0_guard(0x0u, a_timeout);

    if (true == sem0_guard.is_locked())
    {
        bit::flag::set(&(RNG->CR), RNG_CR_CED | RNG_CR_RNGEN, RNG_CR_RNGEN);
        return true;
    }
    return false;
}

void rng::disable()
{
    Scoped_guard<hsem::_1_step> sem0_guard(0x0u);

    bit::flag::clear(&(RNG->CR), RNG_CR_RNGEN);
}
bool rng::disable(Milliseconds a_timeout)
{
    Scoped_guard<hsem::_1_step> sem0_guard(0x0u, a_timeout);

    if (true == sem0_guard.is_locked())
    {
        bit::flag::clear(&(RNG->CR), RNG_CR_RNGEN);

        return true;
    }

    return false;
}
} // namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals

namespace xmcu::soc::st::arm::m4::wb::rm0434 {
using namespace xmcu::soc::st::arm::m4::wb::rm0434::system;
using namespace xmcu::soc::st::arm::m4::wb::rm0434::clocks::sources;

template<> void rcc<rng>::enable<clocks::clk48>(bool a_enable_in_lp)
{
    bit::flag::set(&(RCC->AHB3ENR), RCC_AHB3ENR_RNGEN);
    bit::flag::clear(&(RCC->CCIPR), RCC_CCIPR_RNGSEL);

    if (true == a_enable_in_lp)
    {
        bit::flag::set(&(RCC->AHB3ENR), RCC_AHB3SMENR_RNGSMEN);
    }
    else
    {
        bit::flag::clear(&(RCC->AHB3ENR), RCC_AHB3SMENR_RNGSMEN);
    }
}
template<> void rcc<rng>::enable<lsi>(bool a_enable_in_lp)
{
    bit::flag::set(&(RCC->AHB3ENR), RCC_AHB3ENR_RNGEN);
    bit::flag::set(&(RCC->CCIPR), RCC_CCIPR_RNGSEL, RCC_CCIPR_RNGSEL_0);

    if (true == a_enable_in_lp)
    {
        bit::flag::set(&(RCC->AHB3ENR), RCC_AHB3SMENR_RNGSMEN);
    }
    else
    {
        bit::flag::clear(&(RCC->AHB3ENR), RCC_AHB3SMENR_RNGSMEN);
    }
}
template<> void rcc<rng>::enable<lse>(bool a_enable_in_lp)
{
    bit::flag::set(&(RCC->AHB3ENR), RCC_AHB3ENR_RNGEN);
    bit::flag::set(&(RCC->CCIPR), RCC_CCIPR_RNGSEL, RCC_CCIPR_RNGSEL_1);

    if (true == a_enable_in_lp)
    {
        bit::flag::set(&(RCC->AHB3ENR), RCC_AHB3SMENR_RNGSMEN);
    }
    else
    {
        bit::flag::clear(&(RCC->AHB3ENR), RCC_AHB3SMENR_RNGSMEN);
    }
}
void rcc<rng>::disable()
{
    bit::flag::clear(&(RCC->AHB3ENR), RCC_AHB3ENR_RNGEN);
    bit::flag::clear(&(RCC->CCIPR), RCC_CCIPR_RNGSEL);
    bit::flag::clear(&(RCC->AHB3ENR), RCC_AHB3SMENR_RNGSMEN);
}
} // namespace xmcu::soc::st::arm::m4::wb::rm0434