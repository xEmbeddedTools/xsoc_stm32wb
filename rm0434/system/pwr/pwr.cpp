/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// this
#include <rm0434/system/pwr/pwr.hpp>

// xmcu
#include <rm0434/clocks/sysclk.hpp>
#include <rm0434/peripherals/internal_flash/internal_flash.hpp>
#include <rm0434/rcc.hpp>
#include <rm0434/system/hsem/hsem.hpp>
#include <rm0434/system/mcu/mcu.hpp>
#include <rm0434/utils/wait_until.hpp>
#include <xmcu/bit.hpp>

namespace xmcu::soc::st::arm::m4::wb::rm0434::system {
using namespace xmcu;
using namespace xmcu::soc::st::arm::m4::wb::rm0434::clocks::sources;
using namespace xmcu::soc::st::arm::m4::wb::rm0434::clocks;
using namespace xmcu::soc::st::arm::m4::wb::rm0434::system;
using namespace xmcu::soc::st::arm::m4::wb::rm0434::utils;
using namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals;

void pwr<mcu<1u>>::set_voltage_scaling(Voltage_scaling a_scaling)
{
    bit::flag::set(&(PWR->CR1), PWR_CR1_VOS, static_cast<std::uint32_t>(a_scaling));
    wait_until::all_bits_are_cleared(PWR->SR2, PWR_SR2_VOSF_Pos);
}

bool pwr<mcu<1u>>::set_voltage_scaling(Voltage_scaling a_scaling, xmcu::Milliseconds a_timeout)
{
    bit::flag::set(&(PWR->CR1), PWR_CR1_VOS, static_cast<std::uint32_t>(a_scaling));
    return wait_until::all_bits_are_cleared(PWR->SR2, PWR_SR2_VOSF_Pos, a_timeout);
}

pwr<mcu<1u>>::Voltage_scaling pwr<mcu<1u>>::get_voltage_scaling()
{
    return static_cast<Voltage_scaling>(bit::flag::get(PWR->CR1, PWR_CR1_VOS));
}

void pwr<mcu<2u>>::set_boot(pwr<mcu<2u>>::Boot_after_reset_or_stop a_mode)
{
    bit::flag::set(&(PWR->CR4), PWR_CR4_C2BOOT, static_cast<std::uint32_t>(a_mode));
}

template<> void pwr<mcu<1u>>::stop_mode::enter<hsi16>(Type a_type,
                                                      Method a_method,
                                                      Sleeponexit a_sleeponexit,
                                                      internal_flash::Latency a_desired_flash_latency)
{
    hkm_assert(Method::none != a_method || Sleeponexit::disabled != a_sleeponexit);
    hkm_assert(true == hsi16::is_enabled());

    hsem::_1_step::lock(hsem::Id::rcc);

    if (true == hsem::_1_step::try_lock(hsem::Id::stop_entry))
    {
        if (true == bit::flag::is(PWR->EXTSCR, PWR_EXTSCR_C2DS) || true == bit::flag::is(PWR->EXTSCR, PWR_EXTSCR_C2SBF))
        {
            hsem::_1_step::unlock(hsem::Id::stop_entry);

            sysclk<1u>::set_source<hsi16>();
            internal_flash::set_latency(a_desired_flash_latency);
        }
    }
    else
    {
        sysclk<1u>::set_source<hsi16>();
        internal_flash::set_latency(a_desired_flash_latency);
    }

    hsem::_1_step::unlock(hsem::Id::rcc);

    bit::flag::set(&RCC->CFGR, RCC_CFGR_STOPWUCK);

    PWR->SCR = PWR_SCR_CWUF;
    bit::flag::set(&(PWR->CR1), PWR_CR1_LPMS_Msk, static_cast<std::uint32_t>(a_type));
    bit::flag::set(
        &(SCB->SCR), SCB_SCR_SLEEPONEXIT_Msk, static_cast<std::uint32_t>(a_sleeponexit) | SCB_SCR_SLEEPDEEP_Msk);

    switch (a_method)
    {
        case Method::wfe: {
            __SEV();
            __WFE();
            __WFE();
        }
        break;

        case Method::wfi: {
            __WFI();
        }
        break;

        case Method::none: {
        }
        break;
    }
}

bool pwr<mcu<1u>>::backup_domain::enable(Milliseconds timeout_a)
{
    bit::flag::set(&(PWR->CR1), PWR_CR1_DBP);
    return wait_until::all_bits_are_cleared(PWR->CR1, PWR_CR1_DBP, timeout_a);
}

bool pwr<mcu<1u>>::backup_domain::disable(Milliseconds timeout_a)
{
    bit::flag::clear(&(PWR->CR1), PWR_CR1_DBP);
    return wait_until::all_bits_are_set(PWR->CR1, PWR_CR1_DBP, timeout_a);
}
} // namespace xmcu::soc::st::arm::m4::wb::rm0434::system