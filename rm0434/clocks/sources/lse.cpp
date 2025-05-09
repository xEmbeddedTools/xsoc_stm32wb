/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// this
#include <rm0434/clocks/sources/lse.hpp>

// xmcu
#include <rm0434/utils/tick_counter.hpp>
#include <rm0434/utils/wait_until.hpp>

namespace xmcu::soc::st::arm::m4::wb::rm0434::clocks::sources {
using namespace xmcu;
using namespace xmcu::soc::st::arm::m4::wb::rm0434::utils;

void lse::xtal::enable(Drive a_drive_config)
{
    bit::flag::set(&(PWR->CR1), PWR_CR1_DBP);
    wait_until::all_bits_are_set(PWR->CR1, PWR_CR1_DBP);
    bit::flag::set(&(RCC->BDCR), RCC_BDCR_LSEDRV, static_cast<std::uint32_t>(a_drive_config));

    bit::flag::set(&(RCC->BDCR), RCC_BDCR_LSEON);
    wait_until::all_bits_are_set(RCC->BDCR, RCC_BDCR_LSERDY);
}
bool lse::xtal::enable(Drive a_drive_config, Milliseconds a_timeout)
{
    const std::uint64_t start = tick_counter<Milliseconds>::get();

    bit::flag::set(&(PWR->CR1), PWR_CR1_DBP);
    if (true == wait_until::all_bits_are_set(
                    PWR->CR1, PWR_CR1_DBP, a_timeout.get() - (tick_counter<Milliseconds>::get() - start)))
    {
        bit::flag::set(&(RCC->BDCR), RCC_BDCR_LSEDRV, static_cast<std::uint32_t>(a_drive_config));

        bit::flag::set(&(RCC->BDCR), RCC_BDCR_LSEON);
        return wait_until::all_bits_are_set(
            RCC->BDCR, RCC_BDCR_LSERDY, a_timeout.get() - (tick_counter<Milliseconds>::get() - start));
    }

    return false;
}
void lse::bypass::enable()
{
    bit::flag::set(&(PWR->CR1), PWR_CR1_DBP);
    wait_until::all_bits_are_set(PWR->CR1, PWR_CR1_DBP);
    bit::flag::set(&(RCC->BDCR), RCC_BDCR_LSEBYP);

    bit::flag::set(&(RCC->BDCR), RCC_BDCR_LSEON);
    wait_until::all_bits_are_set(RCC->BDCR, RCC_BDCR_LSERDY);
}
bool lse::bypass::enable(Milliseconds a_timeout)
{
    const std::uint64_t start = tick_counter<Milliseconds>::get();

    bit::flag::set(&(PWR->CR1), PWR_CR1_DBP);
    if (true == wait_until::all_bits_are_set(
                    PWR->CR1, PWR_CR1_DBP, a_timeout.get() - (tick_counter<Milliseconds>::get() - start)))
    {
        bit::flag::set(&(RCC->BDCR), RCC_BDCR_LSEBYP);

        bit::flag::set(&(RCC->BDCR), RCC_BDCR_LSEON);
        return wait_until::all_bits_are_set(
            RCC->BDCR, RCC_BDCR_LSERDY, a_timeout.get() - (tick_counter<Milliseconds>::get() - start));
    }

    return false;
}
void lse::disable()
{
    bit::flag::clear(&(RCC->BDCR), RCC_BDCR_LSEON);
    wait_until::all_bits_are_cleared(RCC->BDCR, RCC_BDCR_LSERDY);
    bit::flag::clear(&(RCC->BDCR), RCC_BDCR_LSEBYP);
}
bool lse::disable(Milliseconds a_timeout)
{
    const std::uint64_t start = tick_counter<Milliseconds>::get();

    bit::flag::clear(&(RCC->BDCR), RCC_BDCR_LSEON);
    if (true == wait_until::all_bits_are_cleared(
                    RCC->BDCR, RCC_BDCR_LSERDY, a_timeout.get() - (tick_counter<Milliseconds>::get() - start)))
    {
        bit::flag::set(&(RCC->BDCR), RCC_BDCR_LSEBYP);
        return true;
    }

    return false;
}
} // namespace xmcu::soc::st::arm::m4::wb::rm0434::clocks::sources