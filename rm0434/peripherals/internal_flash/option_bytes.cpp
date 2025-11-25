/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// this
#include <rm0434/peripherals/internal_flash/option_bytes.hpp>

// externals
#include <stm32wbxx.h>

// xmcu
#include <rm0434/peripherals/internal_flash/internal_flash.hpp>
#include <rm0434/system/hsem/hsem.hpp>
#include <rm0434/utils/tick_counter.hpp>
#include <rm0434/utils/wait_until.hpp>
#include <soc/Scoped_guard.hpp>
#include <soc/st/arm/m4/nvic.hpp>
#include <xmcu/bit.hpp>

namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals {
using namespace xmcu;
using namespace xmcu::soc::st::arm::m4::wb::rm0434::utils;
using namespace xmcu::soc::st::arm::m4::wb::rm0434::system;

void option_bytes::unlocker::unlock()
{
    wait_until::all_bits_are_cleared(FLASH->SR, FLASH_SR_BSY);

    if (true == bit::flag::is(FLASH->CR, FLASH_CR_OPTLOCK))
    {
        Scoped_guard<nvic> interrupt_guard;

        FLASH->OPTKEYR = 0x08192A3Bu;
        FLASH->OPTKEYR = 0x4C5D6E7Fu;
    }
}
bool option_bytes::unlocker::unlock(Milliseconds a_timeout)
{
    if (true == wait_until::all_bits_are_cleared(FLASH->SR, FLASH_SR_BSY, a_timeout))
    {
        if (true == bit::flag::is(FLASH->CR, FLASH_CR_OPTLOCK))
        {
            Scoped_guard<nvic> interrupt_guard;

            FLASH->OPTKEYR = 0x08192A3Bu;
            FLASH->OPTKEYR = 0x4C5D6E7Fu;

            return true;
        }
    }

    return false;
}
void option_bytes::unlocker::lock()
{
    bit::flag::set(&(FLASH->CR), FLASH_CR_OPTLOCK);
}

void option_bytes::BOR::set(Level a_level)
{
    Scoped_guard<internal_flash::unlocker> flash_guard;
    Scoped_guard<option_bytes::unlocker> ob_guard;

    bit::flag::set(&(FLASH->OPTR), FLASH_OPTR_BOR_LEV, (static_cast<std::uint32_t>(a_level)));
}
bool option_bytes::BOR::set(Level level_a, Milliseconds timeout_a)
{
    const std::uint64_t timeout_timestamp = tick_counter<Milliseconds>::get() + timeout_a.get();
    Scoped_guard<internal_flash::unlocker> flash_guard(timeout_timestamp - tick_counter<Milliseconds>::get());

    if (true == flash_guard.is_unlocked())
    {
        Scoped_guard<option_bytes::unlocker> ob_guard(timeout_timestamp - tick_counter<Milliseconds>::get());

        if (true == ob_guard.is_unlocked())
        {
            bit::flag::set(&(FLASH->OPTR), FLASH_OPTR_BOR_LEV, (static_cast<std::uint32_t>(level_a)));
            return true;
        }
    }

    return false;
}

option_bytes::BOR::Level option_bytes::BOR::get()
{
    return static_cast<Level>(bit::flag::get(FLASH->OPTR, FLASH_OPTR_BOR_LEV));
}

void option_bytes::RDP::set(Level level_a)
{
    Scoped_guard<internal_flash::unlocker> flash_guard;
    Scoped_guard<option_bytes::unlocker> ob_guard;

    bit::flag::set(&(FLASH->OPTR), FLASH_OPTR_RDP, static_cast<std::uint32_t>(level_a));
}
bool option_bytes::RDP::set(Level level_a, Milliseconds timeout_a)
{
    const std::uint64_t timeout_timestamp = tick_counter<Milliseconds>::get() + timeout_a.get();
    Scoped_guard<internal_flash::unlocker> flash_guard(timeout_timestamp - tick_counter<Milliseconds>::get());

    if (true == flash_guard.is_unlocked())
    {
        Scoped_guard<option_bytes::unlocker> ob_guard(timeout_timestamp - tick_counter<Milliseconds>::get());

        if (true == ob_guard.is_unlocked())
        {
            bit::flag::set(&(FLASH->OPTR), FLASH_OPTR_RDP, static_cast<std::uint32_t>(level_a));
            return true;
        }
    }

    return false;
}

option_bytes::RDP::Level option_bytes::RDP::get()
{
    std::uint32_t reg_val = bit::flag::get(FLASH->OPTR, FLASH_OPTR_RDP);

    switch (reg_val)
    {
        case 0xAAu:
            return Level::_0;
        case 0xCCu:
            return Level::_2;
    }

    return Level::_1;
}

bool option_bytes::launch()
{
    Scoped_guard<hsem::_1_step> sem2_guard(0x2u);
    Scoped_guard<internal_flash::unlocker> flash_guard;
    Scoped_guard<option_bytes::unlocker> ob_guard;

    while (true)
    {
        wait_until::all_bits_are_cleared(FLASH->SR, FLASH_SR_PESD);

        Scoped_guard<nvic> interrupt_guard;

        if (false == hsem::is_locked(0x6u) && true == hsem::_1_step::try_lock(0x7u))
        {
            wait_until::all_bits_are_cleared(FLASH->SR, FLASH_SR_CFGBSY);
            bit::flag::set(&(FLASH->CR), FLASH_CR_OPTSTRT);

            wait_until::all_bits_are_cleared(FLASH->SR, FLASH_SR_BSY);

            bit::flag::set(&(FLASH->CR), FLASH_CR_OBL_LAUNCH);

            // we should never get to this point
            hsem::_1_step::unlock(0x7u);
            return false;
        }
    }

    return false;
}
bool option_bytes::launch(Milliseconds timeout_a)
{
    const std::uint64_t timeout_timestamp = tick_counter<Milliseconds>::get() + timeout_a.get();
    Scoped_guard<hsem::_1_step> sem2_guard(0x2u, timeout_timestamp - tick_counter<Milliseconds>::get());

    if (true == sem2_guard.is_locked())
    {
        Scoped_guard<internal_flash::unlocker> flash_guard(timeout_timestamp - tick_counter<Milliseconds>::get());

        if (true == flash_guard.is_unlocked())
        {
            Scoped_guard<option_bytes::unlocker> ob_guard(timeout_timestamp - tick_counter<Milliseconds>::get());

            if (true == ob_guard.is_unlocked())
            {
                bool repeat = false;

                do
                {
                    if (true == wait_until::all_bits_are_cleared(
                                    FLASH->SR, FLASH_SR_PESD, timeout_timestamp - tick_counter<Milliseconds>::get()))
                    {
                        Scoped_guard<nvic> interrupt_guard;

                        if (false == hsem::is_locked(0x6u) && true == hsem::_1_step::try_lock(0x7u))
                        {
                            if (true ==
                                wait_until::all_bits_are_cleared(
                                    FLASH->SR, FLASH_SR_CFGBSY, timeout_timestamp - tick_counter<Milliseconds>::get()))
                            {
                                bit::flag::set(&(FLASH->CR), FLASH_CR_OPTSTRT);

                                if (true ==
                                    wait_until::all_bits_are_cleared(
                                        FLASH->SR, FLASH_SR_BSY, timeout_timestamp - tick_counter<Milliseconds>::get()))
                                {
                                    bit::flag::set(&(FLASH->CR), FLASH_CR_OBL_LAUNCH);

                                    // we should never get to this point
                                    hsem::_1_step::unlock(0x7u);
                                    return false;
                                }
                            }
                        }
                        else
                        {
                            repeat = true;
                        }
                    }

                } while (true == repeat && (timeout_timestamp - tick_counter<Milliseconds>::get()) > 0u);
            }
        }
    }

    return false;
}
} // namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals