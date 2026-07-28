#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// soc
#include <rm0434/clocks/sources/hsi48.hpp>
#include <rm0434/clocks/sources/msi.hpp>
#include <rm0434/clocks/sources/pll.hpp>
#include <rm0434/system/hsem/hsem.hpp>

// xmcu
#include <xmcu/Duration.hpp>
#include <xmcu/assertion.hpp>
#include <xmcu/non_constructible.hpp>

// std
#include <cstdint>

namespace xmcu::soc::st::arm::m4::wb::rm0434::clocks {
class clk48 : private non_constructible
{
public:
    template<typename Source_t> static void set() = delete;
    template<typename Source_t> static bool set(Milliseconds a_timeout) = delete;

    template<typename Source_t> static bool is_source() = delete;
    static std::uint32_t get_frequency_Hz()
    {
        switch (bit::flag::get(RCC->CCIPR, RCC_CCIPR_CLK48SEL))
        {
            case 0x0u: {
                return sources::hsi48::get_frequency_Hz();
            }
            break;

            case RCC_CCIPR_CLK48SEL_0: {
                return sources::pll::sai1::q::get_frequency_Hz();
            }
            break;

            case RCC_CCIPR_CLK48SEL_1: {
                return sources::pll::q::get_frequency_Hz();
            }
            break;

            case RCC_CCIPR_CLK48SEL_0 | RCC_CCIPR_CLK48SEL_1: {
                return sources::msi::get_frequency_Hz();
            }
            break;
        }

        return 0;
    }
};

template<> void clk48::set<clocks::sources::hsi48>()
{
    hkm_assert(true == clocks::sources::hsi48::is_enabled());

    Scoped_guard<system::hsem::_1_step> sem5_guard(0x5u);
    bit::flag::clear(&(RCC->CCIPR), RCC_CCIPR_CLK48SEL);
}
template<> bool clk48::set<sources::hsi48>(Milliseconds a_timeout)
{
    hkm_assert(true == clocks::sources::hsi48::is_enabled());

    Scoped_guard<system::hsem::_1_step> sem5_guard(0x5u, a_timeout);

    if (true == sem5_guard.is_locked())
    {
        bit::flag::clear(&(RCC->CCIPR), RCC_CCIPR_CLK48SEL);
        return true;
    }
    return false;
}
template<> bool clk48::is_source<sources::hsi48>()
{
    return 0x0 == bit::flag::get(RCC->CCIPR, RCC_CCIPR_CLK48SEL);
}
template<> void clk48::set<sources::msi>()
{
    hkm_assert(true == clocks::sources::msi::is_enabled());
    hkm_assert(48_MHz == clocks::sources::msi::get_frequency_Hz());

    Scoped_guard<system::hsem::_1_step> sem5_guard(0x5u);
    bit::flag::set(&(RCC->CCIPR), RCC_CCIPR_CLK48SEL, RCC_CCIPR_CLK48SEL_0 | RCC_CCIPR_CLK48SEL_1);
}
template<> bool clk48::set<sources::msi>(Milliseconds a_timeout)
{
    hkm_assert(true == clocks::sources::msi::is_enabled());
    hkm_assert(48_MHz == clocks::sources::msi::get_frequency_Hz());

    Scoped_guard<system::hsem::_1_step> sem5_guard(0x5u, a_timeout);

    if (true == sem5_guard.is_locked())
    {
        bit::flag::set(&(RCC->CCIPR), RCC_CCIPR_CLK48SEL, RCC_CCIPR_CLK48SEL_0 | RCC_CCIPR_CLK48SEL_1);
        return true;
    }
    return false;
}
template<> bool clk48::is_source<sources::msi>()
{
    return (RCC_CCIPR_CLK48SEL_0 | RCC_CCIPR_CLK48SEL_1) == bit::flag::get(RCC->CCIPR, RCC_CCIPR_CLK48SEL);
}
template<> void clk48::set<sources::pll::q>()
{
    hkm_assert(clocks::sources::pll::q::Enable_config::Output::enabled ==
               clocks::sources::pll::q::get_Enable_config().output);
    hkm_assert(48_MHz == clocks::sources::pll::q::get_frequency_Hz());

    Scoped_guard<system::hsem::_1_step> sem5_guard(0x5u);

    bit::flag::set(&(RCC->CCIPR), RCC_CCIPR_CLK48SEL, RCC_CCIPR_CLK48SEL_1);
}
template<> bool clk48::set<sources::pll::q>(Milliseconds a_timeout)
{
    hkm_assert(clocks::sources::pll::q::Enable_config::Output::enabled ==
               clocks::sources::pll::q::get_Enable_config().output);
    hkm_assert(48_MHz == clocks::sources::pll::q::get_frequency_Hz());

    Scoped_guard<system::hsem::_1_step> sem5_guard(0x5u, a_timeout);

    if (true == sem5_guard.is_locked())
    {
        bit::flag::set(&(RCC->CCIPR), RCC_CCIPR_CLK48SEL, RCC_CCIPR_CLK48SEL_1);
        return true;
    }
    return false;
}
template<> bool clk48::is_source<sources::pll::q>()
{
    return RCC_CCIPR_CLK48SEL_1 == bit::flag::get(RCC->CCIPR, RCC_CCIPR_CLK48SEL);
}
template<> void clk48::set<sources::pll::sai1::q>()
{
    hkm_assert(clocks::sources::pll::sai1::q::Enable_config::Output::enabled ==
               clocks::sources::pll::sai1::q::get_Enable_config().output);
    hkm_assert(48_MHz == clocks::sources::pll::sai1::q::get_frequency_Hz());

    Scoped_guard<system::hsem::_1_step> sem5_guard(0x5u);
    bit::flag::set(&(RCC->CCIPR), RCC_CCIPR_CLK48SEL, RCC_CCIPR_CLK48SEL_0);
}
template<> bool clk48::set<sources::pll::sai1::q>(Milliseconds a_timeout)
{
    hkm_assert(clocks::sources::pll::sai1::q::Enable_config::Output::enabled ==
               clocks::sources::pll::sai1::q::get_Enable_config().output);
    hkm_assert(48_MHz == clocks::sources::pll::sai1::q::get_frequency_Hz());

    Scoped_guard<system::hsem::_1_step> sem5_guard(0x5u, a_timeout);

    if (true == sem5_guard.is_locked())
    {
        bit::flag::set(&(RCC->CCIPR), RCC_CCIPR_CLK48SEL, RCC_CCIPR_CLK48SEL_0);
        return true;
    }
    return false;
}
template<> bool clk48::is_source<sources::pll::sai1::q>()
{
    return RCC_CCIPR_CLK48SEL_0 == bit::flag::get(RCC->CCIPR, RCC_CCIPR_CLK48SEL);
}
} // namespace xmcu::soc::st::arm::m4::wb::rm0434::clocks
