#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// externals
#include <stm32wbxx.h>

// xmcu
#include <xmcu/Duration.hpp>
#include <xmcu/Frequency.hpp>
#include <xmcu/Limited.hpp>
#include <xmcu/bit.hpp>
#include <xmcu/non_constructible.hpp>

namespace xmcu::soc::st::arm::m4::wb::rm0434::clocks::sources {
class hse : private non_constructible
{
public:
    enum class Prescaler : std::uint32_t
    {
        _1 = 0x0u,
        _2 = RCC_CR_HSEPRE
    };

    struct tune : private non_constructible
    {
        using Capacitor_tuning = Limited<std::uint32_t, 0, 63>;

        enum class Current_control_max_limit : std::uint32_t
        {
            _0_18 = 0x0u,
            _0_57 = RCC_HSECR_HSEGMC0,
            _0_78 = RCC_HSECR_HSEGMC1,
            _1_13 = RCC_HSECR_HSEGMC1 | RCC_HSECR_HSEGMC1,
            _0_61 = RCC_HSECR_HSEGMC2,
            _1_65 = RCC_HSECR_HSEGMC0 | RCC_HSECR_HSEGMC2,
            _2_12 = RCC_HSECR_HSEGMC1 | RCC_HSECR_HSEGMC2,
            _2_84 = RCC_HSECR_HSEGMC0 | RCC_HSECR_HSEGMC1 | RCC_HSECR_HSEGMC2
        };

        enum class Amplifier_threshold : std::uint32_t
        {
            _1_2 = 0x0u,
            _3_4 = RCC_HSECR_HSES
        };

        static void set(Capacitor_tuning a_capacitor_tuning);
        static void set(Current_control_max_limit a_current_control_max_limit);
        static void set(Amplifier_threshold a_amplifier_threshold);

        static Capacitor_tuning get_Capacitor_tuning();
        static Current_control_max_limit get_Current_control_max_limit();
        static Amplifier_threshold get_amplifier_threshold();
    };

    static void enable();
    static bool enable(Milliseconds a_timeout);

    static void disable();
    static bool disable(Milliseconds a_timeout);

    static bool is_enabled()
    {
        return bit::flag::is(RCC->CR, RCC_CR_HSERDY);
    }

    static std::uint32_t get_frequency_Hz()
    {
        if (true == is_enabled())
        {
            return 32_MHz;
        }

        return 0u;
    }
};
} // namespace xmcu::soc::st::arm::m4::wb::rm0434::clocks::sources