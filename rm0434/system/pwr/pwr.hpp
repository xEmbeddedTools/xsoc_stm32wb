#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// externals
#include <stm32wbxx.h>

// xmcu
#include <rm0434/clocks/sources/hse.hpp>
#include <rm0434/clocks/sources/hsi16.hpp>
#include <rm0434/clocks/sources/msi.hpp>
#include <rm0434/clocks/sources/pll.hpp>
#include <rm0434/peripherals/internal_flash/internal_flash.hpp>
#include <rm0434/system/mcu/mcu.hpp>
#include <xmcu/non_constructible.hpp>

namespace xmcu::soc::st::arm::m4::wb::rm0434::system {
template<typename MCU_t> class pwr : private xmcu::non_constructible
{
};

template<> class pwr<mcu<1u>> : private xmcu::non_constructible
{
public:
    enum class Voltage_scaling : std::uint32_t
    {
        _1 = PWR_CR1_VOS_0,
        _2 = PWR_CR1_VOS_1
    };

    struct stop_mode : private xmcu::non_constructible
    {
        enum class Type : std::uint32_t
        {
            _0 = 0x0u,
            _1 = PWR_CR1_LPMS_0,
            _2 = PWR_CR1_LPMS_1,
            standby = PWR_CR1_LPMS_0 | PWR_CR1_LPMS_1
        };
        enum class Method : std::uint32_t
        {
            wfi,
            wfe,
            none
        };
        enum class Sleeponexit : std::uint32_t
        {
            disabled = 0x0u,
            enabled = SCB_SCR_SLEEPONEXIT_Msk
        };

        template<typename Source_t>
        static void enter(Type a_type,
                          Method a_method,
                          Sleeponexit a_sleeponexit,
                          peripherals::internal_flash::Latency a_desired_flash_latency) = delete;
    };
    struct backup_domain : private xmcu::non_constructible
    {
        static bool enable(xmcu::Milliseconds timeout_a);
        static bool disable(xmcu::Milliseconds timeout_a);
    };

    static void set_voltage_scaling(Voltage_scaling a_scaling);
    static bool set_voltage_scaling(Voltage_scaling a_scaling, xmcu::Milliseconds a_timeout);

    static Voltage_scaling get_voltage_scaling();
};

template<> class pwr<mcu<2u>> : private xmcu::non_constructible
{
public:
    enum class Boot_after_reset_or_stop : std::uint32_t
    {
        enable = PWR_CR4_C2BOOT,
        disable = 0x0u
    };

    static void set_boot(Boot_after_reset_or_stop a_mode);
};

template<> void
pwr<mcu<1u>>::stop_mode::enter<clocks::sources::hsi16>(Type a_type,
                                                       Method a_method,
                                                       Sleeponexit a_sleeponexit,
                                                       peripherals::internal_flash::Latency a_desired_flash_latency);
} // namespace xmcu::soc::st::arm::m4::wb::rm0434::system