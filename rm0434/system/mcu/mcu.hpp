#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// std
#include <cstddef>
#include <cstdint>

// externals
#include <stm32wbxx.h>

// xmcu
#include <rm0434/clocks/sources/hse.hpp>
#include <rm0434/clocks/sources/hsi16.hpp>
#include <rm0434/clocks/sources/hsi48.hpp>
#include <rm0434/clocks/sources/lse.hpp>
#include <rm0434/clocks/sources/pll.hpp>
#include <rm0434/rcc.hpp>
#include <xmcu/bit.hpp>
#include <xmcu/non_constructible.hpp>

namespace xmcu::soc::st::arm::m4::wb::rm0434::system {
template<std::size_t id> class mcu : private non_constructible
{
};
template<> class mcu<1u> : private non_constructible
{
public:
    enum class FPU_mode : std::uint32_t
    {
        disabled = 0x000000u,
        privileged_access_only = 0xA00000u,
        enabled = 0xF00000u,
    };
    enum class DWT_mode : std::uint32_t
    {
        disabled,
        enabled
    };

    enum class Reset_source : std::uint32_t
    {
        unknown,
        low_power = RCC_CSR_LPWRRSTF,
        window_watchdog = RCC_CSR_WWDGRSTF,
        independent_window_watchdog = RCC_CSR_IWDGRSTF,
        software = RCC_CSR_SFTRSTF,
        bor = RCC_CSR_BORRSTF,
        pin = RCC_CSR_PINRSTF,
        option_byte = RCC_CSR_OBLRSTF
    };

    enum class Package : std::uint32_t
    {
        WLCSP100_or_UFBGA129 = 0x11u,
        VFQFPN68 = 0x13u,
        UFQPFN48 = 0xAu
    };

    struct Id
    {
        std::uint8_t serial_number[12] = { 0 };
        std::uint32_t type = 0;
    };

    static void halt()
    {
        __disable_irq();
        __builtin_trap();

        while (true) continue;
    }

    static Id get_id()
    {
        const std::uint8_t* p_id_location = reinterpret_cast<std::uint8_t*>(UID_BASE);

        return { { p_id_location[0],
                   p_id_location[1],
                   p_id_location[2],
                   p_id_location[3],
                   p_id_location[4],
                   p_id_location[5],
                   p_id_location[6],
                   p_id_location[7],
                   p_id_location[8],
                   p_id_location[9],
                   p_id_location[10],
                   p_id_location[11] },

                 DBGMCU->IDCODE };
    }

    static std::uint16_t get_verfint_reference()
    {
        const std::uint16_t value = *(reinterpret_cast<const std::uint16_t*>(0x1FFF75AA));
        return value;
    }

    static void set_vector_table_address(std::uint32_t a_address)
    {
        SCB->VTOR = a_address;
    }

    static void set_main_stack_pointer(std::uint32_t a_address)
    {
        __set_MSP(a_address);
    }

    static void set_DWT_mode(DWT_mode a_mode)
    {
        switch (a_mode)
        {
            case DWT_mode::enabled: {
                bit::flag::set(&(CoreDebug->DEMCR), CoreDebug_DEMCR_TRCENA_Msk);
                bit::flag::set(&(DWT->CTRL), DWT_CTRL_CYCCNTENA_Msk);
            }
            break;

            case DWT_mode::disabled: {
                bit::flag::clear(&(CoreDebug->DEMCR), CoreDebug_DEMCR_TRCENA_Msk);
                bit::flag::clear(&(DWT->CTRL), DWT_CTRL_CYCCNTENA_Msk);
            }
            break;
        }
    }

    static void set_FPU_mode(FPU_mode a_mode)
    {
        bit::flag::set(&(SCB->CPACR), ((3u << 10u * 2u) | (3u << 11u * 2u)), static_cast<uint32_t>(a_mode));
    }

    static DWT_mode get_DWT_mode()
    {
        return static_cast<DWT_mode>(bit::flag::is(DWT->CTRL, DWT_CTRL_CYCCNTENA_Msk));
    }

    static FPU_mode get_FPU_mode()
    {
        return static_cast<FPU_mode>(SCB->CPACR);
    }

    static Reset_source get_reset_source()
    {
        auto l_reader = []() {
            volatile std::uint32_t result { RCC->CSR };
            RCC->CSR = result | RCC_CSR_RMVF;
            return static_cast<Reset_source>(result & 0xFE000000u);
        };
        static Reset_source stored_value = l_reader();
        return stored_value;
    }

    static constexpr Reset_source flags[] = {
        Reset_source::pin,
        Reset_source::bor,
        Reset_source::window_watchdog,
        Reset_source::independent_window_watchdog,
        Reset_source::software,
        Reset_source::low_power,
        Reset_source::option_byte,
    };

    static constexpr Package get_package()
    {
        return static_cast<Package>(*(reinterpret_cast<std::uint32_t*>(PACKAGE_BASE)));
    }

    static bool is_in_debug_mode()
    {
        return bit::flag::is(CoreDebug->DHCSR, CoreDebug_DHCSR_C_DEBUGEN_Msk);
    }

    static std::uint32_t get_unique_device_number()
    {
        return static_cast<std::uint32_t>(*(reinterpret_cast<std::uint32_t*>(UID64_BASE)));
    }

    static std::uint32_t get_manufacturer_id()
    {
        return static_cast<std::uint32_t>(((*reinterpret_cast<uint32_t*>(UID64_BASE) + 1u) >> 8u) & 0x00FFFFFFu);
    }

    static std::uint32_t get_device_id()
    {
        return static_cast<std::uint32_t>((*(reinterpret_cast<uint32_t*>(UID64_BASE) + 1U)) & 0x000000FFu);
    }
};

constexpr mcu<1u>::Reset_source operator~(mcu<1u>::Reset_source flag_a)
{
    return static_cast<mcu<1u>::Reset_source>(~static_cast<std::uint32_t>(flag_a));
}

constexpr mcu<1u>::Reset_source operator|(mcu<1u>::Reset_source left_a, mcu<1u>::Reset_source right_a)
{
    return static_cast<mcu<1u>::Reset_source>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}
constexpr mcu<1u>::Reset_source operator&(mcu<1u>::Reset_source left_a, mcu<1u>::Reset_source right_a)
{
    return static_cast<mcu<1u>::Reset_source>(static_cast<std::uint32_t>(left_a) & static_cast<std::uint32_t>(right_a));
}

constexpr mcu<1u>::Reset_source& operator&=(mcu<1u>::Reset_source& left_a, mcu<1u>::Reset_source right_a)
{
    left_a = (left_a & right_a);
    return left_a;
}
constexpr mcu<1u>::Reset_source& operator|=(mcu<1u>::Reset_source& left_a, mcu<1u>::Reset_source right_a)
{
    left_a = (left_a | right_a);
    return left_a;
}
} // namespace xmcu::soc::st::arm::m4::wb::rm0434::system