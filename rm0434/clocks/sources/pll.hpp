#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// externals
#include <stm32wbxx.h>

// xmcu
#include <rm0434/clocks/sources/hse.hpp>
#include <rm0434/clocks/sources/hsi16.hpp>
#include <rm0434/clocks/sources/msi.hpp>
#include <xmcu/Duration.hpp>
#include <xmcu/Limited.hpp>
#include <xmcu/bit.hpp>
#include <xmcu/non_constructible.hpp>
#include <xmcu/various.hpp>

namespace xmcu::soc::st::arm::m4::wb::rm0434::clocks::sources {
class pll : private non_constructible
{
public:
    enum class M : std::uint32_t
    {
        _1 = 0,
        _2 = RCC_PLLCFGR_PLLM_0,
        _3 = RCC_PLLCFGR_PLLM_1,
        _4 = RCC_PLLCFGR_PLLM_1 | RCC_PLLCFGR_PLLM_0,
        _5 = RCC_PLLCFGR_PLLM_2,
        _6 = RCC_PLLCFGR_PLLM_2 | RCC_PLLCFGR_PLLM_0,
        _7 = RCC_PLLCFGR_PLLM_2 | RCC_PLLCFGR_PLLM_1,
        _8 = RCC_PLLCFGR_PLLM_2 | RCC_PLLCFGR_PLLM_1 | RCC_PLLCFGR_PLLM_0,
    };

    class r : private non_constructible
    {
    public:
        struct Enable_config
        {
            enum class Output : std::uint32_t
            {
                disabled,
                enabled = RCC_PLLCFGR_PLLREN
            };

            enum class Divider : std::uint32_t
            {
                _2 = RCC_PLLCFGR_PLLR_0,
                _3 = RCC_PLLCFGR_PLLR_1,
                _4 = RCC_PLLCFGR_PLLR_0 | RCC_PLLCFGR_PLLR_1,
                _5 = RCC_PLLCFGR_PLLR_2,
                _6 = RCC_PLLCFGR_PLLR_2 | RCC_PLLCFGR_PLLR_0,
                _7 = RCC_PLLCFGR_PLLR_2 | RCC_PLLCFGR_PLLR_1,
                _8 = RCC_PLLCFGR_PLLR_2 | RCC_PLLCFGR_PLLR_1 | RCC_PLLCFGR_PLLR_0,
            };

            Divider divider = various::get_enum_incorrect_value<Divider>();
            Output output = various::get_enum_incorrect_value<Output>();
        };

        static Enable_config get_Enable_config()
        {
            return { static_cast<Enable_config::Divider>(bit::flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLR_Msk)),
                     static_cast<Enable_config::Output>(bit::flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLREN)) };
        }
        static std::uint32_t get_frequency_Hz();
    };
    class q : private non_constructible
    {
    public:
        struct Enable_config
        {
            enum class Output : std::uint32_t
            {
                disabled,
                enabled = RCC_PLLCFGR_PLLQEN
            };

            enum class Divider : std::uint32_t
            {
                _2 = RCC_PLLCFGR_PLLQ_0,
                _3 = RCC_PLLCFGR_PLLQ_1,
                _4 = RCC_PLLCFGR_PLLQ_0 | RCC_PLLCFGR_PLLQ_1,
                _5 = RCC_PLLCFGR_PLLQ_2,
                _6 = RCC_PLLCFGR_PLLQ_2 | RCC_PLLCFGR_PLLQ_0,
                _7 = RCC_PLLCFGR_PLLQ_2 | RCC_PLLCFGR_PLLQ_1,
                _8 = RCC_PLLCFGR_PLLQ_2 | RCC_PLLCFGR_PLLQ_1 | RCC_PLLCFGR_PLLQ_0,
            };

            Divider divider = various::get_enum_incorrect_value<Divider>();
            Output output = various::get_enum_incorrect_value<Output>();
        };

        static Enable_config get_Enable_config()
        {
            return { static_cast<Enable_config::Divider>(bit::flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLQ_Msk)),
                     static_cast<Enable_config::Output>(bit::flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLQEN)) };
        }
        static std::uint32_t get_frequency_Hz();
    };
    class p : private non_constructible
    {
    public:
        struct Enable_config
        {
            enum class Output : std::uint32_t
            {
                disabled = 0x0u,
                enabled = RCC_PLLCFGR_PLLPEN
            };

            Limited<std::uint32_t, 2, 31> divider;
            Output output = various::get_enum_incorrect_value<Output>();
        };

        static Enable_config get_Enable_config()
        {
            return { (bit::flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLP_Msk) >> RCC_PLLCFGR_PLLP_Pos) + 1u,
                     static_cast<Enable_config::Output>(bit::flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLPEN)) };
        }
        static std::uint32_t get_frequency_Hz();
    };

    class sai1 : private non_constructible
    {
    public:
        class r : private non_constructible
        {
        public:
            struct Enable_config
            {
                enum class Output : std::uint32_t
                {
                    disabled,
                    enabled = RCC_PLLSAI1CFGR_PLLREN
                };

                enum class Divider : std::uint32_t
                {
                    _2 = RCC_PLLSAI1CFGR_PLLR_0,
                    _3 = RCC_PLLSAI1CFGR_PLLR_1,
                    _4 = RCC_PLLSAI1CFGR_PLLR_0 | RCC_PLLSAI1CFGR_PLLR_1,
                    _5 = RCC_PLLSAI1CFGR_PLLR_2,
                    _6 = RCC_PLLSAI1CFGR_PLLR_0 | RCC_PLLSAI1CFGR_PLLR_2,
                    _7 = RCC_PLLSAI1CFGR_PLLR_1 | RCC_PLLSAI1CFGR_PLLR_2,
                    _8 = RCC_PLLSAI1CFGR_PLLR_0 | RCC_PLLSAI1CFGR_PLLR_1 | RCC_PLLSAI1CFGR_PLLR_2
                };

                Divider divider = various::get_enum_incorrect_value<Divider>();
                Output output = various::get_enum_incorrect_value<Output>();
            };

            static Enable_config get_Enable_config();
            static std::uint32_t get_frequency_Hz();
        };
        class q : private non_constructible
        {
        public:
            struct Enable_config
            {
                enum class Output : std::uint32_t
                {
                    disabled,
                    enabled = RCC_PLLSAI1CFGR_PLLQEN
                };

                enum class Divider : std::uint32_t
                {
                    _2 = RCC_PLLSAI1CFGR_PLLQ_0,
                    _3 = RCC_PLLSAI1CFGR_PLLQ_1,
                    _4 = RCC_PLLSAI1CFGR_PLLQ_0 | RCC_PLLSAI1CFGR_PLLQ_1,
                    _5 = RCC_PLLSAI1CFGR_PLLQ_2,
                    _6 = RCC_PLLSAI1CFGR_PLLQ_0 | RCC_PLLSAI1CFGR_PLLQ_2,
                    _7 = RCC_PLLSAI1CFGR_PLLQ_1 | RCC_PLLSAI1CFGR_PLLQ_2,
                    _8 = RCC_PLLSAI1CFGR_PLLQ_0 | RCC_PLLSAI1CFGR_PLLQ_1 | RCC_PLLSAI1CFGR_PLLQ_2
                };

                Divider divider = various::get_enum_incorrect_value<Divider>();
                Output output = various::get_enum_incorrect_value<Output>();
            };

            static Enable_config get_Enable_config();
            static std::uint32_t get_frequency_Hz();
        };
        class p : private non_constructible
        {
        public:
            struct Enable_config
            {
                enum class Output : std::uint32_t
                {
                    disabled,
                    enabled = RCC_PLLSAI1CFGR_PLLPEN
                };

                Limited<std::uint32_t, 2, 31> divider;
                Output output = various::get_enum_incorrect_value<Output>();
            };

            static Enable_config get_Enable_config();
            static std::uint32_t get_frequency_Hz();
        };

        static void enable(Limited<std::uint32_t, 8u, 86u> a_N,
                           const r::Enable_config& a_R,
                           const q::Enable_config& a_Q,
                           const p::Enable_config& a_P);
        static bool enable(Limited<std::uint32_t, 8u, 86u> a_N,
                           const r::Enable_config& a_R,
                           const q::Enable_config& a_Q,
                           const p::Enable_config& a_P,
                           Milliseconds a_timeout);
        static void disable();

        static bool is_enabled();
    };

    template<typename Source_t> static void enable(M a_M,
                                                   Limited<std::uint32_t, 8u, 86u> a_N,
                                                   const r::Enable_config& a_R,
                                                   const q::Enable_config& a_Q,
                                                   const p::Enable_config& a_P) = delete;
    template<typename Source_t, hse::Prescaler> static void enable(M a_M,
                                                                   Limited<std::uint32_t, 8u, 86u> a_N,
                                                                   const r::Enable_config& a_R,
                                                                   const q::Enable_config& a_Q,
                                                                   const p::Enable_config& a_P) = delete;

    template<typename Source_t> static bool enable(M a_M,
                                                   Limited<std::uint32_t, 8u, 86u> a_N,
                                                   const r::Enable_config& a_R,
                                                   const q::Enable_config& a_Q,
                                                   const p::Enable_config& a_P,
                                                   Milliseconds a_timeout) = delete;
    template<typename Source_t, hse::Prescaler> static bool enable(M a_M,
                                                                   Limited<std::uint32_t, 8u, 86u> a_N,
                                                                   const r::Enable_config& a_R,
                                                                   const q::Enable_config& a_Q,
                                                                   const p::Enable_config& a_P,
                                                                   Milliseconds a_timeout) = delete;

    static void disable();

    static bool is_enabled()
    {
        return bit::flag::is(RCC->CR, RCC_CR_PLLRDY);
    }
};
template<> void pll::enable<msi>(M a_M,
                                 Limited<std::uint32_t, 8u, 86u> a_N,
                                 const r::Enable_config& a_R,
                                 const q::Enable_config& a_Q,
                                 const p::Enable_config& a_P);
template<> void pll::enable<hsi16>(M a_M,
                                   Limited<std::uint32_t, 8u, 86u> a_N,
                                   const r::Enable_config& a_R,
                                   const q::Enable_config& a_Q,
                                   const p::Enable_config& a_P);
template<> void pll::enable<hse, hse::Prescaler::_1>(M a_M,
                                                     Limited<std::uint32_t, 8u, 86u> a_N,
                                                     const r::Enable_config& a_R,
                                                     const q::Enable_config& a_Q,
                                                     const p::Enable_config& a_P);
template<> void pll::enable<hse, hse::Prescaler::_2>(M a_M,
                                                     Limited<std::uint32_t, 8u, 86u> a_N,
                                                     const r::Enable_config& a_R,
                                                     const q::Enable_config& a_Q,
                                                     const p::Enable_config& a_P);
template<> bool pll::enable<msi>(M a_M,
                                 Limited<std::uint32_t, 8u, 86u> a_N,
                                 const r::Enable_config& a_R,
                                 const q::Enable_config& a_Q,
                                 const p::Enable_config& a_P,
                                 Milliseconds a_timeout);
template<> bool pll::enable<hsi16>(M a_M,
                                   Limited<std::uint32_t, 8u, 86u> a_N,
                                   const r::Enable_config& a_R,
                                   const q::Enable_config& a_Q,
                                   const p::Enable_config& a_P,
                                   Milliseconds a_timeout);
template<> bool pll::enable<hse, hse::Prescaler::_1>(M a_M,
                                                     Limited<std::uint32_t, 8u, 86u> a_N,
                                                     const r::Enable_config& a_R,
                                                     const q::Enable_config& a_Q,
                                                     const p::Enable_config& a_P,
                                                     Milliseconds a_timeout);
template<> bool pll::enable<hse, hse::Prescaler::_2>(M a_M,
                                                     Limited<std::uint32_t, 8u, 86u> a_N,
                                                     const r::Enable_config& a_R,
                                                     const q::Enable_config& a_Q,
                                                     const p::Enable_config& a_P,
                                                     Milliseconds a_timeout);
} // namespace xmcu::soc::st::arm::m4::wb::rm0434::clocks::sources