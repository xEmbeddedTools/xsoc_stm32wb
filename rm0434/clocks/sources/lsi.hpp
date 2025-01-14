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
#include <xmcu/bit.hpp>
#include <xmcu/non_constructible.hpp>

namespace xmcu::soc::st::arm::m4::wb::rm0434::clocks::sources {
class lsi : private non_constructible
{
public:
    enum class Id : std::uint32_t
    {
        _1 = 0x0u,
        _2 = 0x2u
    };

    static void enable(Id a_id);
    static bool enable(Id a_id, Milliseconds a_timeout);

    static void disable(Id a_id);
    static bool disable(Id a_id, Milliseconds a_timeout);

    static bool is_selected(Id a_id);

    static bool is_enabled(Id a_id)
    {
        return bit::flag::is(RCC->CSR, RCC_CSR_LSI1RDY);
    }

    static std::uint32_t get_frequency_Hz(Id a_id)
    {
        if (true == is_enabled(a_id))
        {
            return 32_kHz;
        }

        return 0u;
    }

    static std::uint32_t get_frequency_Hz()
    {
        if (true == is_enabled(Id::_1) || true == is_enabled(Id::_2))
        {
            return 32_kHz;
        }

        return 0u;
    }
};
} // namespace xmcu::soc::st::arm::m4::wb::rm0434::clocks::sources