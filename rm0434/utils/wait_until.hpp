#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// xmcu
#include <rm0434/utils/tick_counter.hpp>
#include <xmcu/Duration.hpp>
#include <xmcu/bit.hpp>
#include <xmcu/non_constructible.hpp>

namespace xmcu::soc::st::arm::m4::wb::rm0434::utils {
struct wait_until : private non_constructible
{
    template<typename Register, typename Mask> static void all_bits_are_set(const Register& a_register, Mask a_mask)
    {
        while (false == bit::flag::is(a_register, a_mask)) continue;
    }

    template<typename Register, typename Mask> static void any_bit_is_set(const Register& a_register, Mask a_mask)
    {
        while (false == bit::is_any(a_register, a_mask)) continue;
    }

    template<typename Register, typename Mask, typename Flag>
    static void masked_bits_are_set(const Register& a_register, Mask a_mask, Flag a_value)
    {
        while (bit::flag::get(a_register, a_mask) != a_value) continue;
    }

    template<typename Register, typename Mask> static void all_bits_are_cleared(const Register& a_register, Mask a_mask)
    {
        while (false == bit::flag::is(~a_register, a_mask)) continue;
    }

    template<typename Register, typename Mask> static void any_bit_is_cleared(const Register& a_register, Mask a_mask)
    {
        while (false == bit::is_any(~a_register, a_mask)) continue;
    }

    template<typename Register, typename Mask>
    static bool all_bits_are_set(const Register& a_register, Mask a_mask, Milliseconds a_timeout)
    {
        const std::uint64_t timeout_end = tick_counter<Milliseconds>::get() + a_timeout.get();
        bool status = false;

        while (tick_counter<Milliseconds>::get() < timeout_end && false == status)
        {
            status = bit::flag::is(a_register, a_mask);
        }

        return status;
    }

    template<typename Register, typename Mask>
    static bool any_bit_is_set(const Register& a_register, Mask a_mask, Milliseconds a_timeout)
    {
        const std::uint64_t timeout_end = tick_counter<Milliseconds>::get() + a_timeout.get();
        bool status = false;

        while (tick_counter<Milliseconds>::get() < timeout_end && false == status)
        {
            status = bit::is_any(a_register, a_mask);
        }

        return status;
    }
    template<typename Register, typename Mask, typename Flag>
    static bool masked_bits_are_set(Register& a_register, Mask a_mask, Flag a_value, Milliseconds a_timeout)
    {
        const std::uint64_t timeout_end = tick_counter<Milliseconds>::get() + a_timeout.get();
        bool status = false;

        while (tick_counter<Milliseconds>::get() < timeout_end && false == status)
        {
            status = (bit::flag::get(a_register, a_mask) == a_value);
        }

        return status;
    }

    template<typename Register, typename Mask>
    static bool all_bits_are_cleared(const Register& a_register, Mask a_mask, Milliseconds a_timeout)
    {
        const std::uint64_t timeout_end = tick_counter<Milliseconds>::get() + a_timeout.get();
        bool status = false;

        while (tick_counter<Milliseconds>::get() < timeout_end && false == status)
        {
            status = bit::flag::is(~a_register, a_mask);
        }

        return status;
    }
    template<typename Register, typename Mask>
    static bool any_bit_is_cleared(const Register& a_register, Mask a_mask, Milliseconds a_timeout)
    {
        const std::uint64_t timeout_end = tick_counter<Milliseconds>::get() + a_timeout.get();
        bool status = false;

        while (tick_counter<Milliseconds>::get() < timeout_end && status == false)
        {
            status = bit::is_any(~a_register, a_mask);
        }

        return status;
    }
};
} // namespace xmcu::soc::st::arm::m4::wb::rm0434::utils