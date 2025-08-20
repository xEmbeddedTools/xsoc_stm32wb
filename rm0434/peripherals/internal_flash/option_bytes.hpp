#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// CMSIS
#include <stm32wbxx.h>

// xmcu
#include <soc/Scoped_guard.hpp>
#include <xmcu/Duration.hpp>
#include <xmcu/Non_copyable.hpp>
#include <xmcu/non_constructible.hpp>

namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals {
class option_bytes : private non_constructible
{
public:
    class unlocker : private non_constructible
    {
    public:
        static void unlock();
        static bool unlock(Milliseconds a_timeout);

        static void lock();
    };

    struct secure_flash : private non_constructible
    {
        static std::uint32_t get_start_address();
        static std::uint32_t get_start_address(Milliseconds a_timeout);
    };

    struct BOR : private non_constructible
    {
        enum class Level : std::uint32_t
        {
            _1_7V = 0x0u,
            _2_0V = FLASH_OPTR_BOR_LEV_0,
            _2_2V = FLASH_OPTR_BOR_LEV_1,
            _2_5V = FLASH_OPTR_BOR_LEV_0 | FLASH_OPTR_BOR_LEV_1,
            _2_8V = FLASH_OPTR_BOR_LEV_2
        };

        static bool set(Level a_level);

        static Level get_level();
        static std::uint16_t get_mV();
    };
};

} // namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals

namespace xmcu::soc {
template<> class Scoped_guard<st::arm::m4::wb::rm0434::peripherals::option_bytes::unlocker> : private Non_copyable
{
public:
    Scoped_guard()
        : unlocked(false)
    {
        st::arm::m4::wb::rm0434::peripherals::option_bytes::unlocker::unlock();
        this->unlocked = true;
    }

    Scoped_guard(Milliseconds a_timeout)
        : unlocked(st::arm::m4::wb::rm0434::peripherals::option_bytes::unlocker::unlock(a_timeout))
    {
    }

    ~Scoped_guard()
    {
        st::arm::m4::wb::rm0434::peripherals::option_bytes::unlocker::lock();
    }

    bool is_unlocked() const
    {
        return this->unlocked;
    }

private:
    bool unlocked;
};
} // namespace xmcu::soc