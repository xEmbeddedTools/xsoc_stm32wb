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

    struct BOR : private non_constructible
    {
        enum class Level : std::uint32_t
        {
            _0 = 0x0u,
            _1 = FLASH_OPTR_BOR_LEV_0,
            _2 = FLASH_OPTR_BOR_LEV_1,
            _3 = FLASH_OPTR_BOR_LEV_0 | FLASH_OPTR_BOR_LEV_1,
            _4 = FLASH_OPTR_BOR_LEV_2
        };

        using enum Level;

        static void set(Level a_level);
        static bool set(Level level_a, xmcu::Milliseconds timeout_a);
        static Level get();
    };

    struct RDP : private non_constructible
    {
        enum class Level : std::uint32_t
        {
            _0 = 0xAAu,
            _1 = 0xBBu,
            _2 = 0xCCu
        };

        using enum Level;

        static void set(Level level_a);
        static bool set(Level level_a, xmcu::Milliseconds timeout_a);
        static Level get();
    };

    static bool launch();
    static bool launch(xmcu::Milliseconds timeout_a);
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