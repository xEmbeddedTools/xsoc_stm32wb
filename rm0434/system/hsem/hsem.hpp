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
#include <xmcu/Limited.hpp>
#include <xmcu/Non_copyable.hpp>
#include <xmcu/non_constructible.hpp>

namespace xmcu::soc::st::arm::m4::wb::rm0434::system {
class hsem : private non_constructible
{
public:
    enum class Id : std::uint8_t
    {
        rng = 0x0u,
        pka = 0x1u,
        flash = 0x2u,
        rcc = 0x3u,
        stop_entry = 0x4u,
        clk48_config = 0x5u
    };

    struct _1_step : private non_constructible
    {
        static void lock(Limited<std::uint8_t, 0, 31> a_semaphore_id);
        static bool try_lock(Limited<std::uint8_t, 0, 31> a_semaphore_id, Milliseconds a_timeout);
        static bool try_lock(Limited<std::uint8_t, 0, 31> a_semaphore_id);
        static void unlock(Limited<std::uint8_t, 0, 31> a_semaphore_id);

        static void lock(Id a_id)
        {
            return lock(static_cast<std::underlying_type<Id>::type>(a_id));
        }
        static bool try_lock(Id a_id, Milliseconds a_timeout)
        {
            return try_lock(static_cast<std::underlying_type<Id>::type>(a_id), a_timeout);
        }
        static bool try_lock(Id a_id)
        {
            return try_lock(static_cast<std::underlying_type<Id>::type>(a_id));
        }
        static void unlock(Id a_id)
        {
            return unlock(static_cast<std::underlying_type<Id>::type>(a_id));
        }
    };

    struct _2_step : private non_constructible
    {
        static void lock(Limited<std::uint8_t, 0, 31> a_semaphore_id, std::uint8_t a_process_id);
        static bool
        try_lock(Limited<std::uint8_t, 0, 31> a_semaphore_id, std::uint8_t a_process_id, Milliseconds a_timeout);
        static bool try_lock(Limited<std::uint8_t, 0, 31> a_semaphore_id, std::uint8_t a_process_id);
        static void unlock(Limited<std::uint8_t, 0, 31> a_semaphore_id, std::uint8_t a_process_id);

        static void lock(Id a_id, std::uint8_t a_process_id)
        {
            return lock(static_cast<std::underlying_type<Id>::type>(a_id), a_process_id);
        }
        static bool try_lock(Id a_id, std::uint8_t a_process_id, Milliseconds a_timeout)
        {
            return try_lock(static_cast<std::underlying_type<Id>::type>(a_id), a_process_id, a_timeout);
        }
        static bool try_lock(Id a_id, std::uint8_t a_process_id)
        {
            return try_lock(static_cast<std::underlying_type<Id>::type>(a_id), a_process_id);
        }
        static void unlock(Id a_id, std::uint8_t a_process_id)
        {
            return unlock(static_cast<std::underlying_type<Id>::type>(a_id), a_process_id);
        }
    };

    static bool is_locked(Limited<std::uint8_t, 0, 31> a_semaphore_id)
    {
        return 0x0u != HSEM->R[a_semaphore_id];
    }
    static bool is_locked(Id a_id)
    {
        return is_locked(static_cast<std::underlying_type<Id>::type>(a_id));
    }
};
} // namespace xmcu::soc::st::arm::m4::wb::rm0434::system

namespace xmcu::soc {
template<> class Scoped_guard<st::arm::m4::wb::rm0434::system::hsem::_1_step> : private Non_copyable
{
public:
    Scoped_guard(Limited<std::uint8_t, 0, 31> a_semaphore_id)
        : semaphore_id(a_semaphore_id)
    {
        st::arm::m4::wb::rm0434::system::hsem::_1_step::lock(a_semaphore_id);
        this->locked = true;
    }

    Scoped_guard(Limited<std::uint8_t, 0, 31> a_semaphore_id, Milliseconds a_timeout)
        : semaphore_id(a_semaphore_id)
    {
        this->locked = st::arm::m4::wb::rm0434::system::hsem::_1_step::try_lock(a_semaphore_id, a_timeout);
    }

    ~Scoped_guard()
    {
        st::arm::m4::wb::rm0434::system::hsem::_1_step::unlock(this->semaphore_id);
    }

    bool is_locked() const
    {
        return this->locked;
    }

private:
    std::uint8_t semaphore_id;
    bool locked;
};

template<> class Scoped_guard<st::arm::m4::wb::rm0434::system::hsem::_2_step> : private Non_copyable
{
public:
    Scoped_guard(Limited<std::uint8_t, 0, 31> a_semaphore_id, std::uint8_t a_process_id)
        : semaphore_id(a_semaphore_id)
        , process_id(a_process_id)
        , locked(false)
    {
        st::arm::m4::wb::rm0434::system::hsem::_2_step::lock(a_semaphore_id, a_process_id);
        this->locked = true;
    }

    Scoped_guard(Limited<std::uint8_t, 0, 31> a_semaphore_id, std::uint8_t a_process_id, Milliseconds a_timeout)
        : semaphore_id(a_semaphore_id)
        , process_id(a_process_id)
        , locked(st::arm::m4::wb::rm0434::system::hsem::_2_step::try_lock(a_semaphore_id, a_process_id, a_timeout))
    {
    }

    ~Scoped_guard()
    {
        st::arm::m4::wb::rm0434::system::hsem::_2_step::unlock(this->semaphore_id, this->process_id);
    }

    bool is_locked() const
    {
        return this->locked;
    }

private:
    std::uint8_t semaphore_id;
    std::uint8_t process_id;
    bool locked;
};
} // namespace xmcu::soc