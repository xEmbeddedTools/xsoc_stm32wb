#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

#include <rm0434/clocks/sources/hse.hpp>
#include <rm0434/clocks/sources/lse.hpp>
#include <rm0434/clocks/sources/lsi.hpp>
#include <rm0434/rcc.hpp>
#include <soc/Scoped_guard.hpp>
#include <xmcu/Duration.hpp>
#include <xmcu/Non_copyable.hpp>
#include <xmcu/bit.hpp>
#include <xmcu/time_utils.hpp>

namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals {
class backup_domain : non_constructible
{
};

class rtc : non_constructible
{
public:
    enum class Alarm_id
    {
        A,
        B
    };

    enum Alarm_mask : std::uint32_t
    {
        seconds = 1 << 0,
        minutes = 1 << 1,
        hours = 1 << 2,
        days = 1 << 3,
    };

    struct Alarm_handler
    {
        using Function = void (*)(Alarm_id a_id, void* a_data);
        Function function;
        void* p_data;
    };

    /**
     * @brief Sets the RTC time to a given time, in milliseconds.
     *
     * The time will be set to a rounded-down second precision.
     *
     * @param a_world_millis Time to be set, in milliseconds.
     */
    static void set_clock(Milliseconds a_world_millis);

    /*
    * @brief Invalidates the RTC time.
    */
    static void unset_clock();

    /**
     * @brief Checks if RTC is set up.
     *
     * Useful to check if RTC holds configuration from before CPU1 reboot.
     *
     * @return True if RTC is already configured, otherwise false.
     */
    static bool is_clock_set();

    /**
     * @brief Reads the RTC time in a synchronized way with RTC's 1Hz counter.
     *
     * This is not to be used near sleep/stop modes.
     *
     * @return RTC time, rounded down to a second.
     */
    static time_utils::Timestamp get_time();

    static uint32_t read_bkp_register(std::size_t a_index);

    static void write_bkp_register(std::size_t a_index, uint32_t a_value);

    static void enable_alarm(Alarm_id a_id, Alarm_mask a_mask, Milliseconds a_world_millis, Alarm_handler a_handler);

    static void disable_alarm(Alarm_id a_id);

    static bool is_alarm_triggered(Alarm_id a_id);

private:
    static void wait_for_sync();
};

constexpr rtc::Alarm_mask operator|(rtc::Alarm_mask a_f1, rtc::Alarm_mask a_f2)
{
    return static_cast<rtc::Alarm_mask>(static_cast<std::uint32_t>(a_f1) | static_cast<std::uint32_t>(a_f2));
}

} // namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals

namespace xmcu::soc::st::arm::m4::wb::rm0434 {
template<> class rcc<peripherals::rtc> : private non_constructible
{
public:
    template<typename Source_t> static void enable(bool a_enable_in_lp) = delete;

    static void disable();

    static bool is_enabled();
};

template<> void rcc<peripherals::rtc>::enable<clocks::sources::lsi>(bool a_enable_in_lp);
template<> void rcc<peripherals::rtc>::enable<clocks::sources::lse>(bool a_enable_in_lp);
template<> void rcc<peripherals::rtc>::enable<clocks::sources::hse>(bool a_enable_in_lp);
} // namespace xmcu::soc::st::arm::m4::wb::rm0434

namespace xmcu::soc {
template<> class Scoped_guard<st::arm::m4::wb::rm0434::peripherals::backup_domain> : public Non_copyable
{
public:
    Scoped_guard()
    {
        // Disable write protection of backup domain peripherals
        bit::flag::set(&PWR->CR1, PWR_CR1_DBP);
    }

    ~Scoped_guard()
    {
        // Re-enable write-protection of backup domain peripherals
        bit::flag::clear(&PWR->CR1, PWR_CR1_DBP);
    }
};

template<> class Scoped_guard<st::arm::m4::wb::rm0434::peripherals::rtc> : private Non_copyable
{
public:
    Scoped_guard()
    {
        // Disable write protection of RTC registers
        RTC->WPR = 0xCA;
        RTC->WPR = 0x53;
    }

    ~Scoped_guard()
    {
        // Re-enable write-protection of RTC registers
        RTC->WPR = 0xFF;
    }

private:
    Scoped_guard<st::arm::m4::wb::rm0434::peripherals::backup_domain>
        bd_guard; // Must be unlocked prior to unlocking RTC
};
} // namespace xmcu::soc