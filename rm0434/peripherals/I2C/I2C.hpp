#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// std
#include <array>
#include <cstddef>

// external
#include <stm32wbxx.h>

// xmcu
#include <rm0434/clocks/pclk.hpp>
#include <rm0434/clocks/sources/hsi16.hpp>
#include <rm0434/clocks/sysclk.hpp>
#include <rm0434/config.hpp>
#include <rm0434/peripherals/GPIO/GPIO.hpp>
#include <rm0434/rcc.hpp>
#include <rm0434/system/mcu/mcu.hpp>
#include <xmcu/Non_copyable.hpp>
#include <xmcu/Not_null.hpp>

namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals {
class I2C : private xmcu::Non_copyable
{
public:
#if defined(XMCU_I2C1_PRESENT)
    enum class _1;
#endif
#if defined(XMCU_I2C3_PRESENT)
    enum class _3;
#endif
    enum class Transfer_result
    {
        ok,
        nack,
        arbitration_lost,
    };

    using Address = std::uint32_t;
    struct Header
    {
        std::uint32_t adr;
        // const std::uint8_t *control;
        bool auto_end = true;
    };

    /// is also possible xD
    class PollingSlave; // : private xmcu::Non_copyable
    class Polling : private xmcu::Non_copyable
    {
    public:
        Transfer_result transmit(Address a_adr, Not_null<const void*> a_p_data, std::size_t a_size, bool a_is_auto_end);

        Transfer_result receive(Address a_adr, Not_null<void*> a_p_data, std::size_t a_size, bool a_is_auto_end);

        Transfer_result transmit(Address a_adr, const std::initializer_list<std::uint8_t>& a_list, bool a_is_auto_end)
        {
            return this->transmit(a_adr, a_list.begin(), a_list.size(), a_is_auto_end);
        }

        template<typename T> Transfer_result transmit(Address a_adr, const T& a_data, bool a_is_auto_end)
        {
            return this->transmit(a_adr, &a_data, sizeof(T), a_is_auto_end);
        }

        template<typename T> Transfer_result receive(Address a_adr, T* a_data, bool a_is_auto_end)
        {
            return this->receive(a_adr, a_data, sizeof(T), a_is_auto_end);
        }

        // because default argument (a_is_auto_end) is to ambiguous . . .
        template<typename T> Transfer_result transmit(Address a_adr, const T& a_data, std::size_t a_size)
        {
            return transmit(a_adr, a_data, a_size, false);
        }

        template<typename T> Transfer_result receive(Address a_adr, T* a_data, std::size_t a_size)
        {
            return receive(a_adr, a_data, a_size, false);
        }

    private:
        Polling(I2C_TypeDef* a_p_registers);
        I2C_TypeDef* p_registers;
        friend class I2C;
    };

    I2C(I2C_TypeDef* a_p_registers, std::uint32_t (*a_get_clk)())
        : polling { a_p_registers }
        , p_registers { a_p_registers }
        , get_clk { a_get_clk }
    {
    }
    bool enable();

    template<std::size_t t_data_size>
    Transfer_result transmit(const Header& a_adr, const std::array<std::uint8_t, t_data_size>& a_data)
    {
        return this->polling.transmit(a_adr.adr, a_data, a_adr.auto_end);
    }

    Polling polling;

protected:
    I2C_TypeDef* p_registers;
    std::uint32_t (*get_clk)();
};
} // namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals

namespace xmcu::soc::st::arm::m4::wb::rm0434 {
#if defined(XMCU_I2C1_PRESENT)
template<> class rcc<peripherals::I2C, peripherals::I2C::_1> : private xmcu::non_constructible
{
public:
    template<typename Source_t> static void enable(bool a_enable_in_lp) = delete;
    static void disable();
    static std::uint32_t get_frequency_Hz();
};

template<> void rcc<peripherals::I2C, peripherals::I2C::_1>::enable<clocks::pclk<1u>>(bool a_enable_in_lp);
template<> void rcc<peripherals::I2C, peripherals::I2C::_1>::enable<clocks::sysclk<1u>>(bool a_enable_in_lp);
template<> void rcc<peripherals::I2C, peripherals::I2C::_1>::enable<clocks::sources::hsi16>(bool a_enable_in_lp);
#endif

#if defined(XMCU_I2C3_PRESENT)
template<> class rcc<peripherals::I2C, peripherals::I2C::_3> : private xmcu::non_constructible
{
public:
    template<typename Source_t> static void enable(bool a_enable_in_lp) = delete;
    static void disable();
    static std::uint32_t get_frequency_Hz();
};

template<> void rcc<peripherals::I2C, peripherals::I2C::_3>::enable<clocks::pclk<1u>>(bool a_enable_in_lp);
template<> void rcc<peripherals::I2C, peripherals::I2C::_3>::enable<clocks::sysclk<1u>>(bool a_enable_in_lp);
template<> void rcc<peripherals::I2C, peripherals::I2C::_3>::enable<clocks::sources::hsi16>(bool a_enable_in_lp);
#endif
} // namespace xmcu::soc::st::arm::m4::wb::rm0434

namespace xmcu::soc::st::arm::m4::wb::rm0434 {
template<>
inline void peripherals::GPIO::Alternate_function::enable<peripherals::I2C, 1u>(Limited<std::uint32_t, 0, 15> a_id,
                                                                                const Enable_config& a_config,
                                                                                Pin* a_p_pin)
{
    hkm_assert(Type::open_drain == a_config.type);
    hkm_assert(1 == this->p_port->idx && (8u == a_id || 9u == a_id));
    this->enable(a_id, a_config, 4u, a_p_pin);
}
} // namespace xmcu::soc::st::arm::m4::wb::rm0434

namespace xmcu::soc {
template<> class peripheral<st::arm::m4::wb::rm0434::peripherals::I2C, st::arm::m4::wb::rm0434::peripherals::I2C::_1>
    : private xmcu::non_constructible
{
public:
    static st::arm::m4::wb::rm0434::peripherals::I2C create()
    {
        std::uint32_t (*fun)() =
            st::arm::m4::wb::rm0434::rcc<st::arm::m4::wb::rm0434::peripherals::I2C,
                                         st::arm::m4::wb::rm0434::peripherals::I2C::_1>::get_frequency_Hz;
        return st::arm::m4::wb::rm0434::peripherals::I2C(I2C1, fun);
    }
};
} // namespace xmcu::soc