#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// external
#include <stm32wbxx.h>

// xmcu
#include <rm0434/rcc.hpp>
#include <rm0434/config.hpp>

#include <xmcu/Non_copyable.hpp>


namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals {

class AES : private xmcu::Non_copyable {
public:
#if defined(XMCU_AES1_PRESENT)
    enum class _1;
#endif
#if defined(XMCU_AES2_PRESENT)
    enum class _2;
#endif

private:
    SPI(std::uint32_t a_idx, AES_TypeDef* a_p_registers, IRQn_Type a_irqn)
        : idx(a_idx)
        , p_registers(a_p_registers)
        , irqn(a_irqn)
    {

    }

    std::uint32_t idx;
    AES_TypeDef* p_registers;
    IRQn_Type irqn;

    template<typename Periph_t, std::uint32_t id> friend class xmcu::soc::peripheral;
};

} // namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals

namespace xmcu::soc::st::arm::m4::wb::rm0434 {

#if defined(XMCU_AES1_PRESENT)
template<> class rcc<peripherals::AES, peripherals::AES::_1> : private xmcu::non_constructible
{
public:
    static void enable();
    static void disable();
};
#endif

#if defined(XMCU_AES2_PRESENT)
template<> class rcc<peripherals::AES, peripherals::AES::_2> : private xmcu::non_constructible
{
public:
    static void enable();
    static void disable();
};
#endif

} // namespace xmcu::soc::st::arm::m4::wb::rm0434

namespace xmcu::soc {

#if defined(XMCU_AES1_PRESENT)
template<> class peripheral<st::arm::m4::wb::rm0434::peripherals::AES, st::arm::m4::wb::rm0434::peripherals::AES::_1>
    : private non_constructible
{
public:
    static st::arm::m4::wb::rm0434::peripherals::AES create()
    {
        return st::arm::m4::wb::rm0434::peripherals::AES(0u, AES1, IRQn_Type::AES1_IRQn);
    }
};
#endif

#if defined(XMCU_AES2_PRESENT)
template<> class peripheral<st::arm::m4::wb::rm0434::peripherals::AES, st::arm::m4::wb::rm0434::peripherals::AES::_2>
    : private non_constructible
{
public:
    static st::arm::m4::wb::rm0434::peripherals::AES create()
    {
        return st::arm::m4::wb::rm0434::peripherals::AES(1u, AES2, IRQn_Type::AES2_IRQn);
    }
};
#endif
} // namespace xmcu::soc
