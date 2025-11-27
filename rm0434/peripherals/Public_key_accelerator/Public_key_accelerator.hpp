#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// externals
#include <stm32wbxx.h>

// xmcu
#include <rm0434/rcc.hpp>
#include <soc/peripheral.hpp>
#include <soc/st/arm/IRQ_config.hpp>
#include <xmcu/Duration.hpp>
#include <xmcu/Non_copyable.hpp>
#include <xmcu/non_constructible.hpp>

namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals {
class Public_key_accelerator : private xmcu::Non_copyable
{
public:
    enum class Mode : std::uint32_t
    {
        montgomery_modular_exp = 0u,
        montgomery_param_only = PKA_CR_MODE_0,
        modular_exp_only = PKA_CR_MODE_1,
        ecc_scalar = PKA_CR_MODE_5,
        ecc_scalar_fast = PKA_CR_MODE_1 | PKA_CR_MODE_5,
        ecdsa_sign = PKA_CR_MODE_2 | PKA_CR_MODE_5,
        ecdsa_verify = PKA_CR_MODE_1 | PKA_CR_MODE_2 | PKA_CR_MODE_5,
        ecc_point_check = PKA_CR_MODE_3 | PKA_CR_MODE_5,
        rsa_crt_exp = PKA_CR_MODE_0 | PKA_CR_MODE_1 | PKA_CR_MODE_2,
        modular_inversion = PKA_CR_MODE_3,
        arithmetic_add = PKA_CR_MODE_0 | PKA_CR_MODE_3,
        arithmetic_sub = PKA_CR_MODE_1 | PKA_CR_MODE_3,
        arithmetic_mul = PKA_CR_MODE_0 | PKA_CR_MODE_1 | PKA_CR_MODE_3,
        arithmetic_comp = PKA_CR_MODE_2 | PKA_CR_MODE_3,
        modular_reduction = PKA_CR_MODE_0 | PKA_CR_MODE_2 | PKA_CR_MODE_3,
        modular_add = PKA_CR_MODE_1 | PKA_CR_MODE_2 | PKA_CR_MODE_3,
        modular_sub = PKA_CR_MODE_0 | PKA_CR_MODE_1 | PKA_CR_MODE_2 | PKA_CR_MODE_3,
        montgomery_mul_small = PKA_CR_MODE_4,
    };

    using enum Mode;

    template<Mode> struct Context;

    class Pooling : private Non_copyable
    {
    public:
        enum class Status
        {
            ok,
            busy,
            ram_err,
            addr_err,
            timeout,
        };

        template<Mode> struct Result;

        using enum Status;

        template<Mode mode> Result<mode> execute(const Context<mode>& a_ctx, Milliseconds a_timeout);

    private:
        Public_key_accelerator* p_pka;
        friend Public_key_accelerator;
    };

    class Interrupt : private Non_copyable
    {
    public:
        enum class Source
        {
            ram_err,
            addr_err,
            operation_end,
        };

        using enum Source;

        struct Callback
        {
            using Function = void (*)(Public_key_accelerator* a_p_this, Source a_source);

            Function function = nullptr;
            void* p_user_data = nullptr;
        };
        
        void enable(const IRQ_config& a_irq_config);
        void disable();
        template<Mode mode> void start(const Context<mode>& a_ctx, const Callback& a_callback);

    private:
        Public_key_accelerator* p_pka;
        friend Public_key_accelerator;
    };

    void enable();
    void disable();

    bool is_ecdsa_signature_valid();

    Pooling pooling;
    Interrupt interrupt;

private:
    explicit Public_key_accelerator(IRQn_Type a_irqn)
        : irqn(a_irqn)
    {
        this->pooling.p_pka = this;
        this->interrupt.p_pka = this;
    }

    IRQn_Type irqn;
    Interrupt::Callback callback;

    template<typename> friend class soc::peripheral;
    friend void PKA_interrupt_handler(Public_key_accelerator* a_p_this);
};

void PKA_interrupt_handler(Public_key_accelerator* a_p_this);

template<> struct Public_key_accelerator::Context<Public_key_accelerator::Mode::ecdsa_verify>
{
    std::uint32_t prime_order_size;
    std::uint32_t modulus_size;
    std::uint32_t coef_sign;
    const uint8_t* p_coef;
    const uint8_t* p_modulus;
    const uint8_t* p_base_point_x;
    const uint8_t* p_base_point_y;
    const uint8_t* p_prime_order;

    const uint8_t* p_pub_key_curve_pt_x;
    const uint8_t* p_pub_key_curve_pt_y;

    const uint8_t* p_r_sign;
    const uint8_t* p_s_sign;
    const uint8_t* p_hash;
};

template<> struct Public_key_accelerator::Pooling::Result<Public_key_accelerator::Mode::ecdsa_verify>
{
    Public_key_accelerator::Pooling::Status status = Status::busy;
    bool is_signature_valid = false;
};

} // namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals

namespace xmcu::soc::st::arm::m4::wb::rm0434 {
template<> class rcc<peripherals::Public_key_accelerator> : private xmcu::non_constructible
{
public:
    static void enable();
    static void disable();
};
} // namespace xmcu::soc::st::arm::m4::wb::rm0434

namespace xmcu::soc {
template<> class peripheral<st::arm::m4::wb::rm0434::peripherals::Public_key_accelerator> : private non_constructible
{
public:
    static st::arm::m4::wb::rm0434::peripherals::Public_key_accelerator create()
    {
        return st::arm::m4::wb::rm0434::peripherals::Public_key_accelerator(IRQn_Type::PKA_IRQn);
    }
};
} // namespace xmcu::soc
