#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>
#include <utility>

// externals
#include <stm32wbxx.h>

// xmcu
#include <rm0434/rcc.hpp>
#include <soc/peripheral.hpp>
#include <soc/st/arm/IRQ_config.hpp>
#include <soc/st/arm/m4/nvic.hpp>
#include <xmcu/Duration.hpp>
#include <xmcu/Non_copyable.hpp>
#include <xmcu/assertion.hpp>
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
        ~Interrupt()
        {
            if (true == this->is_enabled())
            {
                this->disable();
            }
        }

        enum class Source
        {
            ram_err,
            addr_err,
            operation_end,
        };

        using enum Source;

        template<Mode> struct Result;

        struct Callback_ecdsa_verify
        {
            using Function = void (*)(const Result<Mode::ecdsa_verify>& a_result);

            Function function = nullptr;
            void* p_user_data = nullptr;
        };

        template<Mode M> struct Callback_traits;

        void enable(const IRQ_config& a_irq_config);
        void disable();

        template<Mode mode> void start(const Context<mode>& a_ctx, const Callback_traits<mode>::Type& a_callback);

        bool is_enabled() const
        {
            return 0 != NVIC_GetEnableIRQ(this->p_pka->irqn);
        }

    private:
        Public_key_accelerator* p_pka;
        friend Public_key_accelerator;
    };

    ~Public_key_accelerator()
    {
        if (true == this->is_enabled())
        {
            this->disable();
        }
    }

    void enable();
    void disable();

    bool is_enabled() const;

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

    using Irq_dispatcher = void (*)(Public_key_accelerator* a_p_this,
                                    Interrupt::Source a_source,
                                    void* a_p_user_func,
                                    void* a_p_user_data);

    Irq_dispatcher irq_dispatcher = nullptr;
    void* user_func = nullptr;
    void* user_data = nullptr;

    void load(const Context<Public_key_accelerator::Mode::ecdsa_verify>& a_ctx) const;

    template<Mode mode> static void
    dispach_irq(Public_key_accelerator* a_p_this, Interrupt::Source a_source, void* a_p_user_func, void* a_p_user_data);

    template<Mode mode> void populate_result(Interrupt::Result<mode>& a_result, Interrupt::Source a_source);

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

template<> struct Public_key_accelerator::Interrupt::Result<Public_key_accelerator::Mode::ecdsa_verify>
{
    Public_key_accelerator::Interrupt::Source source = Source::operation_end;
    bool is_signature_valid = false;
};

template<> struct Public_key_accelerator::Interrupt::Callback_traits<Public_key_accelerator::Mode::ecdsa_verify>
{
    using Type = Public_key_accelerator::Interrupt::Callback_ecdsa_verify;
};

template<Public_key_accelerator::Mode mode>
void Public_key_accelerator::Interrupt::start(const Context<mode>& a_ctx,
                                              const typename Callback_traits<mode>::Type& a_callback)
{
    Scoped_guard<nvic> guard;

    this->p_pka->user_func = reinterpret_cast<void*>(a_callback.function);
    this->p_pka->user_data = a_callback.p_user_data;
    this->p_pka->irq_dispatcher = &Public_key_accelerator::dispach_irq<mode>;

    this->p_pka->load(a_ctx);

    bit::flag::set(&(PKA->CR), std::to_underlying(mode) | PKA_CR_PROCENDIE | PKA_CR_RAMERRIE | PKA_CR_ADDRERRIE);
    bit::flag::set(&(PKA->CR), PKA_CR_START);
}

template<Public_key_accelerator::Mode mode>
void Public_key_accelerator::dispach_irq(Public_key_accelerator* a_p_this,
                                         Public_key_accelerator::Interrupt::Source a_source,
                                         void* a_p_user_func,
                                         void* a_p_user_data)
{
    hkm_assert(nullptr != a_p_this);
    hkm_assert(nullptr != a_p_user_func);

    using CallbackType = typename Interrupt::Callback_traits<mode>::Type::Function;
    auto callback_fn = reinterpret_cast<CallbackType>(a_p_user_func);

    Interrupt::Result<mode> result {};

    a_p_this->populate_result<mode>(result, a_source);

    callback_fn(result);
}

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
