/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// this
#include <rm0434/peripherals/pka/pka.hpp>

// externals
#include <stm32wbxx.h>

// xmcu
#include <rm0434/utils/tick_counter.hpp>
#include <soc/Scoped_guard.hpp>
#include <soc/st/arm/m4/nvic.hpp>
#include <xmcu/Duration.hpp>
#include <xmcu/bit.hpp>

using namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals;

static pka* pka_context = nullptr;

namespace {

std::uint8_t clz(std::uint8_t a_value)
{
    if (0 == a_value)
    {
        return 32;
    }
    return __builtin_clz(a_value);
}

std::uint32_t get_optimal_bit_size(std::uint32_t a_byte_number, std::uint8_t a_msb)
{
    std::uint32_t position = 32ul - clz(a_msb);

    return ((a_byte_number - 1ul) * 8ul) + position;
}

void copy_to_pka_memory(volatile std::uint32_t* dst, const std::uint8_t* src, std::size_t n)
{
    if (nullptr == dst || nullptr == src)
    {
        return;
    }

    std::size_t index = 0u;

    for (; index < (n / 4u); ++index)
    {
        dst[index] = static_cast<std::uint32_t>(src[n - index * 4u - 1u]) |
                     static_cast<std::uint32_t>(src[n - index * 4u - 2u]) << 8u |
                     static_cast<std::uint32_t>(src[n - index * 4u - 3u]) << 16u |
                     static_cast<std::uint32_t>(src[n - index * 4u - 4u]) << 24u;
    }

    if (1u == n % 4u)
    {
        dst[index] = static_cast<std::uint32_t>(src[n - (index * 4u) - 1u]);
    }
    else if (2u == n % 4u)
    {
        dst[index] = static_cast<std::uint32_t>(src[n - index * 4u - 1u]) |
                     static_cast<std::uint32_t>(src[n - index * 4u - 2u]) << 8u;
    }
    else if (3u == n % 4u)
    {
        dst[index] = static_cast<std::uint32_t>(src[n - index * 4u - 1u]) |
                     static_cast<std::uint32_t>(src[n - index * 4u - 2u]) << 8u |
                     static_cast<std::uint32_t>(src[n - index * 4u - 3u]) << 16u;
    }
}

} // namespace

extern "C" {
using namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals;

void PKA_IRQHandler()
{
    hkm_assert(nullptr != pka_context);
    PKA_interrupt_handler(pka_context);
}

} // extern "C"

namespace xmcu::soc::st::arm::m4::wb::rm0434 {

using namespace utils;
using namespace xmcu;

namespace peripherals {

void PKA_interrupt_handler(pka* a_p_this)
{
    hkm_assert(nullptr != a_p_this);

    pka::Interrupt::Source source = pka::Interrupt::operation_end;
    if (true == bit::flag::is(PKA->SR, PKA_SR_ADDRERRF))
    {
        source = pka::Interrupt::addr_err;
    }
    else if (true == bit::flag::is(PKA->SR, PKA_SR_RAMERRF))
    {
        source = pka::Interrupt::ram_err;
    }

    a_p_this->callback.function(a_p_this, source);

    bit::flag::set(&(PKA->CLRFR), PKA_CLRFR_ADDRERRFC | PKA_CLRFR_RAMERRFC | PKA_CLRFR_PROCENDFC);
}

void pka::enable()
{
    PKA->CR = PKA_CR_EN;
    bit::flag::set(&(PKA->CLRFR), PKA_CLRFR_PROCENDFC | PKA_CLRFR_RAMERRFC | PKA_CLRFR_ADDRERRFC);
}

void pka::disable()
{
    PKA->CR = 0u;
    bit::flag::set(&(PKA->CLRFR), PKA_CLRFR_PROCENDFC | PKA_CLRFR_RAMERRFC | PKA_CLRFR_ADDRERRFC);
}

bool pka::load_to_ram(const Ecdsa_verify_params& a_params)
{
    PKA->RAM[PKA_ECDSA_VERIF_IN_ORDER_NB_BITS] =
        get_optimal_bit_size(a_params.prime_order_size, *(a_params.p_prime_order));
    PKA->RAM[PKA_ECDSA_VERIF_IN_MOD_NB_BITS] = get_optimal_bit_size(a_params.modulus_size, *(a_params.p_modulus));
    PKA->RAM[PKA_ECDSA_VERIF_IN_A_COEFF_SIGN] = a_params.coef_sign;

    auto copy_and_end = [](volatile std::uint32_t* dst, const std::uint8_t* src, std::size_t size) {
        copy_to_pka_memory(dst, src, size);
        dst[(size + 3) / 4] = 0u;
    };

    copy_and_end(&PKA->RAM[PKA_ECDSA_VERIF_IN_A_COEFF], a_params.p_coef, a_params.modulus_size);
    copy_and_end(&PKA->RAM[PKA_ECDSA_VERIF_IN_MOD_GF], a_params.p_modulus, a_params.modulus_size);
    copy_and_end(&PKA->RAM[PKA_ECDSA_VERIF_IN_INITIAL_POINT_X], a_params.p_base_point_x, a_params.modulus_size);
    copy_and_end(&PKA->RAM[PKA_ECDSA_VERIF_IN_INITIAL_POINT_Y], a_params.p_base_point_y, a_params.modulus_size);
    copy_and_end(
        &PKA->RAM[PKA_ECDSA_VERIF_IN_PUBLIC_KEY_POINT_X], a_params.p_pub_key_curve_pt_x, a_params.modulus_size);
    copy_and_end(
        &PKA->RAM[PKA_ECDSA_VERIF_IN_PUBLIC_KEY_POINT_Y], a_params.p_pub_key_curve_pt_y, a_params.modulus_size);
    copy_and_end(&PKA->RAM[PKA_ECDSA_VERIF_IN_SIGNATURE_R], a_params.p_r_sign, a_params.prime_order_size);
    copy_and_end(&PKA->RAM[PKA_ECDSA_VERIF_IN_SIGNATURE_S], a_params.p_s_sign, a_params.prime_order_size);
    copy_and_end(&PKA->RAM[PKA_ECDSA_VERIF_IN_HASH_E], a_params.p_hash, a_params.prime_order_size);
    copy_and_end(&PKA->RAM[PKA_ECDSA_VERIF_IN_ORDER_N], a_params.p_prime_order, a_params.prime_order_size);

    return true;
}

pka::Pooling::Operation_result pka::Pooling::start(Mode a_mode, Milliseconds a_timeout)
{
    if (true == bit::flag::is(PKA->SR, PKA_SR_BUSY)) {
        return Operation_result::busy;
    }

    const std::uint64_t end = tick_counter<Milliseconds>::get() + a_timeout.get();

    bit::flag::set(&(PKA->CR),
                   PKA_CR_MODE | PKA_CR_PROCENDIE | PKA_CR_RAMERRIE | PKA_CR_ADDRERRIE,
                   static_cast<std::uint32_t>(a_mode));

    bit::flag::set(&(PKA->CR), PKA_CR_START);

    Operation_result result = Operation_result::operation_end;

    bool finished = false;

    while (tick_counter<Milliseconds>::get() < end)
    {
        if (bit::flag::is(PKA->SR, PKA_SR_PROCENDF))
        {
            result = Operation_result::operation_end;
            finished = true;
            break;
        }

        if (bit::flag::is(PKA->SR, PKA_SR_RAMERRF))
        {
            result = Operation_result::ram_err;
            finished = true;
            break;
        }

        if (bit::flag::is(PKA->SR, PKA_SR_ADDRERRF))
        {
            result = Operation_result::addr_err;
            finished = true;
            break;
        }
    }

    if (false == finished)
    {
        result = Operation_result::timeout;

        bit::flag::clear(&(PKA->CR), PKA_CR_EN);
        bit::flag::set(&(PKA->CR), PKA_CR_EN);
    }

    if (Operation_result::operation_end == result)
    {
        if (a_mode == Mode::ecdsa_sign && PKA->RAM[PKA_ECDSA_SIGN_OUT_ERROR] != 0u)
        {
            result = Operation_result::ecdsa_sign_out_err;
        }
    }

    bit::flag::set(&(PKA->CLRFR), PKA_CLRFR_PROCENDFC | PKA_CLRFR_RAMERRFC | PKA_CLRFR_ADDRERRFC);

    return result;
}

bool pka::is_ecdsa_signature_valid()
{
    return 0u == PKA->RAM[PKA_ECDSA_VERIF_OUT_RESULT];
}

void pka::Interrupt::enable(const IRQ_config& a_irq_config)
{
    hkm_assert(nullptr == pka_context);

    pka_context = this->p_pka;

    NVIC_SetPriority(
        this->p_pka->irqn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq_config.preempt_priority, a_irq_config.sub_priority));
    NVIC_EnableIRQ(this->p_pka->irqn);
}

void pka::Interrupt::disable()
{
    // TODO: stop processing ?
    // bit::flag::clear(&(PKA->CR), PKA_CR_PROCENDIE | PKA_CR_RAMERRIE | PKA_CR_ADDRERRIE);
    NVIC_DisableIRQ(this->p_pka->irqn);

    pka_context = nullptr;
}

void pka::Interrupt::start(Mode a_mode, const Callback& a_callback)
{
    hkm_assert(nullptr != a_callback.function);

    Scoped_guard<nvic> guard;
    this->p_pka->callback = a_callback;

    bit::flag::set(&(PKA->CR),
                   static_cast<std::uint32_t>(a_mode) | PKA_CR_PROCENDIE | PKA_CR_RAMERRIE | PKA_CR_ADDRERRIE);
    bit::flag::set(&(PKA->CR), PKA_CR_START);
}

} // namespace peripherals

void rcc<pka>::enable()
{
    bit::flag::set(&(RCC->AHB3ENR), RCC_AHB3ENR_PKAEN);
    bit::flag::is(RCC->AHB3ENR, RCC_AHB3ENR_PKAEN);
}

void rcc<pka>::disable()
{
    bit::flag::clear(&(RCC->AHB3ENR), RCC_AHB3ENR_PKAEN);
    bit::flag::is(RCC->AHB3ENR, RCC_AHB3ENR_PKAEN);
}

} // namespace xmcu::soc::st::arm::m4::wb::rm0434
