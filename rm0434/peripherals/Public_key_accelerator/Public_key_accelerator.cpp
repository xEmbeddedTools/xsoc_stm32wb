/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// this
#include <rm0434/peripherals/Public_key_accelerator/Public_key_accelerator.hpp>

// std
#include <cstring>
#include <utility>

// externals
#include <stm32wbxx.h>

// xmcu
#include <rm0434/utils/tick_counter.hpp>
#include <soc/Scoped_guard.hpp>
#include <xmcu/Duration.hpp>
#include <xmcu/assertion.hpp>
#include <xmcu/bit.hpp>

namespace {

using namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals;

Public_key_accelerator* pka_context = nullptr;

std::uint32_t get_optimal_bit_size(std::uint32_t a_byte_number, std::uint8_t a_msb)
{
    hkm_assert(0 != a_msb);

    std::uint32_t position = 32u - __CLZ(a_msb);

    return ((a_byte_number - 1u) * 8ul) + position;
}

void copy_to_pka_memory(volatile std::uint32_t* a_p_dst, const std::uint8_t* a_p_src, std::size_t a_size)
{
    hkm_assert(nullptr != a_p_dst);
    hkm_assert(nullptr != a_p_src);

    volatile std::uint32_t* dst_word = a_p_dst;

    std::size_t full_words = a_size / 4u;
    std::size_t remaining_bytes = a_size % 4u;

    if (remaining_bytes > 0u)
    {
        std::size_t head_bytes_size = remaining_bytes;

        std::uint32_t final_word = 0u;
        std::memcpy(&final_word, a_p_src, head_bytes_size);

        std::uint32_t swapped_word = __builtin_bswap32(final_word);
        std::uint32_t mask = (0xFFFFFFFFu << (8u * (4u - head_bytes_size)));

        *dst_word = swapped_word & mask;
        dst_word++;
    }

    const std::uint8_t* src_byte_current = a_p_src + a_size - 4u;

    for (std::size_t i = 0u; i < full_words; ++i)
    {
        std::uint32_t word;
        std::memcpy(&word, src_byte_current, sizeof(word));

        *dst_word = __builtin_bswap32(word);

        dst_word++;
        src_byte_current -= sizeof(word);
    }
}

void write_to_pka_memory(volatile std::uint32_t* a_p_dst, const std::uint8_t* a_p_src, std::size_t a_size)
{
    hkm_assert(nullptr != a_p_dst);
    hkm_assert(nullptr != a_p_src);

    copy_to_pka_memory(a_p_dst, a_p_src, a_size);
    a_p_dst[(a_size + 3) / 4] = 0u;
}

bool is_ecdsa_signature_valid()
{
    return 0u == PKA->RAM[PKA_ECDSA_VERIF_OUT_RESULT];
}

} // namespace

extern "C" {

void PKA_IRQHandler()
{
    hkm_assert(nullptr != pka_context);
    PKA_interrupt_handler(pka_context);
}

} // extern "C"

namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals {

using namespace utils;
using namespace xmcu;

void PKA_interrupt_handler(Public_key_accelerator* a_p_this)
{
    hkm_assert(nullptr != a_p_this);
    hkm_assert(nullptr != a_p_this->irq_dispatcher);

    Public_key_accelerator::Interrupt::Source source = Public_key_accelerator::Interrupt::operation_end;
    if (true == bit::flag::is(PKA->SR, PKA_SR_ADDRERRF))
    {
        source = Public_key_accelerator::Interrupt::addr_err;
    }
    else if (true == bit::flag::is(PKA->SR, PKA_SR_RAMERRF))
    {
        source = Public_key_accelerator::Interrupt::ram_err;
    }

    bit::flag::set(&(PKA->CLRFR), PKA_CLRFR_ADDRERRFC | PKA_CLRFR_RAMERRFC | PKA_CLRFR_PROCENDFC);

    a_p_this->irq_dispatcher(a_p_this, source, a_p_this->user_func, a_p_this->user_data);
}

void Public_key_accelerator::enable()
{
    PKA->CR = PKA_CR_EN;
    bit::flag::set(&(PKA->CLRFR), PKA_CLRFR_PROCENDFC | PKA_CLRFR_RAMERRFC | PKA_CLRFR_ADDRERRFC);
}

void Public_key_accelerator::disable()
{
    PKA->CR = 0u;
    bit::flag::set(&(PKA->CLRFR), PKA_CLRFR_PROCENDFC | PKA_CLRFR_RAMERRFC | PKA_CLRFR_ADDRERRFC);
}

bool Public_key_accelerator::is_enabled() const
{
    return PKA_CR_EN == bit::flag::get(PKA->CLRFR, PKA_CR_EN);
}

template<> Public_key_accelerator::Pooling::Result<Public_key_accelerator::ecdsa_verify>
Public_key_accelerator::Pooling::execute(const Context<Mode::ecdsa_verify>& a_ctx, Milliseconds a_timeout)
{
    const std::uint64_t end = tick_counter<Milliseconds>::get() + a_timeout.get();

    if (true == bit::flag::is(PKA->SR, PKA_SR_BUSY))
    {
        return { .status = Status::busy, .is_signature_valid = false };
    }

    this->p_pka->load_to_pka_ram(a_ctx);

    bit::flag::set(&(PKA->CR),
                   PKA_CR_MODE | PKA_CR_PROCENDIE | PKA_CR_RAMERRIE | PKA_CR_ADDRERRIE,
                   std::to_underlying(Mode::ecdsa_verify));

    bit::flag::set(&(PKA->CR), PKA_CR_START);

    Status status = Status::ok;

    bool finished = false;

    while (tick_counter<Milliseconds>::get() < end)
    {
        if (bit::flag::is(PKA->SR, PKA_SR_PROCENDF))
        {
            status = Status::ok;
            finished = true;
            break;
        }

        if (bit::flag::is(PKA->SR, PKA_SR_RAMERRF))
        {
            status = Status::ram_err;
            finished = true;
            break;
        }

        if (bit::flag::is(PKA->SR, PKA_SR_ADDRERRF))
        {
            status = Status::addr_err;
            finished = true;
            break;
        }
    }

    if (false == finished)
    {
        status = Status::timeout;

        bit::flag::clear(&(PKA->CR), PKA_CR_EN);
        bit::flag::set(&(PKA->CR), PKA_CR_EN);
    }

    Result<Mode::ecdsa_verify> result = { .status = status, .is_signature_valid = false };

    if (Pooling::Status::ok == status)
    {
        if (true == is_ecdsa_signature_valid())
        {
            result.is_signature_valid = true;
        }
    }

    bit::flag::set(&(PKA->CLRFR), PKA_CLRFR_PROCENDFC | PKA_CLRFR_RAMERRFC | PKA_CLRFR_ADDRERRFC);

    return result;
}

void Public_key_accelerator::load_to_pka_ram(
    const Public_key_accelerator::Context<Public_key_accelerator::Mode::ecdsa_verify>& a_ctx) const
{
    PKA->RAM[PKA_ECDSA_VERIF_IN_ORDER_NB_BITS] = get_optimal_bit_size(a_ctx.prime_order_size, *(a_ctx.p_prime_order));
    PKA->RAM[PKA_ECDSA_VERIF_IN_MOD_NB_BITS] = get_optimal_bit_size(a_ctx.modulus_size, *(a_ctx.p_modulus));
    PKA->RAM[PKA_ECDSA_VERIF_IN_A_COEFF_SIGN] = a_ctx.coef_sign;

    write_to_pka_memory(&PKA->RAM[PKA_ECDSA_VERIF_IN_A_COEFF], a_ctx.p_coef, a_ctx.modulus_size);
    write_to_pka_memory(&PKA->RAM[PKA_ECDSA_VERIF_IN_MOD_GF], a_ctx.p_modulus, a_ctx.modulus_size);
    write_to_pka_memory(&PKA->RAM[PKA_ECDSA_VERIF_IN_INITIAL_POINT_X], a_ctx.p_base_point_x, a_ctx.modulus_size);
    write_to_pka_memory(&PKA->RAM[PKA_ECDSA_VERIF_IN_INITIAL_POINT_Y], a_ctx.p_base_point_y, a_ctx.modulus_size);
    write_to_pka_memory(
        &PKA->RAM[PKA_ECDSA_VERIF_IN_PUBLIC_KEY_POINT_X], a_ctx.p_pub_key_curve_pt_x, a_ctx.modulus_size);
    write_to_pka_memory(
        &PKA->RAM[PKA_ECDSA_VERIF_IN_PUBLIC_KEY_POINT_Y], a_ctx.p_pub_key_curve_pt_y, a_ctx.modulus_size);
    write_to_pka_memory(&PKA->RAM[PKA_ECDSA_VERIF_IN_SIGNATURE_R], a_ctx.p_r_sign, a_ctx.prime_order_size);
    write_to_pka_memory(&PKA->RAM[PKA_ECDSA_VERIF_IN_SIGNATURE_S], a_ctx.p_s_sign, a_ctx.prime_order_size);
    write_to_pka_memory(&PKA->RAM[PKA_ECDSA_VERIF_IN_HASH_E], a_ctx.p_hash, a_ctx.prime_order_size);
    write_to_pka_memory(&PKA->RAM[PKA_ECDSA_VERIF_IN_ORDER_N], a_ctx.p_prime_order, a_ctx.prime_order_size);
}

void Public_key_accelerator::Interrupt::enable(const IRQ_config& a_irq_config)
{
    hkm_assert(nullptr == pka_context);

    pka_context = this->p_pka;

    NVIC_SetPriority(
        this->p_pka->irqn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq_config.preempt_priority, a_irq_config.sub_priority));
    NVIC_EnableIRQ(this->p_pka->irqn);
}

void Public_key_accelerator::Interrupt::disable()
{
    bit::flag::clear(&(PKA->CR), PKA_CR_PROCENDIE | PKA_CR_RAMERRIE | PKA_CR_ADDRERRIE);

    NVIC_DisableIRQ(this->p_pka->irqn);

    pka_context = nullptr;
}

template<>
void Public_key_accelerator::populate_result(Interrupt::Result<Public_key_accelerator::ecdsa_verify>& a_result,
                                             Interrupt::Source a_source)
{
    a_result.is_signature_valid = Interrupt::operation_end == a_source && true == is_ecdsa_signature_valid();
    a_result.source = a_source;
}

} // namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals

namespace xmcu::soc::st::arm::m4::wb::rm0434 {

void rcc<Public_key_accelerator>::enable()
{
    bit::flag::set(&(RCC->AHB3ENR), RCC_AHB3ENR_PKAEN);
    bit::flag::is(RCC->AHB3ENR, RCC_AHB3ENR_PKAEN);
}

void rcc<Public_key_accelerator>::disable()
{
    bit::flag::clear(&(RCC->AHB3ENR), RCC_AHB3ENR_PKAEN);
    bit::flag::is(RCC->AHB3ENR, RCC_AHB3ENR_PKAEN);
}

} // namespace xmcu::soc::st::arm::m4::wb::rm0434
