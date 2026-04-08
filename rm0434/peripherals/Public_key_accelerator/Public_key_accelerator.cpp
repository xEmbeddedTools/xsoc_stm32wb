/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// this
#include <rm0434/peripherals/Public_key_accelerator/Public_key_accelerator.hpp>

// std
#include <climits>
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
using namespace xmcu::soc::st::arm::m4::wb::rm0434::utils;
using namespace xmcu;

constexpr std::size_t bytes_in_word = sizeof(std::uint32_t);
constexpr std::size_t bits_in_word = bytes_in_word * CHAR_BIT;
 
constexpr std::size_t pka_word_mask = 0xFFFFFFFFu;
constexpr std::size_t rsa_modulus_multiplier = 2u;

Public_key_accelerator* pka_context = nullptr;

std::uint32_t get_optimal_bit_size(std::uint32_t a_byte_number, std::uint8_t a_msb)
{
    hkm_assert(0 != a_msb);

    std::uint32_t position = bits_in_word - __CLZ(a_msb);

    return ((a_byte_number - 1u) * CHAR_BIT) + position;
}

void write_to_pka_memory(volatile std::uint32_t* a_p_dst, const std::uint8_t* a_p_src, std::size_t a_size)
{
    hkm_assert(nullptr != a_p_dst);
    hkm_assert(nullptr != a_p_src);

    volatile std::uint32_t* dst_word = a_p_dst;

    const std::size_t full_words = a_size / bytes_in_word;
    const std::size_t remaining_bytes = a_size % bytes_in_word;

    if (remaining_bytes > 0u)
    {
        std::size_t head_bytes_size = remaining_bytes;

        std::uint32_t final_word = 0u;
        std::memcpy(&final_word, a_p_src, head_bytes_size);

        std::uint32_t swapped_word = __builtin_bswap32(final_word);
        std::uint32_t mask = (pka_word_mask << (CHAR_BIT * (bytes_in_word - head_bytes_size)));

        *dst_word = swapped_word & mask;
        dst_word++;
    }

    const std::uint8_t* src_byte_current = a_p_src + a_size - bytes_in_word;

    for (std::size_t i = 0u; i < full_words; ++i)
    {
        std::uint32_t word;
        std::memcpy(&word, src_byte_current, sizeof(word));

        *dst_word = __builtin_bswap32(word);

        dst_word++;
        src_byte_current -= sizeof(word);
    }

    a_p_dst[(a_size + 3) / bytes_in_word] = 0u;
}

void pka_write(std::size_t a_offset, const std::uint8_t* a_p_data, std::size_t a_size)
{
    write_to_pka_memory(&PKA->RAM[a_offset], a_p_data, a_size);
}

void read_from_pka_memory(std::uint8_t* a_p_dst, const volatile std::uint32_t* a_p_src, std::size_t a_size)
{
    hkm_assert(nullptr != a_p_dst);
    hkm_assert(nullptr != a_p_src);

    std::size_t full_words = a_size / bytes_in_word;
    std::size_t remaining_bytes = a_size % bytes_in_word;

    const volatile std::uint32_t* src_word = a_p_src;
    std::uint8_t* dst_byte_current = a_p_dst + a_size - bytes_in_word;

    for (std::size_t i = 0u; i < full_words; ++i)
    {
        std::uint32_t word = __builtin_bswap32(*src_word);
        std::memcpy(dst_byte_current, &word, sizeof(word));

        src_word++;
        dst_byte_current -= sizeof(word);
    }

    if (remaining_bytes > 0u)
    {
        std::uint32_t word = __builtin_bswap32(*src_word);
        std::uint8_t* p_word_bytes = reinterpret_cast<std::uint8_t*>(&word);
        std::memcpy(a_p_dst, p_word_bytes + (bytes_in_word - remaining_bytes), remaining_bytes);
    }
}

bool is_ecdsa_signature_valid()
{
    return 0u == PKA->RAM[PKA_ECDSA_VERIF_OUT_RESULT];
}

void reset_pka()
{
    bit::flag::clear(&(PKA->CR), PKA_CR_EN);
    bit::flag::set(&(PKA->CR), PKA_CR_EN);
}

Public_key_accelerator::Pooling::Status wait_for_pka_completion(xmcu::Milliseconds a_timeout)
{
    const std::uint64_t end = tick_counter<xmcu::Milliseconds>::get() + a_timeout.get();

    while (tick_counter<xmcu::Milliseconds>::get() < end)
    {
        if (bit::flag::is(PKA->SR, PKA_SR_PROCENDF))
        {
            return Public_key_accelerator::Pooling::Status::ok;
        }

        if (bit::flag::is(PKA->SR, PKA_SR_RAMERRF))
        {
            return Public_key_accelerator::Pooling::Status::ram_err;
        }

        if (bit::flag::is(PKA->SR, PKA_SR_ADDRERRF))
        {
            return Public_key_accelerator::Pooling::Status::addr_err;
        }
    }

    // Timeout
    reset_pka();
    return Public_key_accelerator::Pooling::Status::timeout;
}

void wipe_pka_memory(volatile std::uint32_t* a_p_dst, std::size_t a_size_bytes)
{
    hkm_assert(nullptr != a_p_dst);
    std::size_t words = (a_size_bytes + 3u) / bytes_in_word;
    for (std::size_t i = 0u; i < words; ++i)
    {
        a_p_dst[i] = 0u;
    }
}

void clear_pka_flags()
{
    bit::flag::set(&(PKA->CLRFR), PKA_CLRFR_PROCENDFC | PKA_CLRFR_RAMERRFC | PKA_CLRFR_ADDRERRFC);
}

void start_operation(Public_key_accelerator::Mode a_mode)
{
    bit::flag::set(&(PKA->CR), PKA_CR_MODE, std::to_underlying(a_mode));
    bit::flag::set(&(PKA->CR), PKA_CR_START);
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

void Public_key_accelerator::enable()
{
    PKA->CR = PKA_CR_EN;
    clear_pka_flags();
}

void Public_key_accelerator::disable()
{
    PKA->CR = 0u;
    clear_pka_flags();
}

bool Public_key_accelerator::is_enabled() const
{
    return bit::flag::is(PKA->CR, PKA_CR_EN);
}

// --------------------------------------------------------------------------------------------------------------------
// ECDSA VERIFY
// --------------------------------------------------------------------------------------------------------------------
template<> Public_key_accelerator::Pooling::Result<Public_key_accelerator::ecdsa_verify>
Public_key_accelerator::Pooling::execute(const Context<Mode::ecdsa_verify>& a_ctx, Milliseconds a_timeout)
{
    if (true == bit::flag::is(PKA->SR, PKA_SR_BUSY))
    {
        return { .status = Status::busy, .is_signature_valid = false };
    }

    this->p_pka->load(a_ctx);

    start_operation(Mode::ecdsa_verify);

    Status status = wait_for_pka_completion(a_timeout);

    clear_pka_flags();

    Result<Mode::ecdsa_verify> result = { .status = status, .is_signature_valid = false };

    if (Pooling::Status::ok == status)
    {
        if (true == is_ecdsa_signature_valid())
        {
            result.is_signature_valid = true;
        }
    }

    return result;
}

void Public_key_accelerator::load(
    const Public_key_accelerator::Context<Public_key_accelerator::Mode::ecdsa_verify>& a_ctx) const
{
    PKA->RAM[PKA_ECDSA_VERIF_IN_ORDER_NB_BITS] = get_optimal_bit_size(a_ctx.prime_order_size, *(a_ctx.p_prime_order));
    PKA->RAM[PKA_ECDSA_VERIF_IN_MOD_NB_BITS] = get_optimal_bit_size(a_ctx.modulus_size, *(a_ctx.p_modulus));
    PKA->RAM[PKA_ECDSA_VERIF_IN_A_COEFF_SIGN] = a_ctx.coef_sign;

    pka_write(PKA_ECDSA_VERIF_IN_A_COEFF, a_ctx.p_coef, a_ctx.modulus_size);
    pka_write(PKA_ECDSA_VERIF_IN_MOD_GF, a_ctx.p_modulus, a_ctx.modulus_size);
    pka_write(PKA_ECDSA_VERIF_IN_INITIAL_POINT_X, a_ctx.p_base_point_x, a_ctx.modulus_size);
    pka_write(PKA_ECDSA_VERIF_IN_INITIAL_POINT_Y, a_ctx.p_base_point_y, a_ctx.modulus_size);
    pka_write(PKA_ECDSA_VERIF_IN_PUBLIC_KEY_POINT_X, a_ctx.p_pub_key_curve_pt_x, a_ctx.modulus_size);
    pka_write(PKA_ECDSA_VERIF_IN_PUBLIC_KEY_POINT_Y, a_ctx.p_pub_key_curve_pt_y, a_ctx.modulus_size);
    pka_write(PKA_ECDSA_VERIF_IN_SIGNATURE_R, a_ctx.p_r_sign, a_ctx.prime_order_size);
    pka_write(PKA_ECDSA_VERIF_IN_SIGNATURE_S, a_ctx.p_s_sign, a_ctx.prime_order_size);
    pka_write(PKA_ECDSA_VERIF_IN_HASH_E, a_ctx.p_hash, a_ctx.prime_order_size);
    pka_write(PKA_ECDSA_VERIF_IN_ORDER_N, a_ctx.p_prime_order, a_ctx.prime_order_size);
}

template<>
void Public_key_accelerator::populate_result(Interrupt::Result<Public_key_accelerator::ecdsa_verify>& a_result,
                                             Interrupt::Source a_source)
{
    a_result.is_signature_valid = Interrupt::operation_end == a_source && true == is_ecdsa_signature_valid();
    a_result.source = a_source;
}

// --------------------------------------------------------------------------------------------------------------------
// RSA CRT EXPONENTIATION
// --------------------------------------------------------------------------------------------------------------------
template<> Public_key_accelerator::Pooling::Result<Public_key_accelerator::Mode::rsa_crt_exp>
Public_key_accelerator::Pooling::execute(const Context<Mode::rsa_crt_exp>& a_ctx, Milliseconds a_timeout)
{
    if (true == bit::flag::is(PKA->SR, PKA_SR_BUSY))
    {
        return { .status = Status::busy };
    }

    this->p_pka->load(a_ctx);

    start_operation(Mode::rsa_crt_exp);

    Status status = wait_for_pka_completion(a_timeout);

    clear_pka_flags();

    if (Pooling::Status::ok == status)
    {
        this->p_pka->read(a_ctx);
    }

    return { .status = status };
}

void Public_key_accelerator::load(
    const Public_key_accelerator::Context<Public_key_accelerator::Mode::rsa_crt_exp>& a_ctx) const
{
    PKA->RAM[PKA_RSA_CRT_EXP_IN_MOD_NB_BITS] = a_ctx.modulus_size_bits;

    pka_write(PKA_RSA_CRT_EXP_IN_DP_CRT, a_ctx.p_dp, a_ctx.prime_size_bytes);
    pka_write(PKA_RSA_CRT_EXP_IN_DQ_CRT, a_ctx.p_dq, a_ctx.prime_size_bytes);
    pka_write(PKA_RSA_CRT_EXP_IN_QINV_CRT, a_ctx.p_qinv, a_ctx.prime_size_bytes);
    pka_write(PKA_RSA_CRT_EXP_IN_PRIME_P, a_ctx.p_p, a_ctx.prime_size_bytes);
    pka_write(PKA_RSA_CRT_EXP_IN_PRIME_Q, a_ctx.p_q, a_ctx.prime_size_bytes);

    pka_write(PKA_RSA_CRT_EXP_IN_EXPONENT_BASE, a_ctx.p_ciphertext, a_ctx.prime_size_bytes * rsa_modulus_multiplier);
}

void Public_key_accelerator::read(
    const Public_key_accelerator::Context<Public_key_accelerator::Mode::rsa_crt_exp>& a_ctx) const
{
    const std::size_t rsa_bytes = a_ctx.prime_size_bytes * rsa_modulus_multiplier;

    if (nullptr != a_ctx.p_out_plaintext)
    {
        read_from_pka_memory(a_ctx.p_out_plaintext, &PKA->RAM[PKA_RSA_CRT_EXP_OUT_RESULT], rsa_bytes);
    }

    // Safety: clearing private keys after finished operation
    wipe_pka_memory(&PKA->RAM[PKA_RSA_CRT_EXP_IN_DP_CRT], a_ctx.prime_size_bytes);
    wipe_pka_memory(&PKA->RAM[PKA_RSA_CRT_EXP_IN_DQ_CRT], a_ctx.prime_size_bytes);
    wipe_pka_memory(&PKA->RAM[PKA_RSA_CRT_EXP_IN_QINV_CRT], a_ctx.prime_size_bytes);
    wipe_pka_memory(&PKA->RAM[PKA_RSA_CRT_EXP_IN_PRIME_P], a_ctx.prime_size_bytes);
    wipe_pka_memory(&PKA->RAM[PKA_RSA_CRT_EXP_IN_PRIME_Q], a_ctx.prime_size_bytes);
    wipe_pka_memory(&PKA->RAM[PKA_RSA_CRT_EXP_OUT_RESULT], rsa_bytes);
}

template<>
void Public_key_accelerator::populate_result(Interrupt::Result<Public_key_accelerator::Mode::rsa_crt_exp>& a_result,
                                             Interrupt::Source a_source)
{
    a_result.source = a_source;

    if (Interrupt::Source::operation_end == a_source)
    {
        hkm_assert(nullptr != this->p_active_context);
        const auto* p_ctx = static_cast<const Context<Mode::rsa_crt_exp>*>(this->p_active_context);
        this->read(*p_ctx);
    }
}

// --------------------------------------------------------------------------------------------------------------------
// INTERRUPTS COMMON
// --------------------------------------------------------------------------------------------------------------------
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

    clear_pka_flags();

    a_p_this->irq_dispatcher(a_p_this, source, a_p_this->user_func, a_p_this->user_data);
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

template<Public_key_accelerator::Mode mode>
void Public_key_accelerator::Interrupt::start(const Context<mode>& a_ctx,
                                              const typename Callback_traits<mode>::Type& a_callback)
{
    Scoped_guard<nvic> guard;

    this->p_pka->user_func = reinterpret_cast<void*>(a_callback.function);
    this->p_pka->user_data = a_callback.p_user_data;
    this->p_pka->irq_dispatcher = &Public_key_accelerator::dispach_irq<mode>;

    this->p_pka->p_active_context = &a_ctx;

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

    a_p_this->p_active_context = nullptr;
    callback_fn(result);
}

template void Public_key_accelerator::Interrupt::start<Public_key_accelerator::Mode::ecdsa_verify>(
    const Context<Public_key_accelerator::Mode::ecdsa_verify>&,
    const Interrupt::Callback_traits<Public_key_accelerator::Mode::ecdsa_verify>::Type&);

template void Public_key_accelerator::Interrupt::start<Public_key_accelerator::Mode::rsa_crt_exp>(
    const Context<Public_key_accelerator::Mode::rsa_crt_exp>&,
    const Interrupt::Callback_traits<Public_key_accelerator::Mode::rsa_crt_exp>::Type&);

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

namespace xmcu::soc {

st::arm::m4::wb::rm0434::peripherals::Public_key_accelerator
peripheral<st::arm::m4::wb::rm0434::peripherals::Public_key_accelerator>::create()
{
    return st::arm::m4::wb::rm0434::peripherals::Public_key_accelerator(IRQn_Type::PKA_IRQn);
}

} // namespace xmcu::soc
