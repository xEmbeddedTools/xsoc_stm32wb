/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// this
#include <rm0434/peripherals/AES/AES.hpp>

// std
#include <cstring>
#include <utility>

// xmcu
#include <rm0434/utils/tick_counter.hpp>
#include <xmcu/Duration.hpp>
#include <xmcu/bit.hpp>

namespace {

using namespace xmcu;
using namespace xmcu::soc::st::arm::m4::wb::rm0434::utils;
using namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals;

void load_key(AES_TypeDef* p_registers, const std::uint32_t* p_key, AES::Key_size key_size)
{
    if (!p_key) return;

    if (key_size == AES::Key_size::bits_256)
    {
        p_registers->KEYR7 = p_key[0];
        p_registers->KEYR6 = p_key[1];
        p_registers->KEYR5 = p_key[2];
        p_registers->KEYR4 = p_key[3];
        p_registers->KEYR3 = p_key[4];
        p_registers->KEYR2 = p_key[5];
        p_registers->KEYR1 = p_key[6];
        p_registers->KEYR0 = p_key[7];
    }
    else
    {
        p_registers->KEYR3 = p_key[0];
        p_registers->KEYR2 = p_key[1];
        p_registers->KEYR1 = p_key[2];
        p_registers->KEYR0 = p_key[3];
    }
}

void write_block(AES_TypeDef* p_registers, const std::uint8_t* p_data)
{
    std::uint32_t buffer[4];

    std::memcpy(buffer, p_data, 16);

    p_registers->DINR = buffer[0];
    p_registers->DINR = buffer[1];
    p_registers->DINR = buffer[2];
    p_registers->DINR = buffer[3];
}

void read_block(AES_TypeDef* p_registers, std::uint8_t* p_out)
{
    std::uint32_t buffer[4];

    buffer[0] = p_registers->DOUTR;
    buffer[1] = p_registers->DOUTR;
    buffer[2] = p_registers->DOUTR;
    buffer[3] = p_registers->DOUTR;

    std::memcpy(p_out, buffer, 16);
}

AES::Pooling::Status wait_for_ccf(AES_TypeDef* p_registers, std::uint64_t a_end_timestamp)
{
    while (!bit::flag::is(p_registers->SR, AES_SR_CCF))
    {
        if (tick_counter<Milliseconds>::get() > a_end_timestamp)
        {
            return AES::Pooling::Status::timeout;
        }
    }
    return AES::Pooling::Status::ok;
}

} // namespace

namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals {

using namespace xmcu;

template<> AES::Pooling::Result<AES::ecb_encrypt> AES::Pooling::execute(const Context<AES::ecb_encrypt>& a_ctx,
                                                                        xmcu::Milliseconds               a_timeout)
{
    const std::uint64_t end = tick_counter<Milliseconds>::get() + a_timeout.get();

    AES_TypeDef* regs = this->p_aes->p_registers;

    this->p_aes->disable();

    std::uint32_t cr_val = 0u;

    bit::flag::set(&cr_val, std::to_underlying(AES::ecb_encrypt));
    bit::flag::set(&cr_val, std::to_underlying(a_ctx.key_size));
    bit::flag::set(&cr_val, std::to_underlying(a_ctx.data_type));

    regs->CR = cr_val;

    load_key(regs, a_ctx.p_key, a_ctx.key_size);

    this->p_aes->enable();

    const std::size_t   num_blocks = a_ctx.size / 16;
    const std::uint8_t* in_ptr     = a_ctx.p_input;
    std::uint8_t*       out_ptr    = a_ctx.p_output;

    for (std::size_t i = 0; i < num_blocks; ++i)
    {
        write_block(regs, in_ptr);

        if (Status::ok != wait_for_ccf(regs, end))
        {
            this->p_aes->disable();
            return { .status = Status::timeout };
        }

        read_block(regs, out_ptr);

        bit::flag::set(&(regs->CR), AES_CR_CCFC);

        in_ptr += 16;
        out_ptr += 16;
    }

    this->p_aes->disable();

    return { .status = Status::ok };
}

template<> AES::Pooling::Result<AES::ecb_decrypt> AES::Pooling::execute(const Context<AES::ecb_decrypt>& a_ctx,
                                                                        xmcu::Milliseconds               a_timeout)
{
    const std::uint64_t end = tick_counter<Milliseconds>::get() + a_timeout.get();

    AES_TypeDef* regs = this->p_aes->p_registers;

    // --- Step 1: Key Derivation (Mode 2) ---
    // The hardware must run the key backwards first.
    this->p_aes->disable();

    std::uint32_t cr_deriv = AES_CR_MODE_0; // Mode 2
    bit::flag::set(&cr_deriv, std::to_underlying(a_ctx.key_size));

    regs->CR = cr_deriv;

    load_key(regs, a_ctx.p_key, a_ctx.key_size);

    this->p_aes->enable();

    if (Status::ok != wait_for_ccf(regs, end))
    {
        this->p_aes->disable();
        return { .status = Status::timeout };
    }

    bit::flag::set(&(regs->CR), AES_CR_CCFC);

    // --- Step 2: Decryption (Mode 3) ---
    std::uint32_t cr_val = AES_CR_MODE_1; // Mode 3
    bit::flag::set(&cr_val, std::to_underlying(a_ctx.key_size));
    bit::flag::set(&cr_val, std::to_underlying(a_ctx.data_type));

    regs->CR = cr_val;

    this->p_aes->enable();

    std::size_t         num_blocks = a_ctx.size / 16;
    const std::uint8_t* in_ptr     = a_ctx.p_input;
    std::uint8_t*       out_ptr    = a_ctx.p_output;

    for (std::size_t i = 0; i < num_blocks; ++i)
    {
        write_block(regs, in_ptr);

        if (Status::ok != wait_for_ccf(regs, end))
        {
            this->p_aes->disable();
            return { .status = Status::timeout };
        }

        read_block(regs, out_ptr);

        bit::flag::set(&(regs->CR), AES_CR_CCFC);

        in_ptr += 16;
        out_ptr += 16;
    }

    this->p_aes->disable();

    return { .status = Status::ok };
}

void AES::enable()
{
    bit::flag::set(&(this->p_registers->CR), AES_CR_EN);
}

void AES::disable()
{
    bit::flag::clear(&(this->p_registers->CR), AES_CR_EN);
}

} // namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals

namespace xmcu::soc::st::arm::m4::wb::rm0434 {

#if defined(XMCU_AES1_PRESENT)
void rcc<AES, AES::_1>::enable()
{
    bit::flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_AES1EN);
    bit::flag::is(RCC->AHB2ENR, RCC_AHB2ENR_AES1EN);
}

void rcc<AES, AES::_1>::disable()
{
    bit::flag::clear(&(RCC->AHB2ENR), RCC_AHB2ENR_AES1EN);
}
#endif

#if defined(XMCU_AES2_PRESENT)
void rcc<AES, AES::_2>::enable()
{
    bit::flag::set(&(RCC->AHB3ENR), RCC_AHB3ENR_AES2EN);
    bit::flag::is(RCC->AHB3ENR, RCC_AHB3ENR_AES2EN);
}

void rcc<AES, AES::_2>::disable()
{
    bit::flag::clear(&(RCC->AHB3ENR), RCC_AHB3ENR_AES2EN);
}
#endif

} // namespace xmcu::soc::st::arm::m4::wb::rm0434
