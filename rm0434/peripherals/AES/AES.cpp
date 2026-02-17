/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// this
#include "stm32wb55xx.h"
#include <rm0434/peripherals/AES/AES.hpp>

// std
#include <cstring>
#include <utility>

// xmcu
#include <xmcu/Duration.hpp>
#include <xmcu/bit.hpp>

namespace {

using namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals;

void load_key(AES_TypeDef* p_registers, const std::uint32_t* p_key, AES::Key_size key_size)
{
    if (!p_key) return;

    p_registers->KEYR0 = p_key[0];
    p_registers->KEYR1 = p_key[1];
    p_registers->KEYR2 = p_key[2];
    p_registers->KEYR3 = p_key[3];

    if (key_size == AES::bits_256)
    {
        p_registers->KEYR4 = p_key[4];
        p_registers->KEYR5 = p_key[5];
        p_registers->KEYR6 = p_key[6];
        p_registers->KEYR7 = p_key[7];
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

AES::Pooling::Status wait_for_ccf(AES_TypeDef* p_registers, xmcu::Milliseconds a_timeout)
{

}

} // namespace

namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals {

using namespace xmcu;

template<> AES::Pooling::Result<AES::ecb_encrypt> AES::Pooling::execute(const Context<AES::ecb_encrypt>& a_ctx,
                                                                        xmcu::Milliseconds               a_timeout)
{
    AES_TypeDef* regs = this->p_aes->p_registers;

    this->p_aes->disable();

    std::uint32_t cr_val = 0u;

    bit::flag::set(&cr_val, std::to_underlying(AES::ecb_encrypt));
    bit::flag::set(&cr_val, std::to_underlying(a_ctx.key_size));
    bit::flag::set(&cr_val, std::to_underlying(a_ctx.data_type));

    regs->CR = cr_val;

    load_key(regs, a_ctx.p_key, a_ctx.key_size);

    this->p_aes->enable();

    const std::size_t num_blocks = a_ctx.size / 16;
    const std::uint8_t* in_ptr = a_ctx.p_input;
    std::uint8_t* out_ptr = a_ctx.p_output;

    for (std::size_t i = 0; i < num_blocks; ++i)
    {
        write_block(regs, in_ptr);

        if (wait_for_ccf(regs, a_timeout) != Status::ok)
        {
            this->p_aes->disable();
            return { .status = Status::timeout };
        }

        read_block(regs, out_ptr);

        bit::flag::set(&(regs->CR), AES_CR_CCFC);
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

void rcc<AES, AES::_1>::enable()
{
    bit::flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_AES1EN);
    bit::flag::is(RCC->AHB2ENR, RCC_AHB2ENR_AES1EN);
}

void rcc<AES, AES::_2>::disable()
{
    bit::flag::set(&(RCC->AHB3ENR), RCC_AHB3ENR_AES2EN);
    bit::flag::is(RCC->AHB3ENR, RCC_AHB3ENR_AES2EN);
}

} // namespace xmcu::soc::st::arm::m4::wb::rm0434
