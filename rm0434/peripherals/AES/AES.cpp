/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// this
#include <rm0434/peripherals/AES/AES.hpp>

// std
#include <cstring>
#include <expected>
#include <utility>

// xmcu
#include <rm0434/utils/tick_counter.hpp>
#include <xmcu/Duration.hpp>
#include <xmcu/bit.hpp>

namespace {

using namespace xmcu;
using namespace xmcu::soc::st::arm::m4::wb::rm0434::utils;
using namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals;

void load_key(AES_TypeDef* p_registers, AES::Key a_key)
{
    if (a_key.size == AES::Key::Key_size::bits_256)
    {
        p_registers->KEYR7 = a_key.p_data[0];
        p_registers->KEYR6 = a_key.p_data[1];
        p_registers->KEYR5 = a_key.p_data[2];
        p_registers->KEYR4 = a_key.p_data[3];
        p_registers->KEYR3 = a_key.p_data[4];
        p_registers->KEYR2 = a_key.p_data[5];
        p_registers->KEYR1 = a_key.p_data[6];
        p_registers->KEYR0 = a_key.p_data[7];
    }
    else
    {
        p_registers->KEYR3 = a_key.p_data[0];
        p_registers->KEYR2 = a_key.p_data[1];
        p_registers->KEYR1 = a_key.p_data[2];
        p_registers->KEYR0 = a_key.p_data[3];
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

bool wait_for_ccf(AES_TypeDef* p_registers, std::uint64_t a_end_timestamp)
{
    while (!bit::flag::is(p_registers->SR, AES_SR_CCF))
    {
        if (tick_counter<Milliseconds>::get() > a_end_timestamp)
        {
            return false;
        }
    }
    return true;
}

void init_mode(AES::Operation a_operation, volatile std::uint32_t* a_p_cr, AES::Key a_key, AES::Data_type a_data_type)
{
    std::uint32_t cr_val = 0u;

    switch (a_operation)
    {
        case AES::Operation::ecb_encryption:
            // CHMOD: 000, MODE: 00
            break;
        case AES::Operation::ecb_decryption:
            // CHMOD: 000, MODE: 10
            bit::flag::set(&cr_val, AES_CR_MODE_1);
            break;
        case AES::Operation::gcm_encryption:
            // CHMOD: 011, MODE: 00
            bit::flag::set(&cr_val, AES_CR_CHMOD_0 | AES_CR_CHMOD_1);
            break;
        case AES::Operation::gcm_decryption:
            // CHMOD: 011, MODE: 10
            bit::flag::set(&cr_val, AES_CR_CHMOD_0 | AES_CR_CHMOD_1 | AES_CR_MODE_1);
            break;
    };

    bit::flag::set(&cr_val, std::to_underlying(a_key.size));
    bit::flag::set(&cr_val, std::to_underlying(a_data_type));

    *a_p_cr = cr_val;
}

void load_iv(AES_TypeDef* p_registers, const std::uint32_t* p_iv)
{
    if (nullptr == p_iv)
    {
        return;
    }

    p_registers->IVR3 = p_iv[0];
    p_registers->IVR2 = p_iv[1];
    p_registers->IVR1 = p_iv[2];
    p_registers->IVR0 = 0x00000002;
}

} // namespace

namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals {

using namespace xmcu;

template<> AES::Pooling::Init_result<AES::Pooling::CommonContext>
AES::Pooling::init<AES::Operation::ecb_encryption>(AES::Key a_key, xmcu::Milliseconds)
{
    AES_TypeDef* regs = this->p_aes->p_registers;

    this->p_aes->disable();

    init_mode(AES::Operation::ecb_encryption, &(regs->CR), a_key, AES::Data_type::byte);

    load_key(regs, a_key);

    this->p_aes->enable();

    return CommonContext(this->p_aes);
}

template<> AES::Pooling::Init_result<AES::Pooling::CommonContext>
AES::Pooling::init<AES::Operation::ecb_decryption>(AES::Key a_key, xmcu::Milliseconds a_timeout)
{
    const std::uint64_t end = utils::tick_counter<Milliseconds>::get() + a_timeout.get();
    AES_TypeDef* regs = this->p_aes->p_registers;

    this->p_aes->disable();

    // Step 1: Key Derivation CHMOD: 000, MODE: 01
    std::uint32_t cr_deriv = AES_CR_MODE_0;
    bit::flag::set(&cr_deriv, std::to_underlying(a_key.size));
    regs->CR = cr_deriv;

    load_key(regs, a_key);

    this->p_aes->enable();

    if (false == wait_for_ccf(regs, end))
    {
        this->p_aes->disable();
        return std::unexpected(Error::timeout);
    }

    bit::flag::set(&(regs->CR), AES_CR_CCFC);

    init_mode(AES::Operation::ecb_decryption, &(regs->CR), a_key, AES::Data_type::byte);

    this->p_aes->enable();

    return CommonContext(this->p_aes);
}

AES::Pooling::Processing_result
AES::Pooling::CommonContext::process(AES::Const_block a_input, AES::Block a_output, xmcu::Milliseconds a_timeout)
{
    const std::uint64_t end = tick_counter<Milliseconds>::get() + a_timeout.get();

    write_block(this->p_aes->p_registers, a_input.data());

    if (false == wait_for_ccf(this->p_aes->p_registers, end))
    {
        this->p_aes->disable();
        return std::unexpected(Error::timeout);
    }

    read_block(this->p_aes->p_registers, a_output.data());

    bit::flag::set(&(this->p_aes->p_registers->CR), AES_CR_CCFC);

    return {};
}

template<> AES::Pooling::Init_result<AES::Pooling::GcmContext>
AES::Pooling::init<AES::Operation::gcm_encryption>(AES::Key a_key, AES::Iv a_iv, xmcu::Milliseconds a_timeout)
{
    return this->init_gcm(AES::Operation::gcm_encryption, a_key, a_iv, a_timeout);
}

template<> AES::Pooling::Init_result<AES::Pooling::GcmContext>
AES::Pooling::init<AES::Operation::gcm_decryption>(AES::Key a_key, AES::Iv a_iv, xmcu::Milliseconds a_timeout)
{
    return this->init_gcm(AES::Operation::gcm_decryption, a_key, a_iv, a_timeout);
}

AES::Pooling::Processing_result AES::Pooling::GcmContext::add_header(Const_block a_header,
                                                                     xmcu::Limited<std::uint8_t, 0, 16> a_valid_bytes,
                                                                     xmcu::Milliseconds a_timeout)
{
    if (a_valid_bytes.get() == 0)
    {
        return {};
    }

    const std::uint64_t end = utils::tick_counter<Milliseconds>::get() + a_timeout.get();
    AES_TypeDef* regs = this->p_aes->p_registers;

    if ((regs->CR & AES_CR_GCMPH) != AES_CR_GCMPH_0)
    {
        regs->CR = (regs->CR & ~AES_CR_GCMPH) | AES_CR_GCMPH_0;
        this->p_aes->enable();
    }

    write_block(regs, a_header.data());

    if (false == wait_for_ccf(regs, end))
    {
        this->p_aes->disable();
        return std::unexpected(Error::timeout);
    }

    bit::flag::set(&(regs->CR), AES_CR_CCFC);

    this->header_bytes += a_valid_bytes.get();

    return {};
}

AES::Pooling::Processing_result AES::Pooling::GcmContext::add_payload(Const_block a_input,
                                                                      Block a_output,
                                                                      xmcu::Limited<std::uint8_t, 0, 16> a_valid_bytes,
                                                                      xmcu::Milliseconds a_timeout)
{
    if (0 == a_valid_bytes.get())
    {
        return {};
    }

    const std::uint64_t end = tick_counter<Milliseconds>::get() + a_timeout.get();
    AES_TypeDef* regs = this->p_aes->p_registers;

    if ((regs->CR & AES_CR_GCMPH) != AES_CR_GCMPH_1)
    {
        regs->CR = (regs->CR & ~AES_CR_GCMPH) | AES_CR_GCMPH_1;
    }

    std::uint32_t padding_bytes = 16 - a_valid_bytes.get();
    regs->CR = (regs->CR & ~AES_CR_NPBLB_Msk) | (padding_bytes << AES_CR_NPBLB_Pos);

    write_block(regs, a_input.data());

    if (false == wait_for_ccf(regs, end))
    {
        this->p_aes->disable();
        return std::unexpected(Error::timeout);
    }

    read_block(regs, a_output.data());
    bit::flag::set(&(regs->CR), AES_CR_CCFC);

    this->payload_bytes += a_valid_bytes.get();

    return {};
}

AES::Pooling::Processing_result AES::Pooling::GcmContext::finalize(Block a_tag, xmcu::Milliseconds a_timeout)
{
    const std::uint64_t end = tick_counter<Milliseconds>::get() + a_timeout.get();
    AES_TypeDef* regs = this->p_aes->p_registers;

    regs->CR = (regs->CR & ~AES_CR_GCMPH) | (AES_CR_GCMPH_0 | AES_CR_GCMPH_1);

    std::uint32_t header_bits = this->header_bytes * 8;
    std::uint32_t payload_bits = this->payload_bytes * 8;

    regs->DINR = 0;
    regs->DINR = header_bits;
    regs->DINR = 0;
    regs->DINR = payload_bits;

    if (false == wait_for_ccf(regs, end))
    {
        this->p_aes->disable();
        return std::unexpected(Error::timeout);
    }

    bit::flag::set(&(regs->CR), AES_CR_CCFC);

    read_block(regs, a_tag.data());

    this->p_aes->disable();

    return {};
}

void AES::enable()
{
    bit::flag::set(&(this->p_registers->CR), AES_CR_EN);
}

void AES::disable()
{
    bit::flag::clear(&(this->p_registers->CR), AES_CR_EN);
}

AES::Pooling::Init_result<AES::Pooling::GcmContext>
AES::Pooling::init_gcm(AES::Operation a_operation, AES::Key a_key, AES::Iv a_iv, xmcu::Milliseconds a_timeout)
{
    const std::uint64_t end = utils::tick_counter<Milliseconds>::get() + a_timeout.get();

    AES_TypeDef* regs = this->p_aes->p_registers;

    this->p_aes->disable();

    init_mode(a_operation, &(regs->CR), a_key, AES::Data_type::byte);
    bit::flag::clear(&(regs->CR), AES_CR_GCMPH);

    load_key(regs, a_key);
    load_iv(regs, a_iv.data());

    this->p_aes->enable();

    if (false == wait_for_ccf(regs, end))
    {
        this->p_aes->disable();
        return std::unexpected(Error::timeout);
    }

    bit::flag::set(&(regs->CR), AES_CR_CCFC);

    return GcmContext(this->p_aes);
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
