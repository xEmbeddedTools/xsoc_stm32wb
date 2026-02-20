/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// this
#include <rm0434/peripherals/AES/AES.hpp>

// std
#include <cstring>
#include <span>
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

void init_mode(AES::Mode mode, volatile std::uint32_t* a_p_cr, AES::Key_size a_key_size, AES::Data_type a_data_type)
{
    std::uint32_t cr_val = 0u;

    bit::flag::set(&cr_val, std::to_underlying(mode));
    bit::flag::set(&cr_val, std::to_underlying(a_key_size));
    bit::flag::set(&cr_val, std::to_underlying(a_data_type));

    *a_p_cr = cr_val;
}

void load_iv_gcm(AES_TypeDef* p_registers, const std::uint32_t* p_iv)
{
    if (nullptr == p_iv)
    {
        return;
    }

    // RM0434: IVR3 must be explicitly 0x00000002 for GCM
    p_registers->IVR3 = p_iv[0];
    p_registers->IVR2 = p_iv[1];
    p_registers->IVR1 = p_iv[2];
    p_registers->IVR0 = 0x00000002;
}

} // namespace

namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals {

using namespace xmcu;

template<> AES::Pooling::Status AES::Pooling::init<AES::Mode::ecb_encrypt>(const std::uint32_t* a_p_key,
                                                                           AES::Key_size        a_key_size,
                                                                           AES::Data_type       a_data_type,
                                                                           xmcu::Milliseconds   a_timeout)
{
    AES_TypeDef* regs = this->p_aes->p_registers;

    this->p_aes->disable();

    init_mode(AES::Mode::ecb_encrypt, &(regs->CR), a_key_size, a_data_type);

    load_key(regs, a_p_key, a_key_size);

    this->p_aes->enable();

    return Status::ok;
}

template<> AES::Pooling::Status AES::Pooling::init<AES::Mode::ecb_decrypt>(const std::uint32_t* a_p_key,
                                                                           AES::Key_size        a_key_size,
                                                                           AES::Data_type       a_data_type,
                                                                           xmcu::Milliseconds   a_timeout)
{
    const std::uint64_t end  = utils::tick_counter<Milliseconds>::get() + a_timeout.get();
    AES_TypeDef*        regs = this->p_aes->p_registers;

    this->p_aes->disable();

    // --- Step 1: Key Derivation (Mode 2) ---
    std::uint32_t cr_deriv = std::to_underlying(Mode::ecb_key_derivation);
    bit::flag::set(&cr_deriv, std::to_underlying(a_key_size));
    regs->CR = cr_deriv;

    load_key(regs, a_p_key, a_key_size);

    this->p_aes->enable();

    if (Status::ok != wait_for_ccf(regs, end))
    {
        this->p_aes->disable();
        return Status::timeout;
    }

    bit::flag::set(&(regs->CR), AES_CR_CCFC);

    // --- Step 2: Decryption Setup (Mode 3) ---
    init_mode(AES::Mode::ecb_decrypt, &(regs->CR), a_key_size, a_data_type);

    this->p_aes->enable();

    return Status::ok;
}

AES::Pooling::Status AES::init_gcm(AES::Mode                     a_mode,
                                   const std::uint32_t*          a_p_key,
                                   const std::uint32_t*          a_p_iv,
                                   std::span<const std::uint8_t> a_header,
                                   Key_size                      a_key_size,
                                   Data_type                     a_data_type,
                                   xmcu::Milliseconds            a_timeout)
{
    // RM0434 page 608/1530
    const std::uint64_t end = utils::tick_counter<Milliseconds>::get() + a_timeout.get();

    AES_TypeDef* regs = this->p_registers;

    // Reset state trackers
    this->pooling.m_gcm_payload_bytes = 0;
    this->pooling.m_gcm_header_bytes  = a_header.size();

    this->disable();

    init_mode(a_mode, &(regs->CR), a_key_size, a_data_type);
    bit::flag::clear(&(regs->CR), AES_CR_GCMPH); // Phase 1: Init

    load_key(regs, a_p_key, a_key_size);
    load_iv_gcm(regs, a_p_iv);

    this->enable();

    if (Pooling::Status::ok != wait_for_ccf(regs, end))
    {
        return Pooling::Status::timeout;
    }

    bit::flag::set(&(regs->CR), AES_CR_CCFC);

    if (!a_header.empty())
    {
        bit::flag::clear(&(regs->CR), AES_CR_GCMPH);
        bit::flag::set(&(regs->CR), AES_CR_GCMPH_0);
        this->enable();

        write_block(regs, a_header.data());

        if (Pooling::Status::ok != wait_for_ccf(regs, end))
        {
            return Pooling::Status::timeout;
        }

        bit::flag::set(&(regs->CR), AES_CR_CCFC);
        // this->disable();
    }

    return Pooling::Status::ok;
}

template<> AES::Pooling::Status AES::Pooling::init<AES::Mode::gcm_encrypt>(const std::uint32_t*          a_p_key,
                                                                           const std::uint32_t*          a_p_iv,
                                                                           std::span<const std::uint8_t> a_header,
                                                                           Key_size                      a_key_size,
                                                                           Data_type                     a_data_type,
                                                                           xmcu::Milliseconds            a_timeout)
{
    return this->p_aes->init_gcm(AES::Mode::gcm_encrypt, a_p_key, a_p_iv, a_header, a_key_size, a_data_type, a_timeout);
}

template<> AES::Pooling::Status AES::Pooling::init<AES::Mode::gcm_decrypt>(const std::uint32_t*          a_p_key,
                                                                           const std::uint32_t*          a_p_iv,
                                                                           std::span<const std::uint8_t> a_header,
                                                                           Key_size                      a_key_size,
                                                                           Data_type                     a_data_type,
                                                                           xmcu::Milliseconds            a_timeout)
{
    return this->p_aes->init_gcm(AES::Mode::gcm_decrypt, a_p_key, a_p_iv, a_header, a_key_size, a_data_type, a_timeout);
}

AES::Pooling::Status AES::process_ecb(const std::uint8_t* p_input, std::uint8_t* p_output, xmcu::Milliseconds a_timeout)
{
    const std::uint64_t end = tick_counter<Milliseconds>::get() + a_timeout.get();

    write_block(this->p_registers, p_input);

    if (Pooling::Status::ok != wait_for_ccf(this->p_registers, end))
    {
        this->disable();
        return Pooling::Status::timeout;
    }

    read_block(this->p_registers, p_output);

    bit::flag::set(&(this->p_registers->CR), AES_CR_CCFC);

    return Pooling::Status::ok;
}

template<> AES::Pooling::Status AES::Pooling::process<AES::Mode::ecb_encrypt>(const std::uint8_t* p_input,
                                                                              std::uint8_t*       p_output,
                                                                              xmcu::Milliseconds  a_timeout,
                                                                              std::uint32_t* /* p_tag */)
{
    return this->p_aes->process_ecb(p_input, p_output, a_timeout);
}

template<> AES::Pooling::Status AES::Pooling::process<AES::Mode::ecb_decrypt>(const std::uint8_t* p_input,
                                                                              std::uint8_t*       p_output,
                                                                              xmcu::Milliseconds  a_timeout,
                                                                              std::uint32_t* /* p_tag */)
{
    return this->p_aes->process_ecb(p_input, p_output, a_timeout);
}

AES::Pooling::Status AES::process_gcm(const std::uint8_t* a_p_input,
                                      std::uint8_t*       a_p_output,
                                      xmcu::Milliseconds  a_timeout,
                                      std::uint32_t*      a_p_tag)
{
    const std::uint64_t end  = tick_counter<Milliseconds>::get() + a_timeout.get();
    AES_TypeDef*        regs = this->p_registers;

    // If GCMPH is not 10b (Payload Phase), we transition to it.
    if ((regs->CR & AES_CR_GCMPH) != AES_CR_GCMPH_1)
    {
        bit::flag::clear(&(regs->CR), AES_CR_GCMPH);
        bit::flag::set(&(regs->CR), AES_CR_GCMPH_1); // Phase 3: Payload
        this->enable();
    }

    // 2. Process Block
    write_block(regs, a_p_input);

    if (Pooling::Status::ok != wait_for_ccf(regs, end))
    {
        this->disable();
        return Pooling::Status::timeout;
    }

    read_block(regs, a_p_output);
    bit::flag::set(&(regs->CR), AES_CR_CCFC);

    this->pooling.m_gcm_payload_bytes += 16;

    // 3. Auto-Finalize for GCM if tag is requested
    if (nullptr != a_p_tag)
    {
        // RM0434: DO NOT disable the peripheral before entering Phase 4!
        bit::flag::clear(&(regs->CR), AES_CR_GCMPH);
        bit::flag::set(&(regs->CR), AES_CR_GCMPH_0 | AES_CR_GCMPH_1); // Phase 4: Final

        std::uint64_t header_bits  = static_cast<std::uint64_t>(this->pooling.m_gcm_header_bytes) * 8u;
        std::uint64_t payload_bits = static_cast<std::uint64_t>(this->pooling.m_gcm_payload_bytes) * 8u;

        // If DATATYPE is 10b (byte), the hardware bypasses swapping for DINR here.
        // We must push the Big-Endian lengths manually using __builtin_bswap32.
        if ((regs->CR & AES_CR_DATATYPE_Msk) == AES_CR_DATATYPE_1)
        {
            regs->DINR = __builtin_bswap32(static_cast<std::uint32_t>(header_bits >> 32));
            regs->DINR = __builtin_bswap32(static_cast<std::uint32_t>(header_bits & 0xFFFFFFFF));
            regs->DINR = __builtin_bswap32(static_cast<std::uint32_t>(payload_bits >> 32));
            regs->DINR = __builtin_bswap32(static_cast<std::uint32_t>(payload_bits & 0xFFFFFFFF));
        }
        else
        {
            regs->DINR = static_cast<std::uint32_t>(header_bits >> 32);
            regs->DINR = static_cast<std::uint32_t>(header_bits & 0xFFFFFFFF);
            regs->DINR = static_cast<std::uint32_t>(payload_bits >> 32);
            regs->DINR = static_cast<std::uint32_t>(payload_bits & 0xFFFFFFFF);
        }

        if (Pooling::Status::ok != wait_for_ccf(regs, end)) return Pooling::Status::timeout;

        a_p_tag[0] = regs->DOUTR;
        a_p_tag[1] = regs->DOUTR;
        a_p_tag[2] = regs->DOUTR;
        a_p_tag[3] = regs->DOUTR;

        bit::flag::set(&(regs->CR), AES_CR_CCFC);
        this->disable();
    }

    return Pooling::Status::ok;
}

template<> AES::Pooling::Status AES::Pooling::process<AES::Mode::gcm_encrypt>(const std::uint8_t* a_p_input,
                                                                              std::uint8_t*       a_p_output,
                                                                              xmcu::Milliseconds  a_timeout,
                                                                              std::uint32_t*      a_p_tag)
{
    return this->p_aes->process_gcm(a_p_input, a_p_output, a_timeout, a_p_tag);
}

template<> AES::Pooling::Status AES::Pooling::process<AES::Mode::gcm_decrypt>(const std::uint8_t* a_p_input,
                                                                              std::uint8_t*       a_p_output,
                                                                              xmcu::Milliseconds  a_timeout,
                                                                              std::uint32_t*      a_p_tag)
{
    return this->p_aes->process_gcm(a_p_input, a_p_output, a_timeout, a_p_tag);
}

void AES::Pooling::deinit()
{
    this->p_aes->disable();
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
