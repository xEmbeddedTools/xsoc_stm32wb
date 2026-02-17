#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// external
#include <stm32wbxx.h>

// xmcu
#include <rm0434/config.hpp>
#include <rm0434/rcc.hpp>
#include <xmcu/Duration.hpp>
#include <xmcu/Non_copyable.hpp>
#include <xmcu/bit.hpp>

namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals {

class AES : private xmcu::Non_copyable
{
public:
#if defined(XMCU_AES1_PRESENT)
    enum class _1;
#endif
#if defined(XMCU_AES2_PRESENT)
    enum class _2;
#endif

    enum class Mode : std::uint32_t
    {
        ecb_encrypt           = 0u,                                              // CHMOD:000, MODE:00
        ecb_key_derivation    = AES_CR_MODE_0,                                   // CHMOD:000, MODE:01
        ecb_decrypt           = AES_CR_MODE_1,                                   // CHMOD:000, MODE:10
        ecb_key_deriv_decrypt = AES_CR_MODE_0 | AES_CR_MODE_1,                   // CHMOD:000, MODE:11
        cbc_encrypt           = AES_CR_CHMOD_0,                                  // CHMOD:001, MODE:00
        cbc_key_derivation    = AES_CR_CHMOD_0 | AES_CR_MODE_0,                  // CHMOD:001, MODE:01
        cbc_decrypt           = AES_CR_CHMOD_0 | AES_CR_MODE_1,                  // CHMOD:001, MODE:10
        cbc_key_deriv_decrypt = AES_CR_CHMOD_0 | AES_CR_MODE_0 | AES_CR_MODE_1,  // CHMOD:001, MODE:11
        ctr_encrypt           = AES_CR_CHMOD_1,                                  // CHMOD:010, MODE:00
        ctr_decrypt           = AES_CR_CHMOD_1 | AES_CR_MODE_1,                  // CHMOD:010, MODE:10
        gcm_encrypt           = AES_CR_CHMOD_0 | AES_CR_CHMOD_1,                 // CHMOD:011, MODE:00
        gcm_decrypt           = AES_CR_CHMOD_0 | AES_CR_CHMOD_1 | AES_CR_MODE_1, // CHMOD:011, MODE:10
        gmac_encrypt          = gcm_encrypt,                                     // Alias
        gmac_decrypt          = gcm_decrypt,                                     // Alias
        ccm_encrypt           = AES_CR_CHMOD_2,                                  // CHMOD:100, MODE:00
        ccm_decrypt           = AES_CR_CHMOD_2 | AES_CR_MODE_1,                  // CHMOD:100, MODE:10
    };

    enum class Key_size : std::uint32_t
    {
        bits_128 = 0u,
        bits_256 = AES_CR_KEYSIZE,
    };

    enum class Data_type : std::uint32_t
    {
        full_word = 0u,
        half_word = AES_CR_DATATYPE_0,
        byte      = AES_CR_DATATYPE_1,
        bit       = AES_CR_DATATYPE_0 | AES_CR_DATATYPE_1,
    };

    using enum Mode;
    using enum Key_size;
    using enum Data_type;

    template<Mode> struct Context;

    class Pooling : private Non_copyable
    {
    public:
        enum class Status
        {
            ok,
            timeout,
        };

        template<Mode> struct Result;

        using enum Status;

        template<Mode mode> Result<mode> execute(const Context<mode>& a_ctx, xmcu::Milliseconds a_timeout);

    private:
        AES* p_aes;
        friend AES;
    };

    Pooling pooling;

private:
    AES(AES_TypeDef* a_p_registers, IRQn_Type a_irqn)
        : p_registers(a_p_registers)
        , irqn(a_irqn)
    {
        this->pooling.p_aes = this;
    }

    void enable();
    void disable();

    AES_TypeDef* p_registers;
    IRQn_Type    irqn;

    template<typename Periph_t, std::uint32_t id> friend class xmcu::soc::peripheral;
};

template<> struct AES::Context<AES::Mode::ecb_encrypt>
{
    AES::Key_size  key_size;
    AES::Data_type data_type = AES::Data_type::byte;

    const std::uint32_t* p_key;    // Pointer to Key (4 or 8 words)
    const std::uint8_t*  p_input;  // Plaintext (Must be multiple of 16 bytes)
    std::uint8_t*        p_output; // Ciphertext buffer
    std::size_t          size;     // Size in bytes
};

template<> struct AES::Context<AES::Mode::ecb_decrypt> : public AES::Context<AES::Mode::ecb_encrypt>
{
};

template<> struct AES::Pooling::Result<AES::Mode::ecb_encrypt>
{
    AES::Pooling::Status status = AES::Pooling::Status::timeout;
};

template<> struct AES::Pooling::Result<AES::Mode::ecb_decrypt> : AES::Pooling::Result<AES::Mode::ecb_encrypt>
{
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
        return st::arm::m4::wb::rm0434::peripherals::AES(AES1, IRQn_Type::AES1_IRQn);
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
        return st::arm::m4::wb::rm0434::peripherals::AES(AES2, IRQn_Type::AES2_IRQn);
    }
};
#endif
} // namespace xmcu::soc
