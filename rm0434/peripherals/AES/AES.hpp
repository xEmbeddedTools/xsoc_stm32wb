#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// std
#include <span>

// external
#include <stm32wbxx.h>

// xmcu
#include <rm0434/config.hpp>
#include <rm0434/rcc.hpp>
#include <soc/peripheral.hpp>
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

        template<Mode mode> Status init(const std::uint32_t* a_p_key,
                                        Key_size             a_key_size,
                                        Data_type            a_data_type,
                                        xmcu::Milliseconds   a_timeout = 10_ms);

        template<Mode mode> Status init(const std::uint32_t*          a_p_key,
                                        const std::uint32_t*          a_p_iv,
                                        std::span<const std::uint8_t> a_header,
                                        Key_size                      a_key_size,
                                        Data_type                     a_data_type,
                                        xmcu::Milliseconds            a_timeout = 10_ms);

        template<Mode mode> Status process(const std::uint8_t* a_p_input,
                                           std::uint8_t*       a_p_output,
                                           xmcu::Milliseconds  a_timeout,
                                           std::uint32_t*      a_p_tag = nullptr);

        void deinit();

    private:
        AES* p_aes;
        friend AES;

        // static Status process_ecb(const std::uint8_t* p_input, std::uint8_t* p_output, xmcu::Milliseconds a_timeout);

        std::size_t m_gcm_payload_bytes = 0;
        std::size_t m_gcm_header_bytes  = 0;
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

    Pooling::Status process_ecb(const std::uint8_t* p_input, std::uint8_t* p_output, xmcu::Milliseconds a_timeout);

    Pooling::Status init_gcm(AES::Mode                     a_mode,
                             const std::uint32_t*          a_p_key,
                             const std::uint32_t*          a_p_iv,
                             std::span<const std::uint8_t> a_header,
                             Key_size                      a_key_size,
                             Data_type                     a_data_type,
                             xmcu::Milliseconds            a_timeout);

    Pooling::Status process_gcm(const std::uint8_t* a_p_input,
                                std::uint8_t*       a_p_output,
                                xmcu::Milliseconds  a_timeout,
                                std::uint32_t*      a_p_tag);

    AES_TypeDef* p_registers;
    IRQn_Type    irqn;

    template<typename Periph_t, std::uint32_t id> friend class xmcu::soc::peripheral;
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
