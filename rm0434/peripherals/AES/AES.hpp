#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// std
#include <expected>
#include <span>

// external
#include <stm32wbxx.h>

// xmcu
#include <rm0434/config.hpp>
#include <rm0434/rcc.hpp>
#include <soc/peripheral.hpp>
#include <xmcu/Duration.hpp>
#include <xmcu/Limited.hpp>
#include <xmcu/Non_copyable.hpp>

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

    enum class Operation
    {
        ecb_encryption,
        ecb_decryption,
        gcm_encryption,
        gcm_decryption,
    };

    using enum Operation;

    using Block = std::span<std::uint8_t, 16>;
    using Const_block = std::span<const std::uint8_t, 16>;
    using Iv = std::span<const std::uint32_t, 3>;

    struct Key
    {
        enum class Key_size : std::uint32_t
        {
            bits_128 = 0u,
            bits_256 = AES_CR_KEYSIZE,
        };

        const std::uint32_t* p_data;
        Key_size size;

        constexpr Key(std::span<const std::uint32_t, 4> a_key)
            : p_data(a_key.data())
            , size(Key_size::bits_128)
        {
        }
        constexpr Key(std::span<const std::uint32_t, 8> a_key)
            : p_data(a_key.data())
            , size(Key_size::bits_256)
        {
        }
    };

    class Pooling : private Non_copyable
    {
    public:
        enum class Error
        {
            timeout,
        };

        using enum Error;

        template<typename Context> using Init_result = std::expected<Context, Error>;
        using Processing_result = std::expected<void, Error>;

        class CommonContext
        {
        public:
            Processing_result process(Const_block a_input, Block a_output, xmcu::Milliseconds a_timeout = 10_ms);

        private:
            friend class Pooling;
            explicit CommonContext(AES* a_p_aes)
                : p_aes(a_p_aes)
            {
            }

            AES* p_aes;
        };

        class GcmContext
        {
        public:
            Processing_result add_header(Const_block a_header,
                                         xmcu::Limited<std::uint8_t, 0, 16> a_valid_bytes,
                                         xmcu::Milliseconds a_timeout = 10_ms);
            Processing_result add_payload(Const_block a_input,
                                          Block a_p_output,
                                          xmcu::Limited<std::uint8_t, 0, 16> a_valid_bytes,
                                          xmcu::Milliseconds a_timeout = 10_ms);
            Processing_result finalize(Block a_p_tag, xmcu::Milliseconds a_timeout = 10_ms);

        private:
            friend class Pooling;

            explicit GcmContext(AES* a_p_aes)
                : p_aes(a_p_aes)
                , header_bytes(0)
                , payload_bytes(0)
            {
            }

            AES* p_aes;
            std::uint32_t header_bytes;
            std::uint32_t payload_bytes;
        };

        template<Operation op> Init_result<CommonContext> init(Key a_key, xmcu::Milliseconds a_timeout = 10_ms);

        template<Operation op> Init_result<GcmContext> init(Key a_key, Iv a_iv, xmcu::Milliseconds a_timeout = 10_ms);

    private:
        Init_result<GcmContext>
        init_gcm(AES::Operation a_operation, Key a_key, Iv a_iv, xmcu::Milliseconds a_timeout = 10_ms);

        AES* p_aes;
        friend AES;
    };

    enum class Data_type : std::uint32_t
    {
        full_word = 0u,
        half_word = AES_CR_DATATYPE_0,
        byte = AES_CR_DATATYPE_1,
        bit = AES_CR_DATATYPE_0 | AES_CR_DATATYPE_1,
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
    IRQn_Type irqn;

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
