#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// std
#include "stm32wb55xx.h"

#include <expected>
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

    enum class Operation
    {
        ecb_encryption,
        ecb_decryption,
        gcm_encryption,
        gcm_decryption,
    };

    enum class Key_size : std::uint32_t
    {
        bits_128 = 0u,
        bits_256 = AES_CR_KEYSIZE,
    };

    using enum Operation;
    using enum Key_size;

    using Block = std::span<std::uint8_t, 16>;
    using Const_block = std::span<const std::uint8_t, 16>;

    class Pooling : private Non_copyable
    {
    public:
        enum class Error
        {
            timeout,
            null_pointer_key,
            null_pointer_iv,
        };

        using enum Error;

        using Result = std::expected<void, Error>;

        class CommonContext
        {
        public:
            Result process(Const_block a_input, Block a_output, xmcu::Milliseconds a_timeout = 20_ms);

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
            Result add_header(Const_block a_header, xmcu::Milliseconds a_timeout = 20_ms);
            Result add_payload(Const_block a_input,
                               Block a_p_output,
                               std::uint32_t a_length,
                               xmcu::Milliseconds a_timeout = 20_ms);
            Result finalize(std::uint8_t* a_p_tag, xmcu::Milliseconds a_timeout = 20_ms);

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

        template<typename Context> using Init_result = std::expected<Context, Error>;

        template<Operation op> Init_result<CommonContext>
        init(const std::uint32_t* a_p_key, Key_size a_key_size, xmcu::Milliseconds a_timeout = 20_ms);

        template<Operation op> Init_result<GcmContext> init(const std::uint32_t* a_p_key,
                                                            Key_size a_key_size,
                                                            const std::uint32_t* a_p_iv,
                                                            xmcu::Milliseconds a_timeout = 20_ms);

        void deinit();

    private:
        Init_result<GcmContext> init_gcm(AES::Operation a_operation,
                                         const std::uint32_t* a_p_key,
                                         Key_size a_key_size,
                                         const std::uint32_t* a_p_iv,
                                         xmcu::Milliseconds a_timeout = 20_ms);

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

    Pooling::Init_result<Pooling::GcmContext> init_gcm(AES::Operation a_operation,
                                                       const std::uint32_t* a_p_key,
                                                       const std::uint32_t* a_p_iv,
                                                       std::span<const std::uint8_t> a_header,
                                                       Key_size a_key_size,
                                                       xmcu::Milliseconds a_timeout);

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
