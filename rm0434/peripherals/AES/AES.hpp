#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// external
#include <stm32wbxx.h>

// xmcu
#include <rm0434/rcc.hpp>
#include <rm0434/config.hpp>

#include <xmcu/Non_copyable.hpp>


namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals {

class AES : private xmcu::Non_copyable {
public:
#if defined(XMCU_AES1_PRESENT)
    enum class _1;
#endif
#if defined(XMCU_AES2_PRESENT)
    enum class _2;
#endif

    enum class Mode : std::uint32_t
    {
        // ====================================================================
        // 1. Electronic Codebook (ECB)
        // ====================================================================
        // Simplest mode. Block-by-block, no chaining.
        ecb_encrypt           = 0u,                                                      // CHMOD:000, MODE:00
        ecb_key_derivation    = AES_CR_MODE_0,                                           // CHMOD:000, MODE:01
        ecb_decrypt           = AES_CR_MODE_1,                                           // CHMOD:000, MODE:10
        ecb_key_deriv_decrypt = AES_CR_MODE_0 | AES_CR_MODE_1,                           // CHMOD:000, MODE:11

        // ====================================================================
        // 2. Cipher Block Chaining (CBC)
        // ====================================================================
        // Uses Initialization Vector (IV).
        cbc_encrypt           = AES_CR_CHMOD_0,                                          // CHMOD:001, MODE:00
        // Note: Hardware ignores CHMOD in Mode 2, but we define it for API consistency
        cbc_key_derivation    = AES_CR_CHMOD_0 | AES_CR_MODE_0,                          // CHMOD:001, MODE:01
        cbc_decrypt           = AES_CR_CHMOD_0 | AES_CR_MODE_1,                          // CHMOD:001, MODE:10
        cbc_key_deriv_decrypt = AES_CR_CHMOD_0 | AES_CR_MODE_0 | AES_CR_MODE_1,          // CHMOD:001, MODE:11

        // ====================================================================
        // 3. Counter Mode (CTR)
        // ====================================================================
        // Stream cipher. Key Derivation (Mode 2) is forbidden.
        // Decryption is physically the same as encryption (generating keystream),
        // but we expose both for API logic.
        ctr_encrypt           = AES_CR_CHMOD_1,                                          // CHMOD:010, MODE:00
        ctr_decrypt           = AES_CR_CHMOD_1 | AES_CR_MODE_1,                          // CHMOD:010, MODE:10

        // ====================================================================
        // 4. Galois/Counter Mode (GCM) & GMAC
        // ====================================================================
        // Authenticated Encryption. Mode 2 and 4 are forbidden.
        // GMAC uses the same hardware config as GCM (011) but handles data differently.
        gcm_encrypt           = AES_CR_CHMOD_0 | AES_CR_CHMOD_1,                         // CHMOD:011, MODE:00
        gcm_decrypt           = AES_CR_CHMOD_0 | AES_CR_CHMOD_1 | AES_CR_MODE_1,         // CHMOD:011, MODE:10
        
        gmac_encrypt          = gcm_encrypt,                                             // Alias
        gmac_decrypt          = gcm_decrypt,                                             // Alias

        // ====================================================================
        // 5. Counter with CBC-MAC (CCM)
        // ====================================================================
        // Uses Bit 16 for CHMOD[2]. Mode 2 and 4 are forbidden.
        ccm_encrypt           = AES_CR_CHMOD_2,                                          // CHMOD:100, MODE:00
        ccm_decrypt           = AES_CR_CHMOD_2 | AES_CR_MODE_1,                          // CHMOD:100, MODE:10
    };

    enum class Key_size : std::uint32_t
    {
        bits_128 = 0u,
        bits_256 = AES_CR_KEYSIZE,
    };

    void enable();
    void disable();

private:
    AES(std::uint32_t a_idx, AES_TypeDef* a_p_registers, IRQn_Type a_irqn)
        : idx(a_idx)
        , p_registers(a_p_registers)
        , irqn(a_irqn)
    {

    }

    std::uint32_t idx;
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
        return st::arm::m4::wb::rm0434::peripherals::AES(0u, AES1, IRQn_Type::AES1_IRQn);
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
        return st::arm::m4::wb::rm0434::peripherals::AES(1u, AES2, IRQn_Type::AES2_IRQn);
    }
};
#endif
} // namespace xmcu::soc
