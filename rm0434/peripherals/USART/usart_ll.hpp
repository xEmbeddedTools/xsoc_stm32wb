#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// CMSIS
#include <stm32wbxx.h>

// soc
#include <rm0434/peripherals/USART/base.hpp>

// xmcu
#include <xmcu/Limited.hpp>
#include <xmcu/Non_copyable.hpp>
#include <xmcu/non_constructible.hpp>

namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals::ll {
struct usart : public usart_base
{
private:
    struct cr1_descriptor : private xmcu::non_constructible
    {
        enum class Flag : std::uint32_t
        {
            none = 0x0u,
            ue = USART_CR1_UE,
            uesm = USART_CR1_UESM,
            re = USART_CR1_RE,
            te = USART_CR1_TE,
            idleie = USART_CR1_IDLEIE,
            rxneie = USART_CR1_RXNEIE,
            tcie = USART_CR1_TCIE,
            txeie = USART_CR1_TXEIE,
            peie = USART_CR1_PEIE,
            ps = USART_CR1_PS,
            pce = USART_CR1_PCE,
            wake = USART_CR1_WAKE,
            m0 = USART_CR1_M0,
            mme = USART_CR1_MME,
            cmie = USART_CR1_CMIE,
            over8 = USART_CR1_OVER8,
            rtoie = USART_CR1_RTOIE,
            eobie = USART_CR1_EOBIE,
            m1 = USART_CR1_M1,
            fifoen = USART_CR1_FIFOEN,
            txfeie = USART_CR1_TXFEIE,
            rxffie = USART_CR1_RXFFIE
        };

        enum class Shift_5 : std::uint32_t
        {
            deat = USART_CR1_DEAT_Pos,
            dedt = USART_CR1_DEDT_Pos
        };
    };
    struct cr2_descriptor : private xmcu::non_constructible
    {
        enum class Flag : std::uint32_t
        {
            none = 0x0u,
            slven = USART_CR2_SLVEN,
            dis_nss = USART_CR2_DIS_NSS,
            addm7 = USART_CR2_ADDM7,
            lbdl = USART_CR2_LBDL,
            lbdie = USART_CR2_LBDIE,
            lbcl = USART_CR2_LBCL,
            cpha = USART_CR2_CPHA,
            cpol = USART_CR2_CPOL,
            clken = USART_CR2_CLKEN,
            linen = USART_CR2_LINEN,
            swap = USART_CR2_SWAP,
            rxinv = USART_CR2_RXINV,
            txinv = USART_CR2_TXINV,
            datainv = USART_CR2_DATAINV,
            msbfirst = USART_CR2_MSBFIRST,
            abren = USART_CR2_ABREN,
            rtoen = USART_CR2_RTOEN
        };

        enum class Shift_2 : std::uint32_t
        {
            stop = USART_CR2_STOP_Pos,
            abrmod = USART_CR2_ABRMODE_Pos
        };

        enum class Shift_8 : std::uint32_t
        {
            add = USART_CR2_ADD_Pos
        };
    };
    struct cr3_descriptor : private xmcu::non_constructible
    {
        enum class Flag : std::uint32_t
        {
            none = 0x0u,
            eie = USART_CR3_EIE,
            iren = USART_CR3_IREN,
            irlp = USART_CR3_IRLP,
            hdsel = USART_CR3_HDSEL,
            nack = USART_CR3_NACK,
            scen = USART_CR3_SCEN,
            dmar = USART_CR3_DMAR,
            dmat = USART_CR3_DMAT,
            rtse = USART_CR3_RTSE,
            ctse = USART_CR3_CTSE,
            ctsie = USART_CR3_CTSIE,
            onebit = USART_CR3_ONEBIT,
            ovrdis = USART_CR3_OVRDIS,
            ddre = USART_CR3_DDRE,
            dem = USART_CR3_DEM,
            dep = USART_CR3_DEP,
            wufie = USART_CR3_WUFIE,
            txftie = USART_CR3_TXFTIE,
            tcbgtie = USART_CR3_TCBGTIE,
        };

        enum class Shift_2 : std::uint32_t
        {
            wus = USART_CR3_WUS_Pos,
            rxftcfg = USART_CR3_RXFTCFG_Pos,
            txftcfg = USART_CR3_TXFTCFG_Pos
        };
        enum class Shift_3 : std::uint32_t
        {
            scarcnt = USART_CR3_SCARCNT_Pos
        };
    };
    struct brr_descriptor : private xmcu::non_constructible
    {
        using Data = std::uint32_t;
    };
    struct gtpr_descriptor : private xmcu::non_constructible
    {
        enum class Data : std::uint32_t;

        enum class Shift_8 : std::uint32_t
        {
            psc = USART_GTPR_PSC_Pos,
            gt = USART_GTPR_GT_Pos
        };
    };
    struct rtor_descriptor : private xmcu::non_constructible
    {
        enum class Data : std::uint32_t;

        enum class Shift_23 : std::uint32_t
        {
            rto = USART_RTOR_RTO_Pos
        };

        enum class Shift_8 : std::uint32_t
        {
            blen = USART_RTOR_BLEN_Pos
        };
    };
    struct rqr_descriptor : private xmcu::non_constructible
    {
        enum class Flag : std::uint32_t
        {
            abrrq = USART_RQR_ABRRQ,
            sbkrq = USART_RQR_SBKRQ,
            mmrq = USART_RQR_MMRQ,
            rxfrq = USART_RQR_RXFRQ,
            txfrq = USART_RQR_TXFRQ
        };
    };
    struct isr_descriptor : private xmcu::non_constructible
    {
        enum class Flag : std::uint32_t
        {
            none = 0x0u,
            pe = USART_ISR_PE,
            fe = USART_ISR_FE,
            ne = USART_ISR_NE,
            ore = USART_ISR_ORE,
            idle = USART_ISR_IDLE,
            rxne = USART_ISR_RXNE,
            rxfne = USART_ISR_RXNE_RXFNE,
            tc = USART_ISR_TC,
            txfnf = USART_ISR_TXE_TXFNF,
            txne = USART_ISR_TXE,
            lbdf = USART_ISR_LBDF,
            ctsif = USART_ISR_CTSIF,
            cts = USART_ISR_CTS,
            rtof = USART_ISR_RTOF,
            eobf = USART_ISR_EOBF,
            udr = USART_ISR_UDR,
            abre = USART_ISR_ABRE,
            abrf = USART_ISR_ABRF,
            busy = USART_ISR_BUSY,
            cmf = USART_ISR_CMF,
            sbkf = USART_ISR_SBKF,
            rwu = USART_ISR_RWU,
            wuf = USART_ISR_WUF,
            teack = USART_ISR_TEACK,
            reack = USART_ISR_REACK,
            txfe = USART_ISR_TXE_TXFNF,
            txe = USART_ISR_TXE,
            rxff = USART_ISR_RXFF,
            tcbgt = USART_ISR_TCBGT,
            rxft = USART_ISR_RXFT,
            txft = USART_ISR_TXFT
        };
    };
    struct icr_descriptor : private xmcu::non_constructible
    {
        enum class Flag : std::uint32_t
        {
            none = 0x0u,
            pecf = USART_ICR_PECF,
            fecf = USART_ICR_FECF,
            necf = USART_ICR_NECF,
            orecf = USART_ICR_ORECF,
            idlecf = USART_ICR_IDLECF,
            txfecf = USART_ICR_TXFECF,
            tccf = USART_ICR_TCCF,
            tcbgtcf = USART_ICR_TCBGTCF,
            lbdcf = USART_ICR_LBDCF,
            ctscf = USART_ICR_CTSCF,
            rtocf = USART_ICR_RTOCF,
            eobcf = USART_ICR_EOBCF,
            udrcf = USART_ICR_UDRCF,
            cmcf = USART_ICR_CMCF,
            wucf = USART_ICR_WUCF
        };
    };
    struct rdr_descriptor : private xmcu::non_constructible
    {
        using Data = std::uint32_t;
    };
    struct tdr_descriptor : private xmcu::non_constructible
    {
        using Data = std::uint32_t;
    };
    struct presc_descriptor : private xmcu::non_constructible
    {
        enum class Data : std::uint32_t;

        enum class Flag : std::uint32_t
        {
            none = 0x0u
        };

        enum class Shift_4 : std::uint32_t
        {
            prescaler = USART_PRESC_PRESCALER_Pos,
        };
    };

    /*
     * w - write, r - read, c - clear
     */
    template<typename desc, typename Type> struct Reg_wrc
    {
        using Data = Type;

        Reg_wrc()
            : v(static_cast<volatile Data>(0x0u))
        {
        }

        Reg_wrc(Data data_a)
            : v(data_a)
        {
        }
        Reg_wrc(const volatile Reg_wrc& other_a)
            : v(other_a.v)
        {
        }

    protected:
        volatile Data v;
    };
    template<typename desc, typename Data> struct Reg_r
    {
        Reg_r()
            : v(static_cast<volatile Data>(0x0u))
        {
        }

        Reg_r(Data data_a)
            : v(data_a)
        {
        }
        Reg_r(const volatile Reg_r& other_a)
            : v(other_a.v)
        {
        }

    protected:
        const volatile Data v;
    };

public:
    struct CR1 : public Reg_wrc<cr1_descriptor, cr1_descriptor::Flag>
    {
        using Flag = cr1_descriptor::Flag;
        using Shift_5 = cr1_descriptor::Shift_5;

        using enum Flag;
        using enum Shift_5;

        using Data = Reg_wrc<cr1_descriptor, cr1_descriptor::Flag>::Data;

        CR1(Flag flag_a)
            : Reg_wrc<cr1_descriptor, cr1_descriptor::Flag>(flag_a)
        {
        }

        CR1& operator=(Flag value_a)
        {
            this->v = value_a;
            return *this;
        }

        operator Data() const
        {
            return this->v;
        }
    };
    struct CR2 : public Reg_wrc<cr2_descriptor, cr2_descriptor::Flag>
    {
        using Flag = cr2_descriptor::Flag;
        using Shift_2 = cr2_descriptor::Shift_2;
        using Shift_8 = cr2_descriptor::Shift_8;

        using enum Flag;
        using enum Shift_2;
        using enum Shift_8;

        using Data = Reg_wrc<cr2_descriptor, cr2_descriptor::Flag>::Data;

        CR2(Flag flag_a)
            : Reg_wrc<cr2_descriptor, cr2_descriptor::Flag>(flag_a)
        {
        }

        CR2& operator=(Flag value_a)
        {
            this->v = value_a;
            return *this;
        }

        operator Data() const
        {
            return this->v;
        }
    };
    struct CR3 : public Reg_wrc<cr3_descriptor, cr3_descriptor::Flag>
    {
        using Flag = cr3_descriptor::Flag;
        using Shift_2 = cr3_descriptor::Shift_2;
        using Shift_3 = cr3_descriptor::Shift_3;

        using enum Flag;
        using enum Shift_2;
        using enum Shift_3;

        using Data = Reg_wrc<cr3_descriptor, cr3_descriptor::Flag>::Data;

        CR3(Flag data_a)
            : Reg_wrc<cr3_descriptor, cr3_descriptor::Flag>(data_a)
        {
        }

        CR3& operator=(Flag value_a)
        {
            this->v = value_a;
            return *this;
        }

        operator Data() const
        {
            return this->v;
        }
    };
    struct BRR : public Reg_wrc<brr_descriptor, std::uint32_t>
    {
        BRR& operator=(Limited<std::uint32_t, 0x0u, 0xFFFFu> value_a)
        {
            this->v = (value_a.get());
            return *this;
        }
    };
    struct GTPR : public Reg_wrc<gtpr_descriptor, gtpr_descriptor::Data>
    {
        using Shift_8 = gtpr_descriptor::Shift_8;

        using enum Shift_8;

        using Data = Reg_wrc<gtpr_descriptor, gtpr_descriptor::Data>::Data;

        GTPR(Data data_a)
            : Reg_wrc<gtpr_descriptor, gtpr_descriptor::Data>(data_a)
        {
        }

        GTPR& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }

        operator Data() const
        {
            return this->v;
        }
    };
    struct RTOR : public Reg_wrc<rtor_descriptor, rtor_descriptor::Data>
    {
        using Shift_23 = rtor_descriptor::Shift_23;
        using Shift_8 = rtor_descriptor::Shift_8;

        using enum Shift_23;
        using enum Shift_8;

        using Data = Reg_wrc<rtor_descriptor, rtor_descriptor::Data>::Data;

        RTOR(Data data_a)
            : Reg_wrc<rtor_descriptor, rtor_descriptor::Data>(data_a)
        {
        }

        RTOR& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }

        operator Data() const
        {
            return this->v;
        }
    };
    struct RQR : public Reg_wrc<rqr_descriptor, rqr_descriptor::Flag>
    {
        using Flag = rqr_descriptor::Flag;

        using enum Flag;

        using Data = Reg_wrc<rqr_descriptor, rqr_descriptor::Flag>::Data;

        RQR(Flag flag_a)
            : Reg_wrc<rqr_descriptor, rqr_descriptor::Flag>(flag_a)
        {
        }

        RQR& operator=(Flag value_a)
        {
            this->v = static_cast<Data>(value_a);
            return *this;
        }

        operator Data() const
        {
            return this->v;
        }
    };
    struct ISR : public Reg_r<isr_descriptor, isr_descriptor::Flag>
    {
        using Flag = isr_descriptor::Flag;

        using enum Flag;

        using Data = Reg_wrc<isr_descriptor, isr_descriptor::Flag>::Data;

        ISR(Flag flag_a)
            : Reg_r<isr_descriptor, isr_descriptor::Flag>(flag_a)
        {
        }

        operator Data() const
        {
            return this->v;
        }
    };
    struct ICR : public Reg_wrc<icr_descriptor, icr_descriptor::Flag>
    {
        using Flag = icr_descriptor::Flag;

        using enum Flag;

        using Data = Reg_wrc<icr_descriptor, icr_descriptor::Flag>::Data;

        ICR(Flag flag_a)
            : Reg_wrc<icr_descriptor, icr_descriptor::Flag>(flag_a)
        {
        }

        ICR& operator=(Flag value_a)
        {
            this->v = static_cast<Data>(value_a);
            return *this;
        }

        operator Data() const
        {
            return this->v;
        }
    };
    struct RDR : public Reg_r<rdr_descriptor, rdr_descriptor::Data>
    {
        using Data = rdr_descriptor::Data;

        operator RDR::Data() const
        {
            return this->v;
        }
    };
    struct TDR : public Reg_wrc<tdr_descriptor, tdr_descriptor::Data>
    {
        using Data = tdr_descriptor::Data;

        TDR(Data data_a)
            : Reg_wrc<tdr_descriptor, tdr_descriptor::Data>(data_a)
        {
        }
        TDR(Limited<std::uint32_t, 0u, 0x1FF> value_a)
            : Reg_wrc<tdr_descriptor, tdr_descriptor::Data>(static_cast<Data>(value_a.get()))
        {
        }

        TDR& operator=(Data value_a)
        {
            this->v = static_cast<Reg_wrc<tdr_descriptor, tdr_descriptor::Data>::Data>(value_a);
            return *this;
        }

        operator Data() const
        {
            return this->v;
        }
    };
    struct PRESC : public Reg_wrc<presc_descriptor, presc_descriptor::Data>
    {
        using Flag = presc_descriptor::Flag;
        using Shift_4 = presc_descriptor::Shift_4;

        using enum Flag;
        using enum Shift_4;

        using Data = Reg_wrc<presc_descriptor, presc_descriptor::Data>::Data;

        PRESC(Data data_a)
            : Reg_wrc<presc_descriptor, presc_descriptor::Data>(data_a)
        {
        }

        PRESC& operator=(Flag value_a)
        {
            this->v = static_cast<Data>(value_a);
            return *this;
        }

        operator Data() const
        {
            return this->v;
        }
    };

    struct Registers : private xmcu::Non_copyable
    {
        CR1 cr1;     // control register 1
        CR2 cr2;     // control register 2
        CR3 cr3;     // control register 3
        BRR brr;     // baudrate register
        GTPR gtpr;   // guard time and prescaler register
        RTOR rtor;   // receiver timeout register
        RQR rqr;     // request register
        ISR isr;     // interrupt and status register
        ICR icr;     // interrupt flag clear register
        RDR rdr;     // receive data register
        TDR tdr;     // transmit data register
        PRESC presc; // prescaler register,
    };

    template<typename Port> [[nodiscard]] static constexpr Registers* registers() = delete;
};

#if defined XMCU_USART1_PRESENT
template<> [[nodiscard]] constexpr usart::Registers* usart::registers<usart::_1>()
{
    return reinterpret_cast<usart::Registers*>(USART1_BASE);
}
#endif

// CR1
constexpr usart::CR1::Data operator|(usart::CR1::Data left_a, usart::CR1::Data right_a)
{
    return static_cast<usart::CR1::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}
constexpr usart::CR1::Data operator&(usart::CR1::Data left_a, usart::CR1::Data right_a)
{
    return static_cast<usart::CR1::Data>(static_cast<std::uint32_t>(left_a) & static_cast<std::uint32_t>(right_a));
}

constexpr usart::CR1::Data& operator|=(usart::CR1::Data& left_a, usart::CR1::Data right_a)
{
    left_a = left_a | right_a;
    return left_a;
}
constexpr usart::CR1::Data& operator&=(usart::CR1::Data& left_a, usart::CR1::Data right_a)
{
    left_a = (left_a & right_a);
    return left_a;
}

constexpr usart::CR1& operator|=(usart::CR1& left_a, usart::CR1::Data right_a)
{
    left_a = left_a | right_a;
    return left_a;
}
constexpr usart::CR1& operator&=(usart::CR1& left_a, usart::CR1::Data right_a)
{
    left_a = (left_a & right_a);
    return left_a;
}

constexpr usart::CR1::Data operator~(usart::CR1::Data left_a)
{
    return static_cast<usart::CR1::Data>(~static_cast<std::uint32_t>(left_a));
}

constexpr usart::CR1::Data operator<<(Limited<std::uint32_t, 0x0u, 0x1Fu> left_a, usart::CR1::Shift_5 right_a)
{
    return static_cast<usart::CR1::Data>(left_a.get() << static_cast<std::uint32_t>(right_a));
}

// CR2
constexpr usart::CR2::Data operator|(usart::CR2::Data left_a, usart::CR2::Data right_a)
{
    return static_cast<usart::CR2::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}
constexpr usart::CR2::Data operator&(usart::CR2::Data left_a, usart::CR2::Data right_a)
{
    return static_cast<usart::CR2::Data>(static_cast<std::uint32_t>(left_a) & static_cast<std::uint32_t>(right_a));
}

constexpr usart::CR2::Data& operator|=(usart::CR2::Data& left_a, usart::CR2::Data right_a)
{
    left_a = left_a | right_a;
    return left_a;
}
constexpr usart::CR2::Data& operator&=(usart::CR2::Data& left_a, usart::CR2::Data right_a)
{
    left_a = (left_a & right_a);
    return left_a;
}

constexpr usart::CR2& operator|=(usart::CR2& left_a, usart::CR2::Data right_a)
{
    left_a = left_a | right_a;
    return left_a;
}
constexpr usart::CR2& operator&=(usart::CR2& left_a, usart::CR2::Data right_a)
{
    left_a = (left_a & right_a);
    return left_a;
}

constexpr usart::CR2::Data operator~(usart::CR2::Data left_a)
{
    return static_cast<usart::CR2::Data>(~static_cast<std::uint32_t>(left_a));
}

constexpr usart::CR2::Data operator<<(Limited<std::uint32_t, 0x0u, 0x3u> left_a, usart::CR2::Shift_2 right_a)
{
    return static_cast<usart::CR2::Data>(left_a.get() << static_cast<std::uint32_t>(right_a));
}
constexpr usart::CR2::Data operator<<(Limited<std::uint32_t, 0x0u, 0xFu> left_a, usart::CR2::Shift_8 right_a)
{
    return static_cast<usart::CR2::Data>(left_a.get() << static_cast<std::uint32_t>(right_a));
}

// CR3
constexpr usart::CR3::Data operator|(usart::CR3::Data left_a, usart::CR3::Data right_a)
{
    return static_cast<usart::CR3::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}
constexpr usart::CR3::Data operator&(usart::CR3::Data left_a, usart::CR3::Data right_a)
{
    return static_cast<usart::CR3::Data>(static_cast<std::uint32_t>(left_a) & static_cast<std::uint32_t>(right_a));
}

constexpr usart::CR3::Data& operator|=(usart::CR3::Data& left_a, usart::CR3::Data right_a)
{
    left_a = left_a | right_a;
    return left_a;
}
constexpr usart::CR3::Data& operator&=(usart::CR3::Data& left_a, usart::CR3::Data right_a)
{
    left_a = (left_a & right_a);
    return left_a;
}

constexpr usart::CR3& operator|=(usart::CR3& left_a, usart::CR3::Data right_a)
{
    left_a = left_a | right_a;
    return left_a;
}
constexpr usart::CR3& operator&=(usart::CR3& left_a, usart::CR3::Data right_a)
{
    left_a = (left_a & right_a);
    return left_a;
}

constexpr usart::CR3::Data operator~(usart::CR3::Data left_a)
{
    return static_cast<usart::CR3::Data>(~static_cast<std::uint32_t>(left_a));
}

constexpr usart::CR3::Data operator<<(Limited<std::uint32_t, 0x0u, 0x3u> left_a, usart::CR3::Shift_2 right_a)
{
    return static_cast<usart::CR3::Data>(left_a.get() << static_cast<std::uint32_t>(right_a));
}
constexpr usart::CR3::Data operator<<(Limited<std::uint32_t, 0x0u, 0xFu> left_a, usart::CR3::Shift_3 right_a)
{
    return static_cast<usart::CR3::Data>(left_a.get() << static_cast<std::uint32_t>(right_a));
}

// GTPR
constexpr usart::GTPR::Data operator<<(Limited<std::uint32_t, 0x0u, 0xFu> left_a, usart::GTPR::Shift_8 right_a)
{
    return static_cast<usart::GTPR::Data>(left_a.get() << static_cast<std::uint32_t>(right_a));
}
constexpr usart::GTPR::Data operator~(usart::GTPR::Data left_a)
{
    return static_cast<usart::GTPR::Data>(~static_cast<std::uint32_t>(left_a));
}

// RTOR
constexpr usart::RTOR::Data operator<<(Limited<std::uint32_t, 0x0u, 0xFFFFFFu> left_a, usart::RTOR::Shift_23 right_a)
{
    return static_cast<usart::RTOR::Data>(left_a.get() << static_cast<std::uint32_t>(right_a));
}
constexpr usart::RTOR::Data operator<<(Limited<std::uint32_t, 0x0u, 0xFu> left_a, usart::RTOR::Shift_8 right_a)
{
    return static_cast<usart::RTOR::Data>(left_a.get() << static_cast<std::uint32_t>(right_a));
}
constexpr usart::RTOR::Data operator~(usart::RTOR::Data left_a)
{
    return static_cast<usart::RTOR::Data>(~static_cast<std::uint32_t>(left_a));
}

// RQR
constexpr usart::RQR::Data operator|(usart::RQR::Data left_a, usart::RQR::Data right_a)
{
    return static_cast<usart::RQR::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}
constexpr usart::RQR::Data operator&(usart::RQR::Data left_a, usart::RQR::Data right_a)
{
    return static_cast<usart::RQR::Data>(static_cast<std::uint32_t>(left_a) & static_cast<std::uint32_t>(right_a));
}
constexpr usart::RQR::Data operator~(usart::RQR::Data left_a)
{
    return static_cast<usart::RQR::Data>(~static_cast<std::uint32_t>(left_a));
}

// ISR
constexpr usart::ISR::Data operator|(usart::ISR::Data left_a, usart::ISR::Data right_a)
{
    return static_cast<usart::ISR::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}
constexpr usart::ISR::Data operator&(usart::ISR::Data left_a, usart::ISR::Data right_a)
{
    return static_cast<usart::ISR::Data>(static_cast<std::uint32_t>(left_a) & static_cast<std::uint32_t>(right_a));
}
constexpr usart::ISR::Data operator~(usart::ISR::Data left_a)
{
    return static_cast<usart::ISR::Data>(~static_cast<std::uint32_t>(left_a));
}

// ICR
constexpr usart::ICR::Data operator|(usart::ICR::Data left_a, usart::ICR::Data right_a)
{
    return static_cast<usart::ICR::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}
constexpr usart::ICR::Data operator&(usart::ICR::Data left_a, usart::ICR::Data right_a)
{
    return static_cast<usart::ICR::Data>(static_cast<std::uint32_t>(left_a) & static_cast<std::uint32_t>(right_a));
}

constexpr usart::ICR::Data& operator|=(usart::ICR::Data& left_a, usart::ICR::Data right_a)
{
    left_a = left_a | right_a;
    return left_a;
}
constexpr usart::ICR::Data& operator&=(usart::ICR::Data& left_a, usart::ICR::Data right_a)
{
    left_a = (left_a & right_a);
    return left_a;
}

constexpr usart::ICR& operator|=(usart::ICR& left_a, usart::ICR::Data right_a)
{
    left_a = left_a | right_a;
    return left_a;
}
constexpr usart::ICR& operator&=(usart::ICR& left_a, usart::ICR::Data right_a)
{
    left_a = (left_a & right_a);
    return left_a;
}

constexpr usart::ICR::Data operator~(usart::ICR::Data left_a)
{
    return static_cast<usart::ICR::Data>(~static_cast<std::uint32_t>(left_a));
}

// PRESC
constexpr usart::PRESC::Data operator<<(Limited<std::uint32_t, 0x0u, 0xBu> left_a, usart::PRESC::Shift_4 right_a)
{
    return static_cast<usart::PRESC::Data>(left_a.get() << static_cast<std::uint32_t>(right_a));
}
constexpr usart::PRESC::Data operator~(usart::PRESC::Data left_a)
{
    return static_cast<usart::PRESC::Data>(~static_cast<std::uint32_t>(left_a));
}
} // namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals::ll

namespace xmcu {
// CR1
template<> [[nodiscard]] inline xmcu::soc::st::arm::m4::wb::rm0434::peripherals::ll::usart::CR1::Data
bit::flag::get(xmcu::soc::st::arm::m4::wb::rm0434::peripherals::ll::usart::CR1 register_a,
               xmcu::soc::st::arm::m4::wb::rm0434::peripherals::ll::usart::CR1::Flag mask_a)
{
    using ll_usart = xmcu::soc::st::arm::m4::wb::rm0434::peripherals::ll::usart;
    return static_cast<ll_usart::CR1::Data>(static_cast<ll_usart::CR1::Flag>(register_a) & mask_a);
}
// CR2
template<> [[nodiscard]] inline xmcu::soc::st::arm::m4::wb::rm0434::peripherals::ll::usart::CR2::Flag
bit::flag::get(xmcu::soc::st::arm::m4::wb::rm0434::peripherals::ll::usart::CR2 register_a,
               xmcu::soc::st::arm::m4::wb::rm0434::peripherals::ll::usart::CR2::Flag mask_a)
{
    return static_cast<xmcu::soc::st::arm::m4::wb::rm0434::peripherals::ll::usart::CR2::Flag>(register_a & mask_a);
}
// CR3
template<> [[nodiscard]] inline xmcu::soc::st::arm::m4::wb::rm0434::peripherals::ll::usart::CR3::Flag
bit::flag::get(xmcu::soc::st::arm::m4::wb::rm0434::peripherals::ll::usart::CR3 register_a,
               xmcu::soc::st::arm::m4::wb::rm0434::peripherals::ll::usart::CR3::Flag mask_a)
{
    return static_cast<xmcu::soc::st::arm::m4::wb::rm0434::peripherals::ll::usart::CR3::Flag>(register_a & mask_a);
}
// ISR
template<> [[nodiscard]] inline xmcu::soc::st::arm::m4::wb::rm0434::peripherals::ll::usart::ISR::Data
bit::flag::get(xmcu::soc::st::arm::m4::wb::rm0434::peripherals::ll::usart::ISR register_a,
               xmcu::soc::st::arm::m4::wb::rm0434::peripherals::ll::usart::ISR::Flag mask_a)
{
    using ll_usart = xmcu::soc::st::arm::m4::wb::rm0434::peripherals::ll::usart;
    return static_cast<ll_usart::ISR::Data>(static_cast<ll_usart::ISR::Flag>(register_a) & mask_a);
}
// ICR
template<> [[nodiscard]] inline xmcu::soc::st::arm::m4::wb::rm0434::peripherals::ll::usart::ICR::Data
bit::flag::get(xmcu::soc::st::arm::m4::wb::rm0434::peripherals::ll::usart::ICR register_a,
               xmcu::soc::st::arm::m4::wb::rm0434::peripherals::ll::usart::ICR::Flag mask_a)
{
    using ll_usart = xmcu::soc::st::arm::m4::wb::rm0434::peripherals::ll::usart;
    return static_cast<ll_usart::ICR::Data>(static_cast<ll_usart::ICR::Flag>(register_a) & mask_a);
}
} // namespace xmcu
