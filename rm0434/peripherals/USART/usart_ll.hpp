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

#define XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(ReturnEnumType, LeftEnumType, RightEnumType)            \
                                                                                                         \
    constexpr ReturnEnumType operator|(LeftEnumType left_a, RightEnumType right_a) noexcept              \
    {                                                                                                    \
        return static_cast<ReturnEnumType>(static_cast<std::underlying_type_t<LeftEnumType>>(left_a) |   \
                                           static_cast<std::underlying_type_t<RightEnumType>>(right_a)); \
    }                                                                                                    \
                                                                                                         \
    constexpr ReturnEnumType operator&(LeftEnumType left_a, RightEnumType right_a) noexcept              \
    {                                                                                                    \
        return static_cast<ReturnEnumType>(static_cast<std::underlying_type_t<LeftEnumType>>(left_a) &   \
                                           static_cast<std::underlying_type_t<RightEnumType>>(right_a)); \
    }                                                                                                    \
                                                                                                         \
    constexpr ReturnEnumType operator^(LeftEnumType left_a, RightEnumType right_a) noexcept              \
    {                                                                                                    \
        return static_cast<ReturnEnumType>(static_cast<std::underlying_type_t<LeftEnumType>>(left_a) ^   \
                                           static_cast<std::underlying_type_t<RightEnumType>>(right_a)); \
    }

#define XSOC_USART_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(EnumType)            \
    constexpr EnumType& operator|=(EnumType& left_a, EnumType right_a) noexcept \
    {                                                                           \
        left_a = static_cast<EnumType>(left_a | right_a);                       \
        return left_a;                                                          \
    }                                                                           \
                                                                                \
    constexpr EnumType& operator&=(EnumType& left_a, EnumType right_a) noexcept \
    {                                                                           \
        left_a = static_cast<EnumType>(left_a & right_a);                       \
        return left_a;                                                          \
    }                                                                           \
                                                                                \
    constexpr EnumType& operator^=(EnumType& left_a, EnumType right_a) noexcept \
    {                                                                           \
        left_a = static_cast<EnumType>(left_a ^ right_a);                       \
        return left_a;                                                          \
    }

#define XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS(EnumType)   \
    constexpr EnumType operator~(EnumType right_a) noexcept        \
    {                                                              \
        using Type = std::underlying_type_t<EnumType>;             \
        return static_cast<EnumType>(~static_cast<Type>(right_a)); \
    }                                                              \
    constexpr bool operator!(EnumType right_a) noexcept            \
    {                                                              \
        using Type = std::underlying_type_t<EnumType>;             \
        return static_cast<Type>(right_a) == 0;                    \
    }

#define XSOC_USART_LL_GENERATE_COMPARISON_OPERATORS(LeftEnumType, RightEnumType)            \
    constexpr bool operator==(LeftEnumType left_a, RightEnumType right_a)                   \
    {                                                                                       \
        return (static_cast<std::uint32_t>(left_a) == static_cast<std::uint32_t>(right_a)); \
    }                                                                                       \
    constexpr bool operator==(RightEnumType left_a, LeftEnumType right_a)                   \
    {                                                                                       \
        return (static_cast<std::uint32_t>(left_a) == static_cast<std::uint32_t>(right_a)); \
    }                                                                                       \
    constexpr bool operator!=(LeftEnumType left_a, RightEnumType right_a)                   \
    {                                                                                       \
        return false == (left_a == right_a);                                                \
    }                                                                                       \
    constexpr bool operator!=(RightEnumType left_a, LeftEnumType right_a)                   \
    {                                                                                       \
        return false == (left_a == right_a);                                                \
    }

#define XSOC_USART_LL_GENERATE_BITMASK_OPERATORS_FRIEND(ReturnEnumType, LeftEnumType, RightEnumType)     \
                                                                                                         \
    friend constexpr ReturnEnumType operator|(LeftEnumType left_a, RightEnumType right_a) noexcept       \
    {                                                                                                    \
        return static_cast<ReturnEnumType>(static_cast<std::underlying_type_t<LeftEnumType>>(left_a) |   \
                                           static_cast<std::underlying_type_t<RightEnumType>>(right_a)); \
    }                                                                                                    \
                                                                                                         \
    friend constexpr ReturnEnumType operator&(LeftEnumType left_a, RightEnumType right_a) noexcept       \
    {                                                                                                    \
        return static_cast<ReturnEnumType>(static_cast<std::underlying_type_t<LeftEnumType>>(left_a) &   \
                                           static_cast<std::underlying_type_t<RightEnumType>>(right_a)); \
    }                                                                                                    \
                                                                                                         \
    friend constexpr ReturnEnumType operator^(LeftEnumType left_a, RightEnumType right_a) noexcept       \
    {                                                                                                    \
        return static_cast<ReturnEnumType>(static_cast<std::underlying_type_t<LeftEnumType>>(left_a) ^   \
                                           static_cast<std::underlying_type_t<RightEnumType>>(right_a)); \
    }

#define XSOC_USART_LL_GENERATE_COMPARISON_OPERATORS_FRIEND(LeftEnumType, RightEnumType)     \
    friend constexpr bool operator==(LeftEnumType left_a, RightEnumType right_a)            \
    {                                                                                       \
        return (static_cast<std::uint32_t>(left_a) == static_cast<std::uint32_t>(right_a)); \
    }                                                                                       \
    friend constexpr bool operator==(RightEnumType left_a, LeftEnumType right_a)            \
    {                                                                                       \
        return (static_cast<std::uint32_t>(left_a) == static_cast<std::uint32_t>(right_a)); \
    }                                                                                       \
    friend constexpr bool operator!=(LeftEnumType left_a, RightEnumType right_a)            \
    {                                                                                       \
        return false == (left_a == right_a);                                                \
    }                                                                                       \
    friend constexpr bool operator!=(RightEnumType left_a, LeftEnumType right_a)            \
    {                                                                                       \
        return false == (left_a == right_a);                                                \
    }

#define XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS_FRIEND(EnumType) \
    friend constexpr EnumType operator~(EnumType right_a) noexcept      \
    {                                                                   \
        using Type = std::underlying_type_t<EnumType>;                  \
        return static_cast<EnumType>(~static_cast<Type>(right_a));      \
    }                                                                   \
    friend constexpr bool operator!(EnumType right_a) noexcept          \
    {                                                                   \
        using Type = std::underlying_type_t<EnumType>;                  \
        return static_cast<Type>(right_a) == 0;                         \
    }

namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals::ll {
struct usart : public usart_base
{
public:
    struct CR1
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

        using enum Flag;
        using enum Shift_5;

        enum class Data : std::uint32_t;

        CR1& operator=(Flag value_a)
        {
            this->v = static_cast<Data>(value_a);
            return *this;
        }
        CR1& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }

        operator Data() const
        {
            return this->v;
        }

    private:
        volatile Data v;
    };
    struct CR2
    {
    private:
        struct STOP
        {
            enum class Flag : std::uint32_t
            {
                _1_5 = USART_CR2_STOP_0 | USART_CR2_STOP_1,
                _2_0 = USART_CR2_STOP_1,
                _0_5 = USART_CR2_STOP_0,
                _1_0 = 0x0u
            };
            enum class Mask : std::uint32_t
            {
                mask = 0x3u << USART_CR2_STOP_Pos
            };

            using enum Flag;
            using enum Mask;

            operator Mask() const
            {
                return this->mask;
            }
        };
        struct ABRMOD
        {
            enum class Flag : std::uint32_t
            {
                start_bit = 0x0u,
                falling_edge_to_falling_edge = USART_CR2_ABRMODE_0,
                frame_0x75_detection = USART_CR2_ABRMODE_1,
                frame_0x55_detection = USART_CR2_ABRMODE_0 | USART_CR2_ABRMODE_1
            };
            enum class Mask : std::uint32_t
            {
                mask = 0x3u << USART_CR2_ABRMODE_Pos
            };

            using enum Flag;
            using enum Mask;

            operator Mask() const
            {
                return this->mask;
            }
        };

    public:
        enum class Data : std::uint32_t;

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
        enum class Shift_8 : std::uint32_t
        {
            add = USART_CR2_ADD_Pos
        };

        static STOP stop;
        static ABRMOD abrmod;

        using enum Flag;
        using enum Shift_8;

        CR2& operator=(Flag value_a)
        {
            this->v = static_cast<Data>(value_a);
            return *this;
        }
        CR2& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }

        operator Data() const
        {
            return this->v;
        }

        // STOP
        XSOC_USART_LL_GENERATE_BITMASK_OPERATORS_FRIEND(Data, Flag, STOP::Flag);
        XSOC_USART_LL_GENERATE_BITMASK_OPERATORS_FRIEND(Data, STOP::Flag, Flag);
        XSOC_USART_LL_GENERATE_BITMASK_OPERATORS_FRIEND(Data, Data, STOP::Flag);
        XSOC_USART_LL_GENERATE_BITMASK_OPERATORS_FRIEND(Data, STOP::Flag, Data);
        XSOC_USART_LL_GENERATE_BITMASK_OPERATORS_FRIEND(Data, Data, STOP::Mask);
        XSOC_USART_LL_GENERATE_BITMASK_OPERATORS_FRIEND(Data, STOP::Mask, Data);

        XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS_FRIEND(STOP::Flag);
        XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS_FRIEND(STOP::Mask);

        XSOC_USART_LL_GENERATE_COMPARISON_OPERATORS_FRIEND(Data, STOP::Mask);
        XSOC_USART_LL_GENERATE_COMPARISON_OPERATORS_FRIEND(Data, STOP::Flag);

        // ABRMOD
        XSOC_USART_LL_GENERATE_BITMASK_OPERATORS_FRIEND(Data, Flag, ABRMOD::Flag);
        XSOC_USART_LL_GENERATE_BITMASK_OPERATORS_FRIEND(Data, ABRMOD::Flag, Flag);
        XSOC_USART_LL_GENERATE_BITMASK_OPERATORS_FRIEND(Data, Data, ABRMOD::Flag);
        XSOC_USART_LL_GENERATE_BITMASK_OPERATORS_FRIEND(Data, ABRMOD::Flag, Data);
        XSOC_USART_LL_GENERATE_BITMASK_OPERATORS_FRIEND(Data, Data, ABRMOD::Mask);
        XSOC_USART_LL_GENERATE_BITMASK_OPERATORS_FRIEND(Data, ABRMOD::Mask, Data);

        XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS_FRIEND(ABRMOD::Flag);
        XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS_FRIEND(ABRMOD::Mask);

        XSOC_USART_LL_GENERATE_COMPARISON_OPERATORS_FRIEND(Data, ABRMOD::Mask);
        XSOC_USART_LL_GENERATE_COMPARISON_OPERATORS_FRIEND(Data, ABRMOD::Flag);

    private:
        volatile Data v;
    };
    struct CR3
    {
    private:
        struct TXFTCFG
        {
            enum class Flag : std::uint32_t
            {
                _1_div_8 = 0x0u,
                _1_div_4 = USART_CR3_TXFTCFG_0,
                _1_div_2 = USART_CR3_TXFTCFG_1,
                _3_div_4 = USART_CR3_TXFTCFG_0 | USART_CR3_TXFTCFG_1,
                _7_div_8 = USART_CR3_TXFTCFG_2,
                empty = USART_CR3_TXFTCFG_2 | USART_CR3_TXFTCFG_0
            };
            enum class Mask : std::uint32_t
            {
                mask = USART_CR3_TXFTCFG
            };

            using enum Flag;
            using enum Mask;

            operator Mask() const
            {
                return this->mask;
            }
        };
        struct RXFTCFG
        {
            enum class Flag : std::uint32_t
            {
                _1_div_8 = 0x0u,
                _1_div_4 = USART_CR3_RXFTCFG_0,
                _1_div_2 = USART_CR3_RXFTCFG_1,
                _3_div_4 = USART_CR3_RXFTCFG_0 | USART_CR3_RXFTCFG_1,
                _7_div_8 = USART_CR3_RXFTCFG_2,
                full = USART_CR3_RXFTCFG_2 | USART_CR3_RXFTCFG_0
            };
            enum class Mask : std::uint32_t
            {
                mask = USART_CR3_RXFTCFG
            };

            using enum Flag;
            using enum Mask;

            operator Mask() const
            {
                return this->mask;
            }
        };
        struct WUS
        {
            enum class Flag : std::uint32_t
            {
                address_match = 0x0u,
                start_bit_detection = USART_CR3_WUS_1,
                rxne = USART_CR3_WUS_0 | USART_CR3_WUS_1
            };
            enum class Mask : std::uint32_t
            {
                mask = 0x3u << USART_CR2_STOP_Pos
            };

            using enum Flag;
            using enum Mask;

            operator Mask() const
            {
                return this->mask;
            }
        };

    public:
        enum class Data : std::uint32_t;

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
        enum class Shift_3 : std::uint32_t
        {
            scarcnt = USART_CR3_SCARCNT_Pos
        };

        using enum Flag;
        using enum Shift_3;

        static TXFTCFG txftcfg;
        static RXFTCFG rxftcfg;
        static WUS wus;

        CR3& operator=(Flag value_a)
        {
            this->v = static_cast<Data>(value_a);
            return *this;
        }
        CR3& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }

        operator Data() const
        {
            return this->v;
        }

        // TXFTCFG
        XSOC_USART_LL_GENERATE_BITMASK_OPERATORS_FRIEND(Data, Flag, TXFTCFG::Flag);
        XSOC_USART_LL_GENERATE_BITMASK_OPERATORS_FRIEND(Data, TXFTCFG::Flag, Flag);
        XSOC_USART_LL_GENERATE_BITMASK_OPERATORS_FRIEND(Data, Data, TXFTCFG::Flag);
        XSOC_USART_LL_GENERATE_BITMASK_OPERATORS_FRIEND(Data, TXFTCFG::Flag, Data);
        XSOC_USART_LL_GENERATE_BITMASK_OPERATORS_FRIEND(Data, Data, TXFTCFG::Mask);
        XSOC_USART_LL_GENERATE_BITMASK_OPERATORS_FRIEND(Data, TXFTCFG::Mask, Data);

        XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS_FRIEND(TXFTCFG::Flag);
        XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS_FRIEND(TXFTCFG::Mask);

        XSOC_USART_LL_GENERATE_COMPARISON_OPERATORS_FRIEND(Data, TXFTCFG::Mask);
        XSOC_USART_LL_GENERATE_COMPARISON_OPERATORS_FRIEND(Data, TXFTCFG::Flag);

        // RXFTCFG
        XSOC_USART_LL_GENERATE_BITMASK_OPERATORS_FRIEND(Data, Flag, RXFTCFG::Flag);
        XSOC_USART_LL_GENERATE_BITMASK_OPERATORS_FRIEND(Data, RXFTCFG::Flag, Flag);
        XSOC_USART_LL_GENERATE_BITMASK_OPERATORS_FRIEND(Data, Data, RXFTCFG::Flag);
        XSOC_USART_LL_GENERATE_BITMASK_OPERATORS_FRIEND(Data, RXFTCFG::Flag, Data);
        XSOC_USART_LL_GENERATE_BITMASK_OPERATORS_FRIEND(Data, Data, RXFTCFG::Mask);
        XSOC_USART_LL_GENERATE_BITMASK_OPERATORS_FRIEND(Data, RXFTCFG::Mask, Data);

        XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS_FRIEND(RXFTCFG::Flag);
        XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS_FRIEND(RXFTCFG::Mask);

        XSOC_USART_LL_GENERATE_COMPARISON_OPERATORS_FRIEND(Data, RXFTCFG::Mask);
        XSOC_USART_LL_GENERATE_COMPARISON_OPERATORS_FRIEND(Data, RXFTCFG::Flag);

        // WUS
        XSOC_USART_LL_GENERATE_BITMASK_OPERATORS_FRIEND(Data, Flag, WUS::Flag);
        XSOC_USART_LL_GENERATE_BITMASK_OPERATORS_FRIEND(Data, WUS::Flag, Flag);
        XSOC_USART_LL_GENERATE_BITMASK_OPERATORS_FRIEND(Data, Data, WUS::Flag);
        XSOC_USART_LL_GENERATE_BITMASK_OPERATORS_FRIEND(Data, WUS::Flag, Data);
        XSOC_USART_LL_GENERATE_BITMASK_OPERATORS_FRIEND(Data, Data, WUS::Mask);
        XSOC_USART_LL_GENERATE_BITMASK_OPERATORS_FRIEND(Data, WUS::Mask, Data);

        XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS_FRIEND(WUS::Flag);
        XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS_FRIEND(WUS::Mask);

        XSOC_USART_LL_GENERATE_COMPARISON_OPERATORS_FRIEND(Data, WUS::Mask);
        XSOC_USART_LL_GENERATE_COMPARISON_OPERATORS_FRIEND(Data, WUS::Flag);

    private:
        volatile Data v;
    };
    struct BRR
    {
        enum class Data : std::uint32_t;

        BRR& operator=(std::uint32_t value_a)
        {
            this->v = static_cast<Data>(value_a);
            return *this;
        }

        BRR& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }

        operator Data() const
        {
            return this->v;
        }

    private:
        volatile Data v;
    };
    struct GTPR
    {
        enum class Shift_8 : std::uint32_t
        {
            psc = USART_GTPR_PSC_Pos,
            gt = USART_GTPR_GT_Pos
        };

        using enum Shift_8;

        enum class Data : std::uint32_t;

        GTPR& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }

    private:
        volatile Data v;
    };
    struct RTOR
    {
        enum class Shift_23 : std::uint32_t
        {
            rto = USART_RTOR_RTO_Pos
        };

        enum class Shift_8 : std::uint32_t
        {
            blen = USART_RTOR_BLEN_Pos
        };

        using enum Shift_23;
        using enum Shift_8;

        enum class Data : std::uint32_t;

        RTOR& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }

    private:
        volatile Data v;
    };
    struct RQR
    {
        enum class Flag : std::uint32_t
        {
            abrrq = USART_RQR_ABRRQ,
            sbkrq = USART_RQR_SBKRQ,
            mmrq = USART_RQR_MMRQ,
            rxfrq = USART_RQR_RXFRQ,
            txfrq = USART_RQR_TXFRQ
        };

        using enum Flag;

        enum class Data : std::uint32_t;

        RQR& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
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

    private:
        volatile Data v;
    };
    struct ISR
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

        using enum Flag;

        enum class Data : std::uint32_t;

        ISR()
            : v(static_cast<Data>(0x0u))
        {
        }

        operator Data() const
        {
            return this->v;
        }

    private:
        const volatile Data v;
    };
    struct ICR
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

        using enum Flag;

        enum class Data : std::uint32_t;

        ICR& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
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

    private:
        volatile Data v;
    };
    struct RDR
    {
        enum class Data : std::uint32_t;

        RDR()
            : v(static_cast<Data>(0x0u))
        {
        }

        constexpr operator std::uint32_t() const
        {
            return static_cast<std::uint32_t>(this->v);
        }

    private:
        const volatile Data v;
    };
    struct TDR
    {
        enum class Data : std::uint32_t;

        TDR(Limited<std::uint32_t, 0u, 0x1FF> value_a)
            : v(static_cast<Data>(value_a.get()))
        {
        }

        TDR& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }
        TDR& operator=(std::uint32_t value_a)
        {
            this->v = static_cast<Data>(value_a);
            return *this;
        }

    private:
        volatile Data v;
    };
    struct PRESC
    {
        enum class Flag : std::uint32_t
        {
            _1 = 0x0u,
            _2 = USART_PRESC_PRESCALER_0,
            _4 = USART_PRESC_PRESCALER_1,
            _6 = USART_PRESC_PRESCALER_0 | USART_PRESC_PRESCALER_1,
            _8 = USART_PRESC_PRESCALER_2,
            _10 = USART_PRESC_PRESCALER_0 | USART_PRESC_PRESCALER_2,
            _12 = USART_PRESC_PRESCALER_1 | USART_PRESC_PRESCALER_2,
            _16 = USART_PRESC_PRESCALER_0 | USART_PRESC_PRESCALER_1 | USART_PRESC_PRESCALER_2,
            _32 = USART_PRESC_PRESCALER_3,
            _64 = USART_PRESC_PRESCALER_0 | USART_PRESC_PRESCALER_3,
            _128 = USART_PRESC_PRESCALER_1 | USART_PRESC_PRESCALER_3,
            _256 = USART_PRESC_PRESCALER_0 | USART_PRESC_PRESCALER_1 | USART_PRESC_PRESCALER_3
        };

        using enum Flag;

        enum class Data : std::uint32_t;

        PRESC& operator=(Flag value_a)
        {
            this->v = static_cast<Data>(value_a);
            return *this;
        }

        PRESC& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }

        operator Data() const
        {
            return this->v;
        }

    private:
        Data v;
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
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::CR1::Data, usart::CR1::Flag, usart::CR1::Flag);
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::CR1::Data, usart::CR1::Data, usart::CR1::Data);
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::CR1::Data, usart::CR1::Data, usart::CR1::Flag);
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::CR1::Data, usart::CR1::Flag, usart::CR1::Data);

XSOC_USART_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(usart::CR1::Flag);
XSOC_USART_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(usart::CR1::Data);

XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS(usart::CR1::Flag);
XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS(usart::CR1::Data);
XSOC_USART_LL_GENERATE_COMPARISON_OPERATORS(usart::CR1::Flag, usart::CR1::Data);

constexpr usart::CR1::Data operator<<(Limited<std::uint32_t, 0x0u, 0x1Fu> left_a, usart::CR1::Shift_5 right_a)
{
    return static_cast<usart::CR1::Data>(left_a.get() << static_cast<std::uint32_t>(right_a));
}

// CR2
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::CR2::Data, usart::CR2::Flag, usart::CR2::Flag);
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::CR2::Data, usart::CR2::Data, usart::CR2::Data);
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::CR2::Data, usart::CR2::Data, usart::CR2::Flag);
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::CR2::Data, usart::CR2::Flag, usart::CR2::Data);

XSOC_USART_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(usart::CR2::Flag);
XSOC_USART_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(usart::CR2::Data);

XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS(usart::CR2::Flag);
XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS(usart::CR2::Data);
XSOC_USART_LL_GENERATE_COMPARISON_OPERATORS(usart::CR2::Flag, usart::CR2::Data);

constexpr usart::CR2::Data operator<<(Limited<std::uint32_t, 0x0u, 0xFu> left_a, usart::CR2::Shift_8 right_a)
{
    return static_cast<usart::CR2::Data>(left_a.get() << static_cast<std::uint32_t>(right_a));
}

// CR3
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::CR3::Data, usart::CR3::Flag, usart::CR3::Flag);
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::CR3::Data, usart::CR3::Data, usart::CR3::Data);
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::CR3::Data, usart::CR3::Data, usart::CR3::Flag);
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::CR3::Data, usart::CR3::Flag, usart::CR3::Data);

XSOC_USART_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(usart::CR3::Flag);
XSOC_USART_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(usart::CR3::Data);

XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS(usart::CR3::Flag);
XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS(usart::CR3::Data);
XSOC_USART_LL_GENERATE_COMPARISON_OPERATORS(usart::CR3::Flag, usart::CR3::Data);

constexpr usart::CR3::Data operator<<(Limited<std::uint32_t, 0x0u, 0xFu> left_a, usart::CR3::Shift_3 right_a)
{
    return static_cast<usart::CR3::Data>(left_a.get() << static_cast<std::uint32_t>(right_a));
}

// GTPR
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::GTPR::Data, usart::GTPR::Data, usart::GTPR::Data);
XSOC_USART_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(usart::GTPR::Data);
XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS(usart::GTPR::Data);

constexpr usart::GTPR::Data operator<<(Limited<std::uint32_t, 0x0u, 0xFu> left_a, usart::GTPR::Shift_8 right_a)
{
    return static_cast<usart::GTPR::Data>(left_a.get() << static_cast<std::uint32_t>(right_a));
}

// RTOR
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::RTOR::Data, usart::RTOR::Data, usart::RTOR::Data);
XSOC_USART_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(usart::RTOR::Data);
XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS(usart::RTOR::Data);

constexpr usart::RTOR::Data operator<<(Limited<std::uint32_t, 0x0u, 0xFFFFFFu> left_a, usart::RTOR::Shift_23 right_a)
{
    return static_cast<usart::RTOR::Data>(left_a.get() << static_cast<std::uint32_t>(right_a));
}
constexpr usart::RTOR::Data operator<<(Limited<std::uint32_t, 0x0u, 0xFu> left_a, usart::RTOR::Shift_8 right_a)
{
    return static_cast<usart::RTOR::Data>(left_a.get() << static_cast<std::uint32_t>(right_a));
}

// RQR
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::RQR::Data, usart::RQR::Flag, usart::RQR::Flag);
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::RQR::Data, usart::RQR::Data, usart::RQR::Data);
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::RQR::Data, usart::RQR::Data, usart::RQR::Flag);
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::RQR::Data, usart::RQR::Flag, usart::RQR::Data);

XSOC_USART_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(usart::RQR::Flag);
XSOC_USART_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(usart::RQR::Data);

XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS(usart::RQR::Flag);
XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS(usart::RQR::Data);
XSOC_USART_LL_GENERATE_COMPARISON_OPERATORS(usart::RQR::Flag, usart::RQR::Data);

// ISR
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::ISR::Data, usart::ISR::Flag, usart::ISR::Flag);
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::ISR::Data, usart::ISR::Data, usart::ISR::Data);
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::ISR::Data, usart::ISR::Data, usart::ISR::Flag);
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::ISR::Data, usart::ISR::Flag, usart::ISR::Data);

XSOC_USART_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(usart::ISR::Flag);
XSOC_USART_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(usart::ISR::Data);

XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS(usart::ISR::Flag);
XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS(usart::ISR::Data);
XSOC_USART_LL_GENERATE_COMPARISON_OPERATORS(usart::ISR::Flag, usart::ISR::Data);

// ICR
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::ICR::Data, usart::ICR::Flag, usart::ICR::Flag);
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::ICR::Data, usart::ICR::Data, usart::ICR::Data);
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::ICR::Data, usart::ICR::Data, usart::ICR::Flag);
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::ICR::Data, usart::ICR::Flag, usart::ICR::Data);

XSOC_USART_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(usart::ICR::Flag);
XSOC_USART_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(usart::ICR::Data);

XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS(usart::ICR::Flag);
XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS(usart::ICR::Data);
XSOC_USART_LL_GENERATE_COMPARISON_OPERATORS(usart::ICR::Flag, usart::ICR::Data);

// PRESC
XSOC_USART_LL_GENERATE_COMPARISON_OPERATORS(usart::PRESC::Flag, usart::PRESC::Data);
} // namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals::ll