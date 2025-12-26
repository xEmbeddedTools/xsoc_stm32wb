#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// xmcu
#include <rm0434/peripherals/GPIO/base.hpp>
#include <xmcu/Non_copyable.hpp>

// std
#include <cstdint>
#include <type_traits>

// externals
#include <stm32wbxx.h>

#define XSOC_GPIO_LL_SINGLE_ARG(...) __VA_ARGS__
#define XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(DataType, FlagType, MaskType, PinType, shift_value, mask_value)        \
    constexpr DataType operator<<(FlagType left_a, PinType pin_a)                                                    \
    {                                                                                                                \
        return static_cast<DataType>(static_cast<std::uint32_t>(left_a)                                              \
                                     << (static_cast<std::uint32_t>(pin_a) * shift_value));                          \
    }                                                                                                                \
    constexpr DataType operator<<(MaskType mask_a, PinType pin_a)                                                    \
    {                                                                                                                \
        return static_cast<DataType>(static_cast<std::uint32_t>(mask_a)                                              \
                                     << (static_cast<std::uint32_t>(pin_a) * shift_value));                          \
    }                                                                                                                \
    constexpr FlagType operator>>(DataType mask_a, PinType pin_a)                                                    \
    {                                                                                                                \
        return static_cast<FlagType>(                                                                                \
            (static_cast<std::uint32_t>(mask_a) >> (static_cast<std::uint32_t>(pin_a) * shift_value) & mask_value)); \
    }

#define XSOC_GPIO_LL_GENERATE_BITMASK_OPERATORS(ReturnEnumType, LeftEnumType, RightEnumType)             \
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

#define XSOC_GPIO_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(EnumType)             \
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

#define XSOC_GPIO_LL_GENERATE_BITMASK_UNARY_OPERATORS(EnumType)    \
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

namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals::ll {
struct gpio : public gpio_base
{
public:
    struct MODER
    {
        enum class Flag : std::uint32_t
        {
            input = 0x0u,
            output = 0x1u,
            af = 0x2u,
            analog = 0x3u
        };

        enum class Mask : std::uint32_t
        {
            mask = 0x3u
        };

        using enum Flag;
        using enum Mask;

        enum class Data : std::uint32_t;

        MODER& operator=(Data value_a)
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
    struct OTYPER
    {
        enum class Flag : std::uint32_t
        {
            push_pull = 0x0u,
            open_drain = 0x1u,
        };

        enum class Mask : std::uint32_t
        {
            mask = 0x1u
        };

        using enum Flag;
        using enum Mask;

        enum class Data : std::uint32_t;

        OTYPER& operator=(Data value_a)
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
    struct OSPEEDR
    {
        enum class Flag : std::uint32_t
        {
            low = 0x0u,
            medium = 0x1u,
            high = 0x2u,
            ultra = 0x3u
        };

        enum class Mask : std::uint32_t
        {
            mask = 0x3u
        };

        using enum Flag;
        using enum Mask;

        enum class Data : std::uint32_t;

        OSPEEDR& operator=(Data value_a)
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
    struct PUPDR
    {
        enum class Flag : std::uint32_t
        {
            none = 0x0u,
            pull_up = 0x1u,
            pull_down = 0x2u,
        };

        enum class Mask : std::uint32_t
        {
            mask = 0x3u
        };

        using enum Flag;
        using enum Mask;

        enum class Data : std::uint32_t;

        PUPDR& operator=(Data value_a)
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
    struct IDR
    {
        enum class Flag : std::uint32_t
        {
            low = 0x0u,
            high = 0x1u
        };

        enum class Mask : std::uint32_t
        {
            mask = 0x1u
        };

        using enum Flag;
        using enum Mask;

        enum class Data : std::uint32_t;

        IDR()
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
    struct ODR
    {
        enum class Flag : std::uint32_t
        {
            low = 0x0u,
            high = 0x1u
        };

        enum class Mask : std::uint32_t
        {
            mask = 0x1u
        };

        using enum Flag;
        using enum Mask;

        enum class Data : std::uint32_t;

        ODR& operator=(Data value_a)
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
    struct BSRR
    {
        enum class Flag : std::uint32_t
        {
            high = 0x1u,
            low = 0x2u
        };

        using enum Flag;

        enum class Data : std::uint32_t;

        BSRR& operator=(Data value_a)
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
    struct LCKR
    {
        enum class Flag : std::uint32_t
        {
            unlocked = 0x0u,
            lock = 0x01u,
            key = 0x10u
        };

        enum class Mask : std::uint32_t
        {
            mask = 0x1u
        };

        enum class Data : std::uint32_t;

        using enum Flag;
        using enum Mask;

        LCKR& operator=(Data value_a)
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
    struct AFR
    {
        enum class Flag : std::uint32_t
        {
            af0 = 0x0u,
            af1,
            af2,
            af3,
            af4,
            af5,
            af6,
            af7,
            af8,
            af9,
            af10,
            af11,
            af12,
            af13,
            af14
        };

        enum class Mask : std::uint32_t
        {
            mask = 0xFu
        };

        using enum Flag;
        using enum Mask;

        enum class Data : std::uint32_t;

        AFR& operator=(Data value_a)
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
    struct BRR
    {
        static constexpr std::uint32_t shift_multiplier = 1u;

        enum class Flag : std::uint32_t
        {
            low = 0x0u,
        };

        enum class Mask : std::uint32_t
        {
            mask = 0x1u
        };

        using enum Flag;
        using enum Mask;

        enum class Data : std::uint32_t;

        BRR& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }

    private:
        volatile Data v;
    };

    struct Registers : private Non_copyable
    {
        MODER moder;     // port mode register
        OTYPER otyper;   // port output type register
        OSPEEDR ospeedr; // port output speed register
        PUPDR pupdr;     // port pull-up/pull-down register
        IDR idr;         // port input data register
        ODR odr;         // port output data register
        BSRR bsrr;       // port bit set/reset  register
        LCKR lckr;       // port configuration lock register
        AFR afr[2];      // alternate function registers
        BRR brr;         // bit reset register
    };

    template<typename Port_t> [[nodiscard]] static constexpr Registers* registers() = delete;
};

// MODER
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::MODER::Data,
                                      gpio::MODER::Flag,
                                      gpio::MODER::Mask,
                                      XSOC_GPIO_LL_SINGLE_ARG(xmcu::Limited<std::uint32_t, 0u, 15u>),
                                      2u,
                                      0x3u);
XSOC_GPIO_LL_GENERATE_BITMASK_OPERATORS(gpio::MODER::Data, gpio::MODER::Data, gpio::MODER::Data);
XSOC_GPIO_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(gpio::MODER::Data);

XSOC_GPIO_LL_GENERATE_BITMASK_UNARY_OPERATORS(gpio::MODER::Flag);
XSOC_GPIO_LL_GENERATE_BITMASK_UNARY_OPERATORS(gpio::MODER::Data);
XSOC_GPIO_LL_GENERATE_BITMASK_UNARY_OPERATORS(gpio::MODER::Mask);

// OTYPER
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::OTYPER::Data,
                                      gpio::OTYPER::Flag,
                                      gpio::OTYPER::Mask,
                                      XSOC_GPIO_LL_SINGLE_ARG(xmcu::Limited<std::uint32_t, 0u, 15u>),
                                      1u,
                                      0x1u);
XSOC_GPIO_LL_GENERATE_BITMASK_OPERATORS(gpio::OTYPER::Data, gpio::OTYPER::Data, gpio::OTYPER::Data);
XSOC_GPIO_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(gpio::OTYPER::Data);

XSOC_GPIO_LL_GENERATE_BITMASK_UNARY_OPERATORS(gpio::OTYPER::Flag);
XSOC_GPIO_LL_GENERATE_BITMASK_UNARY_OPERATORS(gpio::OTYPER::Data);
XSOC_GPIO_LL_GENERATE_BITMASK_UNARY_OPERATORS(gpio::OTYPER::Mask);

// OSPEEDR
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::OSPEEDR::Data,
                                      gpio::OSPEEDR::Flag,
                                      gpio::OSPEEDR::Mask,
                                      XSOC_GPIO_LL_SINGLE_ARG(xmcu::Limited<std::uint32_t, 0u, 15u>),
                                      2u,
                                      0x3u);
XSOC_GPIO_LL_GENERATE_BITMASK_OPERATORS(gpio::OSPEEDR::Data, gpio::OSPEEDR::Data, gpio::OSPEEDR::Data);
XSOC_GPIO_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(gpio::OSPEEDR::Data);

XSOC_GPIO_LL_GENERATE_BITMASK_UNARY_OPERATORS(gpio::OSPEEDR::Flag);
XSOC_GPIO_LL_GENERATE_BITMASK_UNARY_OPERATORS(gpio::OSPEEDR::Data);
XSOC_GPIO_LL_GENERATE_BITMASK_UNARY_OPERATORS(gpio::OSPEEDR::Mask);

// PUPDR
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::PUPDR::Data,
                                      gpio::PUPDR::Flag,
                                      gpio::PUPDR::Mask,
                                      XSOC_GPIO_LL_SINGLE_ARG(xmcu::Limited<std::uint32_t, 0u, 15u>),
                                      2u,
                                      0x3u);
XSOC_GPIO_LL_GENERATE_BITMASK_OPERATORS(gpio::PUPDR::Data, gpio::PUPDR::Data, gpio::PUPDR::Data);
XSOC_GPIO_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(gpio::PUPDR::Data);

XSOC_GPIO_LL_GENERATE_BITMASK_UNARY_OPERATORS(gpio::PUPDR::Flag);
XSOC_GPIO_LL_GENERATE_BITMASK_UNARY_OPERATORS(gpio::PUPDR::Data);
XSOC_GPIO_LL_GENERATE_BITMASK_UNARY_OPERATORS(gpio::PUPDR::Mask);

// IDR
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::IDR::Data,
                                      gpio::IDR::Flag,
                                      gpio::IDR::Mask,
                                      XSOC_GPIO_LL_SINGLE_ARG(xmcu::Limited<std::uint32_t, 0u, 15u>),
                                      1u,
                                      0x1u);
XSOC_GPIO_LL_GENERATE_BITMASK_OPERATORS(gpio::IDR::Data, gpio::IDR::Data, gpio::IDR::Data);
XSOC_GPIO_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(gpio::IDR::Data);

XSOC_GPIO_LL_GENERATE_BITMASK_UNARY_OPERATORS(gpio::IDR::Flag);
XSOC_GPIO_LL_GENERATE_BITMASK_UNARY_OPERATORS(gpio::IDR::Data);
XSOC_GPIO_LL_GENERATE_BITMASK_UNARY_OPERATORS(gpio::IDR::Mask);

// ODR
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::ODR::Data,
                                      gpio::ODR::Flag,
                                      gpio::ODR::Mask,
                                      XSOC_GPIO_LL_SINGLE_ARG(xmcu::Limited<std::uint32_t, 0u, 15u>),
                                      1u,
                                      0x1u);
XSOC_GPIO_LL_GENERATE_BITMASK_OPERATORS(gpio::ODR::Data, gpio::ODR::Data, gpio::ODR::Data);
XSOC_GPIO_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(gpio::ODR::Data);

XSOC_GPIO_LL_GENERATE_BITMASK_UNARY_OPERATORS(gpio::ODR::Flag);
XSOC_GPIO_LL_GENERATE_BITMASK_UNARY_OPERATORS(gpio::ODR::Data);
XSOC_GPIO_LL_GENERATE_BITMASK_UNARY_OPERATORS(gpio::ODR::Mask);

// BSRR
constexpr gpio::BSRR::Data operator<<(gpio::BSRR::Flag left_a, xmcu::Limited<std::uint32_t, 0u, 15u> pin_a)
{
    return static_cast<gpio::BSRR::Data>(0x1u << (pin_a * static_cast<std::uint32_t>(left_a)));
}
constexpr gpio::BSRR::Data operator|(gpio::BSRR::Data left_a, gpio::BSRR::Data right_a)
{
    return static_cast<gpio::BSRR::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}
constexpr gpio::BSRR::Data operator&(gpio::BSRR::Data left_a, gpio::BSRR::Data right_a)
{
    return static_cast<gpio::BSRR::Data>((static_cast<std::uint32_t>(left_a) & static_cast<std::uint32_t>(right_a)));
}
constexpr gpio::BSRR::Flag operator~(gpio::BSRR::Flag flag_a)
{
    return static_cast<gpio::BSRR::Flag>(~static_cast<std::uint32_t>(flag_a));
}
constexpr gpio::BSRR::Data operator~(gpio::BSRR::Data flag_a)
{
    return static_cast<gpio::BSRR::Data>(~static_cast<std::uint32_t>(flag_a));
}
constexpr gpio::BSRR::Data operator^(gpio::BSRR::Data left_a, std::uint32_t right_a)
{
    return static_cast<gpio::BSRR::Data>(static_cast<std::uint32_t>(left_a) ^ right_a);
}

// LCKR
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::LCKR::Data,
                                      gpio::LCKR::Flag,
                                      gpio::LCKR::Mask,
                                      XSOC_GPIO_LL_SINGLE_ARG(xmcu::Limited<std::uint32_t, 0u, 15u>),
                                      1u,
                                      0x1u);
XSOC_GPIO_LL_GENERATE_BITMASK_OPERATORS(gpio::LCKR::Data, gpio::LCKR::Data, gpio::LCKR::Data);
XSOC_GPIO_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(gpio::LCKR::Data);

XSOC_GPIO_LL_GENERATE_BITMASK_UNARY_OPERATORS(gpio::LCKR::Flag);
XSOC_GPIO_LL_GENERATE_BITMASK_UNARY_OPERATORS(gpio::LCKR::Data);
XSOC_GPIO_LL_GENERATE_BITMASK_UNARY_OPERATORS(gpio::LCKR::Mask);

// AFR
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::AFR::Data,
                                      gpio::AFR::Flag,
                                      gpio::AFR::Mask,
                                      XSOC_GPIO_LL_SINGLE_ARG(xmcu::Limited<std::uint32_t, 0u, 15u>),
                                      0xFu,
                                      0x4u);
XSOC_GPIO_LL_GENERATE_BITMASK_OPERATORS(gpio::AFR::Data, gpio::AFR::Data, gpio::AFR::Data);
XSOC_GPIO_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(gpio::AFR::Data);

XSOC_GPIO_LL_GENERATE_BITMASK_UNARY_OPERATORS(gpio::AFR::Flag);
XSOC_GPIO_LL_GENERATE_BITMASK_UNARY_OPERATORS(gpio::AFR::Data);
XSOC_GPIO_LL_GENERATE_BITMASK_UNARY_OPERATORS(gpio::AFR::Mask);

// BRR
constexpr gpio::BRR::Data operator<<(gpio::BRR::Flag left_a, xmcu::Limited<std::uint32_t, 0u, 15u> pin_a)
{
    return static_cast<gpio::BRR::Data>(static_cast<std::uint32_t>(left_a) << pin_a);
}
XSOC_GPIO_LL_GENERATE_BITMASK_OPERATORS(gpio::BRR::Data, gpio::BRR::Data, gpio::BRR::Data);
XSOC_GPIO_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(gpio::BRR::Data);

XSOC_GPIO_LL_GENERATE_BITMASK_UNARY_OPERATORS(gpio::BRR::Flag);
XSOC_GPIO_LL_GENERATE_BITMASK_UNARY_OPERATORS(gpio::BRR::Data);
XSOC_GPIO_LL_GENERATE_BITMASK_UNARY_OPERATORS(gpio::BRR::Mask);

#if defined XMCU_GPIOA_PRESENT
template<> [[nodiscard]] constexpr gpio::Registers* gpio::registers<gpio::A>()
{
    return reinterpret_cast<gpio::Registers*>(GPIOA_BASE);
}

// MODER
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::MODER::Data, gpio::MODER::Flag, gpio::MODER::Mask, gpio::A, 2u, 0x3u);

// OTYPER
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::OTYPER::Data, gpio::OTYPER::Flag, gpio::OTYPER::Mask, gpio::A, 1u, 0x1u);

// OSPEEDR
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::OSPEEDR::Data, gpio::OSPEEDR::Flag, gpio::OSPEEDR::Mask, gpio::A, 2u, 0x3u);

// PUPDR
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::PUPDR::Data, gpio::PUPDR::Flag, gpio::PUPDR::Mask, gpio::A, 2u, 0x3u);

// IDR
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::IDR::Data, gpio::IDR::Flag, gpio::IDR::Mask, gpio::A, 1u, 0x1u);

// ODR
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::ODR::Data, gpio::ODR::Flag, gpio::ODR::Mask, gpio::A, 1u, 0x1u);

// BSRR
constexpr gpio::BSRR::Data operator<<(gpio::BSRR::Flag left_a, gpio::A pin_a)
{
    return static_cast<gpio::BSRR::Data>(0x1u
                                         << (static_cast<std::uint32_t>(pin_a) * static_cast<std::uint32_t>(left_a)));
}

// LCKR
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::LCKR::Data, gpio::LCKR::Flag, gpio::LCKR::Mask, gpio::A, 1u, 0x1u);

// AFR
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::AFR::Data, gpio::AFR::Flag, gpio::AFR::Mask, gpio::A, 4u, 0xFu);

// BRR
constexpr gpio::BRR::Data operator<<(gpio::BRR::Flag left_a, gpio::A pin_a)
{
    return static_cast<gpio::BRR::Data>(static_cast<std::uint32_t>(left_a) << (static_cast<std::uint32_t>(pin_a)));
}
#endif
#if defined XMCU_GPIOB_PRESENT
template<> [[nodiscard]] constexpr gpio::Registers* gpio::registers<gpio::B>()
{
    return reinterpret_cast<gpio::Registers*>(GPIOB_BASE);
}

// MODER
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::MODER::Data, gpio::MODER::Flag, gpio::MODER::Mask, gpio::B, 2u, 0x3u);

// OTYPER
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::OTYPER::Data, gpio::OTYPER::Flag, gpio::OTYPER::Mask, gpio::B, 1u, 0x1u);

// OSPEEDR
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::OSPEEDR::Data, gpio::OSPEEDR::Flag, gpio::OSPEEDR::Mask, gpio::B, 2u, 0x3u);

// PUPDR
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::PUPDR::Data, gpio::PUPDR::Flag, gpio::PUPDR::Mask, gpio::B, 2u, 0x3u);

// IDR
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::IDR::Data, gpio::IDR::Flag, gpio::IDR::Mask, gpio::B, 1u, 0x1u);

// ODR
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::ODR::Data, gpio::ODR::Flag, gpio::ODR::Mask, gpio::B, 1u, 0x1u);

// BSRR
constexpr gpio::BSRR::Data operator<<(gpio::BSRR::Flag left_a, gpio::B pin_a)
{
    return static_cast<gpio::BSRR::Data>(0x1u
                                         << (static_cast<std::uint32_t>(pin_a) * static_cast<std::uint32_t>(left_a)));
}

// LCKR
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::LCKR::Data, gpio::LCKR::Flag, gpio::LCKR::Mask, gpio::B, 1u, 0x1u);

// AFR
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::AFR::Data, gpio::AFR::Flag, gpio::AFR::Mask, gpio::B, 4u, 0xFu);

// BRR
constexpr gpio::BRR::Data operator<<(gpio::BRR::Flag left_a, gpio::B pin_a)
{
    return static_cast<gpio::BRR::Data>(static_cast<std::uint32_t>(left_a) << (static_cast<std::uint32_t>(pin_a)));
}
#endif
#if defined XMCU_GPIOC_PRESENT
template<> [[nodiscard]] constexpr gpio::Registers* gpio::registers<gpio::C>()
{
    return reinterpret_cast<gpio::Registers*>(GPIOC_BASE);
}

// MODER
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::MODER::Data, gpio::MODER::Flag, gpio::MODER::Mask, gpio::C, 2u, 0x3u);

// OTYPER
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::OTYPER::Data, gpio::OTYPER::Flag, gpio::OTYPER::Mask, gpio::C, 1u, 0x1u);

// OSPEEDR
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::OSPEEDR::Data, gpio::OSPEEDR::Flag, gpio::OSPEEDR::Mask, gpio::C, 2u, 0x3u);

// PUPDR
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::PUPDR::Data, gpio::PUPDR::Flag, gpio::PUPDR::Mask, gpio::C, 2u, 0x3u);

// IDR
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::IDR::Data, gpio::IDR::Flag, gpio::IDR::Mask, gpio::C, 1u, 0x1u);

// ODR
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::ODR::Data, gpio::ODR::Flag, gpio::ODR::Mask, gpio::C, 1u, 0x1u);

// BSRR
constexpr gpio::BSRR::Data operator<<(gpio::BSRR::Flag left_a, gpio::C pin_a)
{
    return static_cast<gpio::BSRR::Data>(0x1u
                                         << (static_cast<std::uint32_t>(pin_a) * static_cast<std::uint32_t>(left_a)));
}

// LCKR
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::LCKR::Data, gpio::LCKR::Flag, gpio::LCKR::Mask, gpio::C, 1u, 0x1u);

// AFR
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::AFR::Data, gpio::AFR::Flag, gpio::AFR::Mask, gpio::C, 4u, 0xFu);

// BRR
constexpr gpio::BRR::Data operator<<(gpio::BRR::Flag left_a, gpio::C pin_a)
{
    return static_cast<gpio::BRR::Data>(static_cast<std::uint32_t>(left_a) << (static_cast<std::uint32_t>(pin_a)));
}

#endif
#if defined XMCU_GPIOE_PRESENT
template<> [[nodiscard]] constexpr gpio::Registers* gpio::registers<gpio::E>()
{
    return reinterpret_cast<gpio::Registers*>(GPIOE_BASE);
}

// MODER
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::MODER::Data, gpio::MODER::Flag, gpio::MODER::Mask, gpio::E, 2u, 0x3u);

// OTYPER
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::OTYPER::Data, gpio::OTYPER::Flag, gpio::OTYPER::Mask, gpio::E, 1u, 0x1u);

// OSPEEDR
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::OSPEEDR::Data, gpio::OSPEEDR::Flag, gpio::OSPEEDR::Mask, gpio::E, 2u, 0x3u);

// PUPDR
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::PUPDR::Data, gpio::PUPDR::Flag, gpio::PUPDR::Mask, gpio::E, 2u, 0x3u);

// IDR
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::IDR::Data, gpio::IDR::Flag, gpio::IDR::Mask, gpio::E, 1u, 0x1u);

// ODR
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::ODR::Data, gpio::ODR::Flag, gpio::ODR::Mask, gpio::E, 1u, 0x1u);

// BSRR
constexpr gpio::BSRR::Data operator<<(gpio::BSRR::Flag left_a, gpio::E pin_a)
{
    return static_cast<gpio::BSRR::Data>(0x1u
                                         << (static_cast<std::uint32_t>(pin_a) * static_cast<std::uint32_t>(left_a)));
}

// LCKR
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::LCKR::Data, gpio::LCKR::Flag, gpio::LCKR::Mask, gpio::E, 1u, 0x1u);

// AFR
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::AFR::Data, gpio::AFR::Flag, gpio::AFR::Mask, gpio::E, 4u, 0xFu);

// BRR
constexpr gpio::BRR::Data operator<<(gpio::BRR::Flag left_a, gpio::E pin_a)
{
    return static_cast<gpio::BRR::Data>(static_cast<std::uint32_t>(left_a) << (static_cast<std::uint32_t>(pin_a)));
}
#endif
#if defined XMCU_GPIOH_PRESENT
template<> [[nodiscard]] constexpr gpio::Registers* gpio::registers<gpio::H>()
{
    return reinterpret_cast<gpio::Registers*>(GPIOH_BASE);
}

// MODER
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::MODER::Data, gpio::MODER::Flag, gpio::MODER::Mask, gpio::H, 2u, 0x3u);

// OTYPER
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::OTYPER::Data, gpio::OTYPER::Flag, gpio::OTYPER::Mask, gpio::H, 1u, 0x1u);

// OSPEEDR
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::OSPEEDR::Data, gpio::OSPEEDR::Flag, gpio::OSPEEDR::Mask, gpio::H, 2u, 0x3u);

// PUPDR
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::PUPDR::Data, gpio::PUPDR::Flag, gpio::PUPDR::Mask, gpio::H, 2u, 0x3u);

// IDR
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::IDR::Data, gpio::IDR::Flag, gpio::IDR::Mask, gpio::H, 1u, 0x1u);

// ODR
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::ODR::Data, gpio::ODR::Flag, gpio::ODR::Mask, gpio::H, 1u, 0x1u);

// BSRR
constexpr gpio::BSRR::Data operator<<(gpio::BSRR::Flag left_a, gpio::H pin_a)
{
    return static_cast<gpio::BSRR::Data>(0x1u
                                         << (static_cast<std::uint32_t>(pin_a) * static_cast<std::uint32_t>(left_a)));
}

// LCKR
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::LCKR::Data, gpio::LCKR::Flag, gpio::LCKR::Mask, gpio::H, 1u, 0x1u);

// AFR
XSOC_GPIO_LL_GENERATE_SHIFT_OPERATORS(gpio::AFR::Data, gpio::AFR::Flag, gpio::AFR::Mask, gpio::H, 4u, 0xFu);

// BRR
constexpr gpio::BRR::Data operator<<(gpio::BRR::Flag left_a, gpio::H pin_a)
{
    return static_cast<gpio::BRR::Data>(static_cast<std::uint32_t>(left_a) << (static_cast<std::uint32_t>(pin_a)));
}
#endif
} // namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals::ll

namespace xmcu {
template<> [[nodiscard]] constexpr bool
bit::is<soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::IDR, xmcu::Limited<std::uint32_t, 0u, 15u>>(
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::IDR a_register,
    xmcu::Limited<std::uint32_t, 0u, 15u> a_index)
{
    using ll_gpio = soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio;

    const std::uint32_t flag = 0x1u << static_cast<std::uint32_t>(a_index);
    return flag == (static_cast<std::uint32_t>(static_cast<ll_gpio::IDR::Data>(a_register)) & flag);
}

template<> [[nodiscard]] constexpr bool
bit::is_any<soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::IDR, xmcu::Limited<std::uint32_t, 0x0u, 0xFFFFu>>(
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::IDR a_register,
    xmcu::Limited<std::uint32_t, 0x0u, 0xFFFFu> a_mask)
{
    using ll_gpio = soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio;
    return 0u != (static_cast<std::uint32_t>(static_cast<ll_gpio::IDR::Data>(a_register)) & a_mask);
}

template<> constexpr void
bit::set<soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::ODR, xmcu::Limited<std::uint32_t, 0u, 15u>>(
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::ODR* a_p_register,
    xmcu::Limited<std::uint32_t, 0u, 15u> a_index)
{
    using ll_gpio = soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio;
    (*a_p_register) = (*a_p_register) | static_cast<ll_gpio::ODR::Data>(0x1u << static_cast<std::uint32_t>(a_index));
}

template<> constexpr void
bit::clear<soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::ODR, xmcu::Limited<std::uint32_t, 0u, 15u>>(
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::ODR* a_p_register,
    xmcu::Limited<std::uint32_t, 0u, 15u> a_index)
{
    using ll_gpio = soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio;
    (*a_p_register) = (*a_p_register) & static_cast<ll_gpio::ODR::Data>(~(0x1u << static_cast<std::uint32_t>(a_index)));
}

template<> constexpr void
bit::toggle<soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::ODR, xmcu::Limited<std::uint32_t, 0u, 15u>>(
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::ODR* a_p_register,
    xmcu::Limited<std::uint32_t, 0u, 15u> a_index)
{
    using ll_gpio = soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio;
    (*a_p_register) = (*a_p_register) ^ static_cast<ll_gpio::ODR::Data>(0x1u << (a_index));
}

#if defined XMCU_GPIOA_PRESENT
template<> [[nodiscard]] constexpr bool bit::is<soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::IDR,
                                                soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::A>(
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::IDR a_register,
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::A a_index)
{
    using ll_gpio = soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio;
    const std::uint32_t flag = 0x1u << static_cast<std::uint32_t>(a_index);
    return static_cast<ll_gpio::IDR::Data>(flag) ==
           (static_cast<ll_gpio::IDR::Data>(a_register) & static_cast<ll_gpio::IDR::Data>(flag));
}

template<> [[nodiscard]] constexpr bool bit::is_any<soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::IDR,
                                                    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::A>(
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::IDR a_register,
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::A a_mask)
{
    using ll_gpio = soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio;
    return 0u != static_cast<std::uint32_t>(a_register & static_cast<ll_gpio::IDR::Data>(a_mask));
}

template<> constexpr void bit::set<soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::ODR,
                                   soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::A>(
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::ODR* a_p_register,
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::A a_index)
{
    using ll_gpio = soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio;

    (*a_p_register) = (*a_p_register) | static_cast<ll_gpio::ODR::Data>(0x1u << static_cast<std::uint32_t>(a_index));
}

template<> constexpr void bit::clear<soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::ODR,
                                     soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::A>(
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::ODR* a_p_register,
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::A a_index)
{
    using ll_gpio = soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio;

    (*a_p_register) = (*a_p_register) & static_cast<ll_gpio::ODR::Data>(~(0x1u << static_cast<std::uint32_t>(a_index)));
}

template<> constexpr void bit::toggle<soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::ODR,
                                      soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::A>(
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::ODR* a_p_register,
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::A a_index)
{
    using ll_gpio = soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio;
    (*a_p_register) = (*a_p_register) ^ static_cast<ll_gpio::ODR::Data>(0x1u << static_cast<std::uint32_t>(a_index));
}
#endif
#if defined XMCU_GPIOB_PRESENT
template<> [[nodiscard]] constexpr bool bit::is<soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::IDR,
                                                soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::B>(
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::IDR a_register,
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::B a_index)
{
    using ll_gpio = soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio;
    const std::uint32_t flag = 0x1u << static_cast<std::uint32_t>(a_index);
    return static_cast<ll_gpio::IDR::Data>(flag) ==
           (static_cast<ll_gpio::IDR::Data>(a_register) & static_cast<ll_gpio::IDR::Data>(flag));
}

template<> [[nodiscard]] constexpr bool bit::is_any<soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::IDR,
                                                    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::B>(
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::IDR a_register,
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::B a_mask)
{
    using ll_gpio = soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio;
    return 0u != static_cast<std::uint32_t>(a_register & static_cast<ll_gpio::IDR::Data>(a_mask));
}

template<> constexpr void bit::set<soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::ODR,
                                   soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::B>(
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::ODR* a_p_register,
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::B a_index)
{
    using ll_gpio = soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio;

    (*a_p_register) = (*a_p_register) | static_cast<ll_gpio::ODR::Data>(0x1u << static_cast<std::uint32_t>(a_index));
}

template<> constexpr void bit::clear<soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::ODR,
                                     soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::B>(
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::ODR* a_p_register,
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::B a_index)
{
    using ll_gpio = soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio;

    (*a_p_register) = (*a_p_register) & static_cast<ll_gpio::ODR::Data>(~(0x1u << static_cast<std::uint32_t>(a_index)));
}

template<> constexpr void bit::toggle<soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::ODR,
                                      soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::B>(
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::ODR* a_p_register,
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::B a_index)
{
    using ll_gpio = soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio;
    (*a_p_register) = (*a_p_register) ^ static_cast<ll_gpio::ODR::Data>(0x1u << static_cast<std::uint32_t>(a_index));
}
#endif
#if defined XMCU_GPIOC_PRESENT
template<> [[nodiscard]] constexpr bool bit::is<soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::IDR,
                                                soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::C>(
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::IDR a_register,
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::C a_index)
{
    using ll_gpio = soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio;
    const std::uint32_t flag = 0x1u << static_cast<std::uint32_t>(a_index);
    return static_cast<ll_gpio::IDR::Data>(flag) ==
           (static_cast<ll_gpio::IDR::Data>(a_register) & static_cast<ll_gpio::IDR::Data>(flag));
}

template<> [[nodiscard]] constexpr bool bit::is_any<soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::IDR,
                                                    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::C>(
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::IDR a_register,
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::C a_mask)
{
    using ll_gpio = soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio;
    return 0u != static_cast<std::uint32_t>(a_register & static_cast<ll_gpio::IDR::Data>(a_mask));
}

template<> constexpr void bit::set<soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::ODR,
                                   soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::C>(
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::ODR* a_p_register,
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::C a_index)
{
    using ll_gpio = soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio;

    (*a_p_register) = (*a_p_register) | static_cast<ll_gpio::ODR::Data>(0x1u << static_cast<std::uint32_t>(a_index));
}

template<> constexpr void bit::clear<soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::ODR,
                                     soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::C>(
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::ODR* a_p_register,
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::C a_index)
{
    using ll_gpio = soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio;

    (*a_p_register) = (*a_p_register) & static_cast<ll_gpio::ODR::Data>(~(0x1u << static_cast<std::uint32_t>(a_index)));
}

template<> constexpr void bit::toggle<soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::ODR,
                                      soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::C>(
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::ODR* a_p_register,
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::C a_index)
{
    using ll_gpio = soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio;
    (*a_p_register) = (*a_p_register) ^ static_cast<ll_gpio::ODR::Data>(0x1u << static_cast<std::uint32_t>(a_index));
}
#endif
#if defined XMCU_GPIOE_PRESENT
template<> [[nodiscard]] constexpr bool bit::is<soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::IDR,
                                                soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::E>(
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::IDR a_register,
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::E a_index)
{
    using ll_gpio = soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio;
    const std::uint32_t flag = 0x1u << static_cast<std::uint32_t>(a_index);
    return static_cast<ll_gpio::IDR::Data>(flag) ==
           (static_cast<ll_gpio::IDR::Data>(a_register) & static_cast<ll_gpio::IDR::Data>(flag));
}

template<> [[nodiscard]] constexpr bool bit::is_any<soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::IDR,
                                                    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::E>(
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::IDR a_register,
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::E a_mask)
{
    using ll_gpio = soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio;
    return 0u != static_cast<std::uint32_t>(a_register & static_cast<ll_gpio::IDR::Data>(a_mask));
}

template<> constexpr void bit::set<soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::ODR,
                                   soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::E>(
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::ODR* a_p_register,
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::E a_index)
{
    using ll_gpio = soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio;

    (*a_p_register) = (*a_p_register) | static_cast<ll_gpio::ODR::Data>(0x1u << static_cast<std::uint32_t>(a_index));
}

template<> constexpr void bit::clear<soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::ODR,
                                     soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::E>(
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::ODR* a_p_register,
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::E a_index)
{
    using ll_gpio = soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio;

    (*a_p_register) = (*a_p_register) & static_cast<ll_gpio::ODR::Data>(~(0x1u << static_cast<std::uint32_t>(a_index)));
}

template<> constexpr void bit::toggle<soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::ODR,
                                      soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::E>(
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::ODR* a_p_register,
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::E a_index)
{
    using ll_gpio = soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio;
    (*a_p_register) = (*a_p_register) ^ static_cast<ll_gpio::ODR::Data>(0x1u << static_cast<std::uint32_t>(a_index));
}
#endif
#if defined XMCU_GPIOH_PRESENT
template<> [[nodiscard]] constexpr bool bit::is<soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::IDR,
                                                soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::H>(
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::IDR a_register,
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::H a_index)
{
    using ll_gpio = soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio;
    const std::uint32_t flag = 0x1u << static_cast<std::uint32_t>(a_index);
    return static_cast<ll_gpio::IDR::Data>(flag) ==
           (static_cast<ll_gpio::IDR::Data>(a_register) & static_cast<ll_gpio::IDR::Data>(flag));
}

template<> [[nodiscard]] constexpr bool bit::is_any<soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::IDR,
                                                    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::H>(
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::IDR a_register,
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::H a_mask)
{
    using ll_gpio = soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio;
    return 0u != static_cast<std::uint32_t>(a_register & static_cast<ll_gpio::IDR::Data>(a_mask));
}

template<> constexpr void bit::set<soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::ODR,
                                   soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::H>(
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::ODR* a_p_register,
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::H a_index)
{
    using ll_gpio = soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio;

    (*a_p_register) = (*a_p_register) | static_cast<ll_gpio::ODR::Data>(0x1u << static_cast<std::uint32_t>(a_index));
}

template<> constexpr void bit::clear<soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::ODR,
                                     soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::H>(
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::ODR* a_p_register,
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::H a_index)
{
    using ll_gpio = soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio;

    (*a_p_register) = (*a_p_register) & static_cast<ll_gpio::ODR::Data>(~(0x1u << static_cast<std::uint32_t>(a_index)));
}

template<> constexpr void bit::toggle<soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::ODR,
                                      soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::H>(
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::ODR* a_p_register,
    soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::H a_index)
{
    using ll_gpio = soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio;
    (*a_p_register) = (*a_p_register) ^ static_cast<ll_gpio::ODR::Data>(0x1u << static_cast<std::uint32_t>(a_index));
}
#endif
} // namespace xmcu
