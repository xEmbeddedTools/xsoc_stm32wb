#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// externals
#include <stm32wbxx.h>

// xmcu
#include <rm0434/clocks/sources/hsi16.hpp>
#include <rm0434/clocks/sources/hsi48.hpp>
#include <rm0434/clocks/sources/lse.hpp>
#include <rm0434/clocks/sources/lsi.hpp>
#include <rm0434/clocks/sources/pll.hpp>
#include <rm0434/peripherals/GPIO/gpio_ll.hpp>
#include <rm0434/rcc.hpp>
#include <rm0434/system/mcu/mcu.hpp>
#include <soc/peripheral.hpp>
#include <soc/st/arm/IRQ_config.hpp>
#include <xmcu/Limited.hpp>
#include <xmcu/Non_copyable.hpp>
#include <xmcu/bit.hpp>
#include <xmcu/non_constructible.hpp>
#include <xmcu/various.hpp>

// debug
#include <xmcu/assertion.hpp>

// TODO: use with xmcu_assert in gpio pin initialization to check if pin can actually be used
#define BV(x) (1 << (x))
#if defined(XMCU_SOC_MODEL_STM32WB35CEU6A)
#define GPIOA_PIN_MASK                                                                                          \
    (BV(0) | BV(1) | BV(2) | BV(3) | BV(4) | BV(5) | BV(6) | BV(7) | BV(8) | BV(9) | BV(10) | BV(11) | BV(12) | \
     BV(13) | BV(14) | BV(15))
#define GPIOB_PIN_MASK (BV(0) | BV(1) | BV(2) | BV(3) | BV(4) | BV(5) | BV(6) | BV(7) | BV(8) | BV(9))
#define GPIOC_PIN_MASK (BV(14) | BV(15))
#define GPIOE_PIN_MASK (BV(4))
#define GPIOH_PIN_MASK (BV(3))
#elif defined(XMCU_SOC_MODEL_STM32WB55RGV)
#define GPIOA_PIN_MASK                                                                                          \
    (BV(0) | BV(1) | BV(2) | BV(3) | BV(4) | BV(5) | BV(6) | BV(7) | BV(8) | BV(9) | BV(10) | BV(11) | BV(12) | \
     BV(13) | BV(14) | BV(15))
#define GPIOB_PIN_MASK                                                                                          \
    (BV(0) | BV(1) | BV(2) | BV(3) | BV(4) | BV(5) | BV(6) | BV(7) | BV(8) | BV(9) | BV(10) | BV(11) | BV(12) | \
     BV(13) | BV(14) | BV(15))
#define GPIOC_PIN_MASK (BV(1) | BV(2) | BV(3) | BV(4) | BV(5) | BV(6) | BV(10) | BV(11) | BV(12))
#define GPIOD_PIN_MASK (BV(0) | BV(1))
#define GPIOE_PIN_MASK (BV(4))
#define GPIOH_PIN_MASK (BV(3))
#else
#error "Undefined GPIO pins for device"
#endif
#undef BVs

namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals {
class GPIO : private Non_copyable
{
public:
    enum class Level : std::uint32_t
    {
        low = static_cast<std::uint32_t>(ll::gpio::ODR::low),
        high = static_cast<std::uint32_t>(ll::gpio::ODR::high)
    };
    enum class Type : std::uint32_t
    {
        push_pull = static_cast<std::uint32_t>(ll::gpio::OTYPER::push_pull),
        open_drain = static_cast<std::uint32_t>(ll::gpio::OTYPER::open_drain),
    };
    enum class Pull : std::uint32_t
    {
        none = static_cast<std::uint32_t>(ll::gpio::PUPDR::none),
        up = static_cast<std::uint32_t>(ll::gpio::PUPDR::pull_up),
        down = static_cast<std::uint32_t>(ll::gpio::PUPDR::pull_down),
    };
    enum class Speed : std::uint32_t
    {
        low = static_cast<std::uint32_t>(ll::gpio::OSPEEDR::low),
        medium = static_cast<std::uint32_t>(ll::gpio::OSPEEDR::medium),
        high = static_cast<std::uint32_t>(ll::gpio::OSPEEDR::high),
        ultra = static_cast<std::uint32_t>(ll::gpio::OSPEEDR::ultra),
    };

    class Out : private xmcu::Non_copyable
    {
    public:
        struct Enable_config
        {
            Type type;
            Pull pull;
            Speed speed;
        };

        class Pin : private xmcu::Non_copyable
        {
        public:
            Pin()
                : p_port(nullptr)
                , id(0xFFu)
            {
            }

            void set_level(Level a_level);
            void toggle_level();

            void set_type(Type a_type);
            void set_pull(Pull a_pull);
            void set_speed(Speed a_speed);

            Level get_level() const;
            Type get_type() const;
            Pull get_pull() const;
            Speed get_speed() const;

            GPIO* get_port() const
            {
                return this->p_port;
            }
            std::uint32_t get_id() const
            {
                return this->id;
            }

        private:
            GPIO* p_port;
            std::uint32_t id;

            friend Out;
        };
        class Bus : private Non_copyable
        {
        public:
            Bus()
                : p_port(nullptr)
                , id_start(0xFFu)
                , id_end(0xFF)
            {
            }

            void set_value(std::uint32_t a_value);

            void set_type(Type a_type);
            void set_pull(Pull a_pull);
            void set_speed(Speed a_speed);

            GPIO* get_port() const
            {
                return this->p_port;
            }

        private:
            GPIO* p_port;
            std::uint8_t id_start, id_end;

            friend Out;
        };

        void enable(Limited<std::uint32_t, 0, 15> a_id, const Enable_config& a_enable_config, Pin* a_p_pin = nullptr);
        void disable(Limited<std::uint32_t, 0, 15> a_id);
        void disable(Pin* p_pin);

    private:
        Out(GPIO* a_p_port)
            : p_port(a_p_port)
        {
        }

        GPIO* p_port;
        friend GPIO;
    };
    class In : private xmcu::Non_copyable
    {
    public:
        class Pin : private xmcu::Non_copyable
        {
        public:
            Pin()
                : p_port(nullptr)
                , id(0xFFu)
            {
            }

            void set_pull(Pull a_pull);

            GPIO::Pull get_pull() const;
            GPIO::Level get_level() const;

            GPIO* get_port() const
            {
                return this->p_port;
            }
            std::uint32_t get_id() const
            {
                return this->id;
            }

        private:
            GPIO* p_port;
            std::uint32_t id;

            friend In;
        };

        void enable(Limited<std::uint32_t, 0, 15> a_id, Pull a_pull, Pin* a_p_pin = nullptr);
        void disable(Limited<std::uint32_t, 0, 15> a_id);
        void disable(Pin* p_pin);

    private:
        In(GPIO* a_p_port)
            : p_port(a_p_port)
        {
        }

        GPIO* p_port;

        friend GPIO;
    };
    class Analog : private xmcu::Non_copyable
    {
    public:
        class Pin : private xmcu::Non_copyable
        {
        public:
            Pin()
                : p_port(nullptr)
                , id(0xFFu)
            {
            }

            void set_pull(Pull a_pull);

            GPIO::Pull get_pull() const;

            GPIO* get_port() const
            {
                return this->p_port;
            }
            std::uint32_t get_id() const
            {
                return this->id;
            }

        private:
            GPIO* p_port;
            std::uint32_t id;

            friend Analog;
        };

        void enable(Limited<std::uint32_t, 0, 15>, Pull a_pull, Pin* a_p_pin = nullptr);
        void disable(Limited<std::uint32_t, 0, 15> a_id);
        void disable(Pin* p_pin);

    private:
        Analog(GPIO* a_p_port)
            : p_port(a_p_port)
        {
        }

        GPIO* p_port;

        friend GPIO;
    };
    class Alternate_function : private xmcu::Non_copyable
    {
    public:
        struct Enable_config
        {
            Type type = various::get_enum_incorrect_value<Type>();
            Pull pull = various::get_enum_incorrect_value<Pull>();
            Speed speed = various::get_enum_incorrect_value<Speed>();
        };

        class Pin : private xmcu::Non_copyable
        {
        public:
            Pin()
                : p_port(nullptr)
                , id(0xFFu)
            {
            }

            void set_type(Type a_type);
            void set_pull(Pull a_pull);
            void set_speed(Speed a_speed);

            GPIO::Type get_type() const;
            GPIO::Pull get_pull() const;
            GPIO::Speed get_speed() const;

            std::uint32_t get_function() const
            {
                return this->function;
            }
            GPIO* get_port() const
            {
                return this->p_port;
            }
            std::uint32_t get_id() const
            {
                return this->id;
            }

        private:
            GPIO* p_port;
            std::uint32_t id;

            std::uint32_t function;

            friend Alternate_function;
        };

        template<typename Periph_t, std::uint32_t periph_id = std::numeric_limits<std::uint32_t>::max()> void
        enable(Limited<std::uint32_t, 0, 15>, const Enable_config& a_enable_config, Pin* a_p_pin = nullptr) = delete;
        void disable(Limited<std::uint32_t, 0, 15> a_id);
        void disable(Pin* p_pin);

    private:
        Alternate_function(GPIO* a_p_port)
            : p_port(a_p_port)
        {
        }

        void enable(Limited<std::uint32_t, 0, 15>,
                    const Enable_config& a_enable_config,
                    std::uint32_t a_function,
                    Pin* a_p_pin);

        GPIO* p_port;

        friend GPIO;
    };

    class Interrupt : private xmcu::Non_copyable
    {
    private:
        template<std::uint8_t> struct H
        {
        };

    public:
        enum class Type : std::uint32_t
        {
            interrupt,
            event
        };

        enum class Trigger_flag : std::uint32_t
        {
            rising = 0x1,
            falling = 0x2,
        };

        struct Id
        {
            static H<0> _0;
            static H<1> _1;
            static H<2> _2;
            static H<3> _3;
            static H<4> _4;
            static H<5> _5_9;
            static H<6> _10_15;
        };

        struct Callback
        {
            using Function = void (*)(std::uint32_t a_pin, void* a_p_user_data);

            Function function = nullptr;
            void* p_user_data = nullptr;
        };

        Interrupt(Interrupt&&) = default;
        Interrupt& operator=(Interrupt&&) = default;

        Interrupt()
            : idx(std::numeric_limits<decltype(this->idx)>::max())
            , irqn(static_cast<IRQn_Type>(std::numeric_limits<int32_t>::max()))
        {
        }

        Interrupt(H<0>)
            : idx(0)
            , irqn(EXTI0_IRQn)
        {
        }
        Interrupt(H<1>)
            : idx(1)
            , irqn(EXTI1_IRQn)
        {
        }
        Interrupt(H<2>)
            : idx(2)
            , irqn(EXTI2_IRQn)
        {
        }
        Interrupt(H<3>)
            : idx(3)
            , irqn(EXTI3_IRQn)
        {
        }
        Interrupt(H<4>)
            : idx(4)
            , irqn(EXTI4_IRQn)
        {
        }
        Interrupt(H<5>)
            : idx(5)
            , irqn(EXTI9_5_IRQn)
        {
        }
        Interrupt(H<6>)
            : idx(6)
            , irqn(EXTI15_10_IRQn)
        {
        }

        ~Interrupt()
        {
            if (0x0 != NVIC_GetEnableIRQ(this->irqn))
            {
                this->disable();
            }
        }

        void enable(const Callback& a_callback, const IRQ_config& a_irq_config);
        void disable();

        void attach(const GPIO& a_port, std::uint32_t a_pin, Trigger_flag a_trigger, Type a_type);
        void attach(const GPIO::In::Pin& a_pin, Trigger_flag a_trigger, Type a_type);
        void attach(const GPIO::Out::Pin& a_pin, Trigger_flag a_trigger, Type a_type);
        void attach(const GPIO::Alternate_function::Pin& a_pin, Trigger_flag a_trigger, Type a_type);

        void deattach(const GPIO& a_port, std::uint32_t a_pin);
        void deattach(const GPIO::In::Pin& a_pin);
        void deattach(const GPIO::Out::Pin& a_pin);
        void deattach(const GPIO::Alternate_function::Pin& a_pin);

    private:
        std::uint32_t idx;
        IRQn_Type irqn;

        friend GPIO;
    };

    struct mco : private xmcu::non_constructible
    {
        enum class Divider : std::uint32_t
        {
            _1 = 0x0u,
            _2 = RCC_CFGR_MCOPRE_0,
            _4 = RCC_CFGR_MCOPRE_1,
            _8 = RCC_CFGR_MCOPRE_0 | RCC_CFGR_MCOPRE_1,
            _16 = RCC_CFGR_MCOPRE_2
        };

        template<typename Clock_t, std::uint32_t clock_id = std::numeric_limits<std::uint32_t>::max()>
        static void enable(Divider a_divider) = delete;
        static void disable();
    };

    struct lsco : private xmcu::non_constructible
    {
        template<typename Clock_t> static void enable() = delete;
        static void disable();
    };

    GPIO()
        : out(nullptr)
        , in(nullptr)
        , analog(nullptr)
        , alternate_function(nullptr)
        , idx(std::numeric_limits<decltype(this->idx)>::max())
        , p_registers(nullptr)
        , flags(0u)
    {
    }

    ~GPIO()
    {
        if (true == this->is_enabled())
        {
            this->disable();
        }
    }

    void enable()
    {
        bit::set(&(this->flags), 31u);
    }

    void disable()
    {
        bit::clear(&(this->flags), 31u);
    }

    bool is_pin_taken(std::uint8_t a_id) const
    {
        return bit::is(this->flags, a_id);
    }

    bool is_enabled() const
    {
        return bit::is(this->flags, 31u);
    }

    bool is_created()
    {
        return std::numeric_limits<decltype(this->idx)>::max() != this->idx && nullptr != this->p_registers;
    }

    explicit operator xmcu::soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::Registers*()
    {
        return this->p_registers;
    }

    Out out;
    In in;
    Analog analog;
    Alternate_function alternate_function;

private:
    GPIO(std::uint32_t a_idx, rm0434::peripherals::ll::gpio::Registers* a_p_registers)
        : out(this)
        , in(this)
        , analog(this)
        , alternate_function(this)
        , idx(a_idx)
        , p_registers(a_p_registers)
        , flags(0u)
    {
    }

    void take_pin(std::uint8_t a_id)
    {
        bit::set(&(this->flags), a_id);
    }

    void give_pin(std::uint8_t a_id)
    {
        bit::clear(&(this->flags), a_id);
    }

    std::uint32_t idx;
    xmcu::soc::st::arm::m4::wb::rm0434::peripherals::ll::gpio::Registers* p_registers;

    std::uint32_t flags;

    friend Out;
    friend In;
    friend Analog;
    friend Alternate_function;
    template<typename Periph_t, std::uint32_t id> friend class xmcu::soc::peripheral;
};

template<> void GPIO::mco::enable<clocks::sources::lse>(Divider a_divider);
template<> void GPIO::mco::enable<clocks::sources::lsi, 1>(Divider a_divider);
template<> void GPIO::mco::enable<clocks::sources::lsi, 2>(Divider a_divider);
template<> void GPIO::mco::enable<clocks::sources::hse>(Divider a_divider);
template<> void GPIO::mco::enable<clocks::sources::hsi16>(Divider a_divider);
template<> void GPIO::mco::enable<clocks::sources::pll::r>(Divider a_divider);
template<> void GPIO::mco::enable<clocks::sources::msi>(Divider a_divider);
template<> void GPIO::mco::enable<clocks::sources::hsi48>(Divider a_divider);
template<> void GPIO::mco::enable<rcc<system::mcu<1>>>(Divider a_divider);

template<> void GPIO::lsco::enable<clocks::sources::lsi>();
template<> void GPIO::lsco::enable<clocks::sources::lse>();

constexpr GPIO::Interrupt::Trigger_flag operator|(GPIO::Interrupt::Trigger_flag a_f1,
                                                  GPIO::Interrupt::Trigger_flag a_f2)
{
    return static_cast<GPIO::Interrupt::Trigger_flag>(static_cast<std::uint32_t>(a_f1) |
                                                      static_cast<std::uint32_t>(a_f2));
}
constexpr GPIO::Interrupt::Trigger_flag operator&(GPIO::Interrupt::Trigger_flag a_f1,
                                                  GPIO::Interrupt::Trigger_flag a_f2)
{
    return static_cast<GPIO::Interrupt::Trigger_flag>(static_cast<std::uint32_t>(a_f1) &
                                                      static_cast<std::uint32_t>(a_f2));
}
constexpr GPIO::Interrupt::Trigger_flag operator|=(GPIO::Interrupt::Trigger_flag& a_f1,
                                                   GPIO::Interrupt::Trigger_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}
} // namespace xmcu::soc::st::arm::m4::wb::rm0434::peripherals

namespace xmcu::soc::st::arm::m4::wb::rm0434 {
template<std::uint32_t id> class rcc<peripherals::GPIO, id> : private xmcu::non_constructible
{
public:
    static void enable(bool a_enable_in_lp) = delete;
    static void disable() = delete;
};

template<> void rcc<peripherals::GPIO, 1>::enable(bool a_enable_in_lp);
template<> void rcc<peripherals::GPIO, 1>::disable();

template<> void rcc<peripherals::GPIO, 2>::enable(bool a_enable_in_lp);
template<> void rcc<peripherals::GPIO, 2>::disable();

template<> void rcc<peripherals::GPIO, 3>::enable(bool a_enable_in_lp);
template<> void rcc<peripherals::GPIO, 3>::disable();

template<> void rcc<peripherals::GPIO, 4>::enable(bool a_enable_in_lp);
template<> void rcc<peripherals::GPIO, 4>::disable();

template<> void rcc<peripherals::GPIO, 5>::enable(bool a_enable_in_lp);
template<> void rcc<peripherals::GPIO, 5>::disable();

template<> void rcc<peripherals::GPIO, 8>::enable(bool a_enable_in_lp);
template<> void rcc<peripherals::GPIO, 8>::disable();

template<>
void peripherals::GPIO::Alternate_function::enable<peripherals::GPIO::mco>(Limited<std::uint32_t, 0, 15> a_id,
                                                                           const Enable_config& a_config,
                                                                           Pin* a_p_pin);
template<>
void peripherals::GPIO::Alternate_function::enable<peripherals::GPIO::lsco>(Limited<std::uint32_t, 0, 15> a_id,
                                                                            const Enable_config& a_config,
                                                                            Pin* a_p_pin);
} // namespace xmcu::soc::st::arm::m4::wb::rm0434

namespace xmcu::soc {
#if defined(XMCU_GPIOA_PRESENT)
template<> class peripheral<st::arm::m4::wb::rm0434::peripherals::GPIO, 1u> : private non_constructible
{
public:
    static st::arm::m4::wb::rm0434::peripherals::GPIO create()
    {
        namespace wb_peripherals = st::arm::m4::wb::rm0434::peripherals;
        return st::arm::m4::wb::rm0434::peripherals::GPIO(
            0u, wb_peripherals::ll::gpio::registers<wb_peripherals::ll::gpio::A>());
    }
};
#endif

#if defined(XMCU_GPIOB_PRESENT)
template<> class peripheral<st::arm::m4::wb::rm0434::peripherals::GPIO, 2u> : private non_constructible
{
public:
    static st::arm::m4::wb::rm0434::peripherals::GPIO create()
    {
        namespace wb_peripherals = st::arm::m4::wb::rm0434::peripherals;
        return st::arm::m4::wb::rm0434::peripherals::GPIO(
            1u, wb_peripherals::ll::gpio::registers<wb_peripherals::ll::gpio::B>());
    }
};
#endif

#if defined(XMCU_GPIOC_PRESENT)
template<> class peripheral<st::arm::m4::wb::rm0434::peripherals::GPIO, 3u> : private non_constructible
{
public:
    static st::arm::m4::wb::rm0434::peripherals::GPIO create()
    {
        namespace wb_peripherals = st::arm::m4::wb::rm0434::peripherals;
        return st::arm::m4::wb::rm0434::peripherals::GPIO(
            2u, wb_peripherals::ll::gpio::registers<wb_peripherals::ll::gpio::C>());
    }
};
#endif

#if defined(XMCU_GPIOE_PRESENT)
template<> class peripheral<st::arm::m4::wb::rm0434::peripherals::GPIO, 5u> : private non_constructible
{
public:
    static st::arm::m4::wb::rm0434::peripherals::GPIO create()
    {
        namespace wb_peripherals = st::arm::m4::wb::rm0434::peripherals;
        return st::arm::m4::wb::rm0434::peripherals::GPIO(
            4u, wb_peripherals::ll::gpio::registers<wb_peripherals::ll::gpio::E>());
    }
};
#endif

#if defined(XMCU_GPIOH_PRESENT)
template<> class peripheral<st::arm::m4::wb::rm0434::peripherals::GPIO, 8u> : private non_constructible
{
public:
    static st::arm::m4::wb::rm0434::peripherals::GPIO create()
    {
        namespace wb_peripherals = st::arm::m4::wb::rm0434::peripherals;
        return st::arm::m4::wb::rm0434::peripherals::GPIO(
            7u, wb_peripherals::ll::gpio::registers<wb_peripherals::ll::gpio::H>());
    }
};
#endif
} // namespace xmcu::soc
