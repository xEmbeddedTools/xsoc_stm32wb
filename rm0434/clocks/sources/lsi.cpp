/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// this
#include <rm0434/clocks/sources/lsi.hpp>

// xmcu
#include <xmcu/bit.hpp>
#include <rm0434/utils/tick_counter.hpp>
#include <rm0434/utils/wait_until.hpp>

namespace xmcu::soc::st::arm::m4::wb::rm0434::clocks::sources {
using namespace xmcu;
using namespace xmcu::soc::st::arm::m4::wb::rm0434::utils;

void lsi::enable(Id a_id)
{
    bit::set(&(RCC->CSR), RCC_CSR_LSI1ON_Pos + static_cast<std::uint32_t>(a_id));
    wait_until::all_bits_are_set(RCC->CSR, 0x1u << (RCC_CSR_LSI1RDY_Pos + static_cast<std::uint32_t>(a_id)));
}

bool lsi::enable(Id a_id, Milliseconds a_timeout)
{
    const std::uint64_t start = tick_counter<Milliseconds>::get();

    bit::set(&(RCC->CSR), RCC_CSR_LSI1ON_Pos + static_cast<std::uint32_t>(a_id));
    return wait_until::all_bits_are_set(RCC->CSR,
                                        0x1u << (RCC_CSR_LSI1RDY_Pos + static_cast<std::uint32_t>(a_id)),
                                        a_timeout.get() - (tick_counter<Milliseconds>::get() - start));
}

void lsi::disable(Id a_id)
{
    bit::clear(&(RCC->CSR), RCC_CSR_LSI1ON_Pos + static_cast<std::uint32_t>(a_id));
    wait_until::all_bits_are_cleared(RCC->CSR, 0x1u << (RCC_CSR_LSI1RDY_Pos + static_cast<std::uint32_t>(a_id)));
}

bool lsi::disable(Id a_id, Milliseconds a_timeout)
{
    const std::uint64_t start = tick_counter<Milliseconds>::get();

    bit::clear(&(RCC->CSR), RCC_CSR_LSI1ON_Pos + static_cast<std::uint32_t>(a_id));
    return wait_until::all_bits_are_cleared(RCC->CSR,
                                            0x1u << (RCC_CSR_LSI1RDY_Pos + static_cast<std::uint32_t>(a_id)),
                                            a_timeout.get() - (tick_counter<Milliseconds>::get() - start));
}

bool lsi::is_selected(Id a_id)
{
    switch (a_id)
    {
        case Id::_1: {
            if (true == is_enabled(Id::_2))
            {
                return false;
            }

            if (true == is_enabled(Id::_1))
            {
                return true;
            }

            return false;
        }
        break;

        case Id::_2: {
            if (true == is_enabled(Id::_2))
            {
                return true;
            }
            return false;
        }
        break;
    }

    return false;
}
} // namespace xmcu::soc::st::arm::m4::wb::rm0434::clocks::sources