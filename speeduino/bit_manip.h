#pragma once

/**
 * @file
 * @brief Bit twiddling macros
 */

/** @brief Set bit b (0-7) in byte a */
#define BIT_SET(var,pos) ((var) |= (1U<<(pos)))

/** @brief Clear bit b (0-7) in byte a */
#define BIT_CLEAR(var,pos) ((var) &= ~(1U<<(pos)))

/** @brief Is bit pos (0-7) in byte var set? */
#define BIT_CHECK(var,pos) !!((var) & (1U<<(pos)))

/** @brief Toggle the value of bit pos (0-7) in byte var */
#define BIT_TOGGLE(var,pos) ((var)^= 1UL << (pos))

/** @brief Set the value ([0,1], [true, false]) of bit pos (0-7) in byte var */
#define BIT_WRITE(var, pos, bitvalue) ((bitvalue) ? BIT_SET((var), (pos)) : BIT_CLEAR((var), (pos)))