/** 
 * @addtogroup table_3d 
 * @{
 */

#pragma once


#include <stdint.h>

#ifdef USE_LIBDIVIDE
#include "src/libdivide/libdivide.h"
#endif

/** @brief Byte type. This is not defined in any C or C++ standard header. */
typedef uint8_t byte;

/** @brief Represents a 16-bit value as a byte. Useful for I/O. 
 *
 * Often we need to deal internally with values that fit in 16-bits but do
 * not require much accuracy. E.g. table axes in RPM. For these values we can 
 * save storage space (EEPROM) by scaling to/from 8-bits using a fixed divisor.
 */
class int16_byte
{
public:

    /**
     * @brief Construct
     *
     * @param factor The factor to multiply when converting \c byte to \c int16_t
     * @param divider The factor to divide by when converting \c int16_t to \c byte
     * 
     * \c divider could be computed from \c factor, but including it as a parameter
     * allows callers to create \c factor instances at compile time.
     */
    constexpr int16_byte(uint8_t factor
#ifdef USE_LIBDIVIDE
    , const libdivide::libdivide_s16_t &divider
#endif
        )
        : _factor(factor)
#ifdef USE_LIBDIVIDE
        , _divider(divider)
#endif
    {
    }

    /** @brief Convert to a \c byte */
#ifdef USE_LIBDIVIDE
    inline byte to_byte(int16_t value) const { return _factor==1 ? value : _factor==2 ? value>>1 : (byte)libdivide::libdivide_s16_do(value, &_divider); }
#else
    inline byte to_byte(int16_t value) const { return (byte)(value/_factor); }
#endif
    /** @brief Convert from a \c byte */
    inline int16_t from_byte( byte in ) const { return (int16_t)in * _factor;  }

private:
    uint8_t _factor;
#ifdef USE_LIBDIVIDE
    libdivide::libdivide_s16_t _divider;
#endif
};

/** @} */
