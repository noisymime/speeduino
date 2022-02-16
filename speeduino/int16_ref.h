/** 
 * @addtogroup table_3d 
 * @{
 */

#pragma once

#include <stdint.h>
#include "src/libdivide/libdivide.h"

/** @brief Byte type. This is not defined in any C or C++ standard header. */
typedef uint8_t byte;

/** @brief Represents a 16-bit value as a byte. Useful for I/O. 
 * 
 * Often we need to deal internally with values that fit in 16-bits but do
 * not require much accuracy. E.g. table axes in RPM. For these values we can 
 * save storage space (EEPROM) by scaling to/from 8-bits using a fixed divisor.
 */
class int16_ref
{
public:

#pragma pack(push, 1)
    struct scalar {
        uint8_t _factor;
        libdivide::libdivide_s16_t divider;
    };
#pragma pack(pop)

    /**
     * @brief Construct
     * @param value The \c int16_t to encapsulate.
     * @param pScalar The factor to scale the \c int16_t value by when converting to/from a \c byte
     */
    int16_ref(int16_t &value, const scalar *pScalar) 
        : _value(value), _pScalar(pScalar)
    {
    }

    /** @brief Convert to a \c byte */
    inline byte operator*() const { return (byte)libdivide::libdivide_s16_do(_value, &_pScalar->divider); }

    /** @brief Convert to a \c byte */
    inline explicit operator byte() const { return **this; }

    /** @brief Convert from a \c byte */
    inline int16_ref &operator=( byte in )  { _value = (int16_t)in * (int16_t)(_pScalar->_factor); return *this;  }

private:
    int16_t &_value;
    const scalar *_pScalar;
};

/** @} */
