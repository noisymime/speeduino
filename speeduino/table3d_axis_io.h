/** 
 * @addtogroup table_3d 
 *  @{
 */

/** \file
 * @brief 3D table axis I/O support 
 */

#pragma once

#include "int16_byte.h"
#include "table3d_axes.h"
#include "src/libdivide/constant_fast_div.h"

/** @brief table axis I/O support 
 * 
 * @attention Using \c constexpr class static variables seems to be the best
 * combination of size and speed - \c constexpr implies \c inline, so we
 * can't use it on traditional \c extern global variables.
*/
class table3d_axis_io {
public:

    /**
     * @brief Obtain a converter instance for a given axis domain.
     * 
     * Often we need to deal internally with values that fit in 16-bits but do
     * not require much accuracy. E.g. RPM. For these values we can 
     * save storage space (EEPROM) by scaling to/from 8-bits using a fixed divisor.
     * 
     * The divisor is dependent on the domain. I.e all axes with the same domain use 
     * the same divisor 
     * 
     * Conversion during I/O is orthogonal to other axis concerns, so is separated and
     * encapsulated here.
     */
    static constexpr const int16_byte* get_converter(axis_domain domain)  {
        return domain==axis_domain_Rpm ? &converter_100 :
                domain==axis_domain_Load ? &converter_2 : &converter_1;
    }

    /** 
     * @brief Convert to a \c byte 
     * 
     * Useful for converting a single value.
     * If converting multiple, probably faster to cache the converter rather than
     * repeatedly calling this function.
    */
    static inline byte to_byte(axis_domain domain, int16_t value) { return get_converter(domain)->to_byte(value); }

    /** 
     * @brief Convert from a \c byte 
     * 
     * Useful for converting a single value.
     * If converting multiple, probably faster to cache the converter rather than
     * repeatedly calling this function.
    */
    static inline int16_t from_byte(axis_domain domain, byte in ) { return get_converter(domain)->from_byte(in); }    

private:
    static constexpr int16_byte converter_100 = { 100, { S16_MAGIC(100), S16_MORE(100) } };
    static constexpr int16_byte converter_2 = { 2, { S16_MAGIC(2), S16_MORE(2) } };
    static constexpr int16_byte converter_1 = { 1, { S16_MAGIC(1), S16_MORE(1) } };
};

/** @} */