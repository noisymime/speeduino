/** 
 * @addtogroup table_3d 
 *  @{
 */

/** \file
 * @brief 3D table axis types and iterators
 */

#pragma once

#include "table3d_typedefs.h"

/**\enum axis_domain
 * @brief Encodes the real world measurement that a table axis captures
 * */
enum axis_domain {
    /** RPM (engine speed) */
    axis_domain_Rpm,
    /** Load */
    axis_domain_Load,
    /** Throttle position */ 
    axis_domain_Tps 
};

/** @brief Iterate over table axis elements */
class table_axis_iterator
{
public:

    /** @brief Construct */
    table_axis_iterator(const table3d_axis_t *pStart, const table3d_axis_t *pEnd, int8_t stride, axis_domain domain)
     : _pAxis(pStart), _pAxisEnd(pEnd), _stride(stride), _domain(domain)
    {
    }

    axis_domain domain(void) const { return _domain; }

    /** @brief Advance the iterator
     * @param steps The number of elements to move the iterator
    */
    inline table_axis_iterator& advance(table3d_dim_t steps)
    {
        _pAxis = _pAxis + (_stride * steps);
        return *this;
    }

    /** @brief Increment the iterator by one element*/
    inline table_axis_iterator& operator++()
    {
        return advance(1);
    }

    /** @brief Test for end of iteration */
    inline bool at_end() const
    {
        return _pAxis == _pAxisEnd;
    }

    /** @brief Dereference the iterator */
    inline table3d_axis_t& operator*()
    {
        return *const_cast<table3d_axis_t *>(_pAxis);
    }
    /** @copydoc table_axis_iterator::operator*()  */
    inline const table3d_axis_t& operator*() const
    {
        return *_pAxis;
    }    
    
    /** @brief Reverse the iterator direction
     * 
     * Iterate from the end to the start. <b>This is only meant to be called on a freshly constructed iterator.</b>
     */
    inline table_axis_iterator& reverse()
    {
        const table3d_axis_t *_pOldAxis = _pAxis;
        _pAxis = _pAxisEnd - _stride;
        _pAxisEnd = _pOldAxis - _stride;
        _stride = (int8_t)(_stride * -1);
        return *this;
    }

private:
    const table3d_axis_t *_pAxis;
    const table3d_axis_t *_pAxisEnd;
    int8_t _stride;
    axis_domain _domain;
};

#define TABLE3D_TYPENAME_XAXIS(size, xDom, yDom) CONCAT(TABLE3D_TYPENAME_BASE(size, xDom, yDom), _xaxis)

#define TABLE3D_GEN_XAXIS(size, xDom, yDom) \
    /** @brief The x-axis for a 3D table with size x size dimensions, xDom x-axis and yDom y-axis */ \
    struct TABLE3D_TYPENAME_XAXIS(size, xDom, yDom) { \
        /** @brief The length of the axis in elements */ \
        static constexpr table3d_dim_t length = size; \
        /** @brief The domain the axis represents */ \
        static constexpr axis_domain domain = axis_domain_ ## xDom; \
        /**
          @brief The axis elements \
          @details The x-axis is conventional: axis[0] is the minimum \
        */ \
        table3d_axis_t axis[size]; \
        \
        /** @brief Iterate over the axis elements */ \
        inline table_axis_iterator begin() \
        {  \
            return table_axis_iterator(axis, axis+size, 1, domain); \
        } \
    };
TABLE3D_GENERATOR(TABLE3D_GEN_XAXIS)

#define TABLE3D_TYPENAME_YAXIS(size, xDom, yDom) CONCAT(TABLE3D_TYPENAME_BASE(size, xDom, yDom), _yaxis)

#define TABLE3D_GEN_YAXIS(size, xDom, yDom) \
    /** @brief The y-axis for a 3D table with size x size dimensions, xDom x-axis and yDom y-axis */ \
    struct CONCAT(TABLE3D_TYPENAME_BASE(size, xDom, yDom), _yaxis) { \
        /** @brief The length of the axis in elements */ \
        static constexpr table3d_dim_t length = size; \
        /** @brief The domain the axis represents */ \
        static constexpr axis_domain domain = axis_domain_ ## yDom; \
        /**
          @brief The axis elements \
          @details The y-axis is reversed: axis[n-1] is the minimum \
        */ \
        table3d_axis_t axis[size]; \
        \
        /** @brief Iterate over the axis elements */ \
        inline table_axis_iterator begin() \
        { \
            return table_axis_iterator(axis+(size-1), axis-1, -1, domain); \
        } \
    };
TABLE3D_GENERATOR(TABLE3D_GEN_YAXIS)

/** @} */