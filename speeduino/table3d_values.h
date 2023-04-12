/** 
 * @addtogroup table_3d 
 *  @{
 */

/** \file
 * @brief 3D table value structs and iterators
 */

#pragma once

#include "table3d_typedefs.h"

// ========================= INTRA-ROW ITERATION ========================= 

/**  @brief Iterate through a table row. I.e. constant Y, changing X 
 * 
 * Instances of this class are normally created via a table_value_iterator instance.
*/
class table_row_iterator {
public:

    /** 
     * @brief Construct
     * @param pRowStart Pointer to the 1st element in the row
     * @param rowWidth The number of elements to in the row
    */
    table_row_iterator(const table3d_value_t *pRowStart, table3d_dim_t rowWidth)
        : pValue(pRowStart), pEnd(pRowStart+rowWidth)  //cppcheck-suppress misra-c2012-10.4
    {
    }

    /** @brief Pointer to the end of the row */
    const table3d_value_t* end(void) const { return pEnd; }
    /** @copydoc table_row_iterator::end() const */
    table3d_value_t* end(void) { return const_cast<table3d_value_t *>(pEnd); }

    /** @brief Advance the iterator
     * @param steps The number of elements to move the iterator
    */
    table_row_iterator& advance(table3d_dim_t steps)
    { 
        pValue  = pValue + steps;
        return *this;
    }

    /** @brief Increment the iterator by one element*/
    table_row_iterator& operator++(void)
    {
        return advance(1);
    }

    /** @brief Test for end of iteration */
    bool at_end(void) const
    {
        return pValue == pEnd;
    }

    /** @brief Dereference the iterator */
    const table3d_value_t& operator*(void) const
    {
        return *pValue;
    }
    /** @copydoc table_row_iterator::operator*() const */
    table3d_value_t& operator*(void)
    {
        return *const_cast<table3d_value_t *>(pValue);
    }

    /** @brief Number of elements available */
    table3d_dim_t size(void) const { return pEnd-pValue; }

private:
    const table3d_value_t *pValue;
    const table3d_value_t *pEnd;
};

// ========================= INTER-ROW ITERATION ========================= 

/**  @brief Iterate through a tables values, row by row. */
class table_value_iterator
{
public:

    /** 
     * @brief Construct
     * @param pValues Pointer to the 1st value in a 1-d array
     * @param axisSize The number of columns & elements per row (square tables only)
    */
    table_value_iterator(const table3d_value_t *pValues, table3d_dim_t axisSize)
        : pRowsStart(pValues + (axisSize*(axisSize-1U))),  //cppcheck-suppress misra-c2012-10.4
        pRowsEnd(pValues - axisSize),
        rowWidth(axisSize)
    {
        // Table values are not linear in memory - rows are in reverse order
        // E.g. a 4x4 table with logical element [0][0] at the bottom left
        // (normal cartesian coordinates) has this layout.
        //  0	1	2	3
        //  4   5   6   7
        //  8   9   10  11
        //  12  13  14  15
        // So we start at row 3 (index 12 of the array) and iterate towards
        // the start of the array
        //
        // This all supports fast 3d interpolation.
    }

    /** @brief Advance the iterator
     * @param rows The number of \b rows to move
    */
    table_value_iterator& advance(table3d_dim_t rows)
    {
        pRowsStart = pRowsStart - (rowWidth * rows);
        return *this;
    }

    /** @brief Increment the iterator by one \b row */
    table_value_iterator& operator++(void)
    {
        return advance(1U);
    }

    /** @brief Dereference the iterator to access a row of data */
    const table_row_iterator operator*(void) const
    {
        return table_row_iterator(pRowsStart, rowWidth);
    }
    /** @copydoc table_value_iterator::operator*() const */
    table_row_iterator operator*(void)
    {
        return table_row_iterator(pRowsStart, rowWidth);
    }    

    /** @brief Test for end of iteration */
    bool at_end(void) const
    {
        return pRowsStart == pRowsEnd;
    }

private:
    const table3d_value_t *pRowsStart;
    const table3d_value_t *pRowsEnd;
    table3d_dim_t rowWidth;
};

#define TABLE3D_TYPENAME_VALUE(size, xDom, yDom) CONCAT(TABLE3D_TYPENAME_BASE(size, xDom, yDom), _values)

#define TABLE3D_GEN_VALUES(size, xDom, yDom) \
    /** @brief The values for a 3D table with size x size dimensions, xDom x-axis and yDom y-axis */ \
    struct TABLE3D_TYPENAME_VALUE(size, xDom, yDom) { \
        /** @brief The number of items in a row. I.e. it's length  */ \
        static constexpr table3d_dim_t row_size = (size); \
        /** @brief The number of rows */ \
        static constexpr table3d_dim_t num_rows = (size); \
        /** \
         @brief The row values \
         @details Table values are not linear in memory - rows are in reverse order<br> \
         E.g. a 3x3 table with logical element [0][0] at the bottom left \
         (normal cartesian coordinates) has this layout:<br> \
         6, 7, 8, 3, 4, 5, 0, 1, 2 \
        */ \
        table3d_value_t values[(uint16_t)row_size*num_rows]; \
        \
        /** @brief Iterate over the values */ \
        table_value_iterator begin(void) \
        {  \
            return table_value_iterator(values, row_size); \
        } \
        \
        /** \
         @brief Direct access to table value element from a linear index \
         @details Since table values aren't laid out linearly, converting a linear \
         offset to the equivalent memory address requires a modulus operation.<br> \
         <br> \
         This is slow, since AVR hardware has no divider. We can gain performance \
         in 2 ways:<br> \
          1. Forcing uint8_t calculations. These are much faster than 16-bit calculations<br> \
          2. Compiling this per table *type*. This encodes the axis length as a constant \
          thus allowing the optimising compiler more opportunity. E.g. for axis lengths \
          that are a power of 2, the modulus can be optimised to add/multiply/shift - much \
          cheaper than calling a software division routine such as __udivmodqi4<br> \
         <br> \
         THIS IS WORTH 20% to 30% speed up<br> \
         <br> \
         This limits us to 16x16 tables. If we need bigger and move to 16-bit \
         operations, consider using libdivide. <br> \
         */ \
        table3d_value_t& value_at(table3d_dim_t linear_index) \
        { \
            static_assert(row_size<17U, "Table is too big"); \
            static_assert(num_rows<17U, "Table is too big"); \
            /* Zero length will mess up unsigned calcs */ \
            static_assert(row_size>0U, "No zero length rows"); \
            static_assert(num_rows>0U, "No empty tables"); \
            constexpr table3d_dim_t first_index = row_size*(table3d_dim_t)(num_rows-1U); \
            const table3d_dim_t index = (table3d_dim_t)(first_index + (table3d_dim_t)(2U*(linear_index % row_size)) - linear_index); \
            return values[index]; \
        } \
    };
TABLE3D_GENERATOR(TABLE3D_GEN_VALUES)

/** @} */
