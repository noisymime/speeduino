/** 
 * @defgroup table_3d 3D Tables
 * @brief Structures and functions related to 3D tables, such as VE, Spark Advance, AFR etc.
 * 
 * Logical: 
 *      - each 3D table is a continuous height map spread over a cartesian (x, y) plane
 *          - Continuous: we expect to interpolate between any 4 points
 *      - The axes are
 *          - Bounded. I.e. non-infinite 
 *          - Non-linear. I.e. x[n]-x[n-1] != x[n+1]-x[n]
 *          - Increasing. I.e. x[n] >= x[n-1]
 *          - Do not have to start at [0,0]
 * 
 * E.g. for a 3x3 table, this is what the TS table editor would show:
 * <pre>
 *      Y-Max    V6      V7      V8
 *      Y-Int    V3      V4      V5
 *      Y-Min    V0      V1      V2
 *              X-Min   X-Int   X-Max
 * </pre>
 *
 * In memory, we store rows in reverse:
 *      - The X axis is conventional: <c>x[0]</c> stores \c X-Min
 *      - The Y-axis is inverted: <c>y[0]</c> stores \c Y-Max
 *      - The value locations match the axes.
 *          - <c>value[0][0]</c> stores \c V6.
 *          - <c>value[2][0]</c> stores \c V0.
 * 
 * I.e.
 * <pre>
 *      Y-Min    V0      V1      V2
 *      Y-Int    V3      V4      V5
 *      Y-Max    V6      V7      V8
 *              X-Min   X-Int   X-Max
 * </pre>
 *  @{
 */

/** \file
 * @brief 3D table data types and functions
 */

#pragma once

#include "table3d_interpolate.h"
#include "table3d_axes.h"
#include "table3d_values.h"

#define TO_TYPE_KEY(size, xDom, yDom) table3d ## size ## xDom ## yDom ## _key

/**
 * @brief Table \b type identifiers. Limited compile time RTTI
 * 
 * With no virtual functions (they have quite a bit of overhead in both space & 
 * time), we have to pass void* around in certain cases. In order to cast that 
 * back to a concrete table type, we need to somehow identify the type.
 * 
 * Once approach is to register each type - but that requires a central registry
 * which will use RAM.
 * 
 * Since we have a compile time fixed set of table types, we can map a unique
 * identifier to the type via a cast - this enum is that unique identifier.
 * 
 * Typically used in conjunction with the '#CONCRETE_TABLE_ACTION' macro
 */
enum table_type_t {
    table_type_None,
    #define TABLE3D_GEN_TYPEKEY(size, xDom, yDom) TO_TYPE_KEY(size, xDom, yDom),
    TABLE3D_GENERATOR(TABLE3D_GEN_TYPEKEY)
};

// Generate the 3D table types
#define TABLE3D_GEN_TYPE(size, xDom, yDom) \
    /** @brief A 3D table with size x size dimensions, xDom x-axis and yDom y-axis */ \
    struct TABLE3D_TYPENAME_BASE(size, xDom, yDom) \
    { \
        typedef TABLE3D_TYPENAME_XAXIS(size, xDom, yDom) xaxis_t; \
        typedef TABLE3D_TYPENAME_YAXIS(size, xDom, yDom) yaxis_t; \
        typedef TABLE3D_TYPENAME_VALUE(size, xDom, yDom) value_t; \
        /* This will take up zero space unless we take the address somewhere */ \
        static constexpr table_type_t type_key = TO_TYPE_KEY(size, xDom, yDom); \
        \
        table3DGetValueCache get_value_cache; \
        TABLE3D_TYPENAME_VALUE(size, xDom, yDom) values; \
        TABLE3D_TYPENAME_XAXIS(size, xDom, yDom) axisX; \
        TABLE3D_TYPENAME_YAXIS(size, xDom, yDom) axisY; \
    };
TABLE3D_GENERATOR(TABLE3D_GEN_TYPE)

// Generate get3DTableValue() functions
#define TABLE3D_GEN_GET_TABLE_VALUE(size, xDom, yDom) \
    inline int get3DTableValue(TABLE3D_TYPENAME_BASE(size, xDom, yDom) *pTable, table3d_axis_t y, table3d_axis_t x) \
    { \
      return get3DTableValue( &pTable->get_value_cache, \
                              TABLE3D_TYPENAME_BASE(size, xDom, yDom)::value_t::row_size, \
                              pTable->values.values, \
                              pTable->axisX.axis, \
                              pTable->axisY.axis, \
                              y, x); \
    } 
TABLE3D_GENERATOR(TABLE3D_GEN_GET_TABLE_VALUE)

// =============================== Table function calls =========================

// With no templates or inheritance we need some way to call functions
// for the various distinct table types. CONCRETE_TABLE_ACTION dispatches
// to a caller defined function overloaded by the type of the table. 
#define CONCRETE_TABLE_ACTION_INNER(size, xDomain, yDomain, action, ...) \
  case TO_TYPE_KEY(size, xDomain, yDomain): action(size, xDomain, yDomain, ##__VA_ARGS__);
#define CONCRETE_TABLE_ACTION(testKey, action, ...) \
  switch ((table_type_t)testKey) { \
  TABLE3D_GENERATOR(CONCRETE_TABLE_ACTION_INNER, action, ##__VA_ARGS__ ) \
  default: abort(); }

// =============================== Table function calls =========================

table_value_iterator rows_begin(const void *pTable, table_type_t key);

table_axis_iterator x_begin(const void *pTable, table_type_t key);

table_axis_iterator y_begin(const void *pTable, table_type_t key);

/** @} */
