#include <stdlib.h>
#include "table3d.h"

// =============================== Iterators =========================

table_value_iterator rows_begin(const void *pTable, table_type_t key)
{
  #define GET_ROW_ITERATOR(size, xDomain, yDomain, pTable) \
      return ((DECLARE_3DTABLE_TYPENAME(size, xDomain, yDomain)*)pTable)->values.begin();
  CONCRETE_TABLE_ACTION(key, GET_ROW_ITERATOR, pTable);
}


/**
 * Convert page iterator to table x axis iterator.
 */
table_axis_iterator x_begin(const void *pTable, table_type_t key)
{
  #define GET_X_ITERATOR(size, xDomain, yDomain, pTable) \
      return ((DECLARE_3DTABLE_TYPENAME(size, xDomain, yDomain)*)pTable)->axisX.begin();
  CONCRETE_TABLE_ACTION(key, GET_X_ITERATOR, pTable);
}


/**
 * Convert page iterator to table y axis iterator.
 */
table_axis_iterator y_begin(const void *pTable, table_type_t key)
{
  #define GET_Y_ITERATOR(size, xDomain, yDomain, pTable) \
      return ((DECLARE_3DTABLE_TYPENAME(size, xDomain, yDomain)*)pTable)->axisY.begin();
  CONCRETE_TABLE_ACTION(key, GET_Y_ITERATOR, pTable);
}