#include <stdlib.h>
#include "table3d.h"

// =============================== Iterators =========================

table_value_iterator rows_begin(const void *pTable, table_type_t key)
{
  #define CTA_GET_ROW_ITERATOR(size, xDomain, yDomain, pTable) \
      return ((TABLE3D_TYPENAME_BASE(size, xDomain, yDomain)*)pTable)->values.begin();
  CONCRETE_TABLE_ACTION(key, CTA_GET_ROW_ITERATOR, pTable);
}


/**
 * Convert page iterator to table x axis iterator.
 */
table_axis_iterator x_begin(const void *pTable, table_type_t key)
{
  #define CTA_GET_X_ITERATOR(size, xDomain, yDomain, pTable) \
      return ((TABLE3D_TYPENAME_BASE(size, xDomain, yDomain)*)pTable)->axisX.begin();
  CONCRETE_TABLE_ACTION(key, CTA_GET_X_ITERATOR, pTable);
}

table_axis_iterator x_rbegin(const void *pTable, table_type_t key)
{
  #define CTA_GET_X_RITERATOR(size, xDomain, yDomain, pTable) \
      return ((TABLE3D_TYPENAME_BASE(size, xDomain, yDomain)*)pTable)->axisX.rbegin();
  CONCRETE_TABLE_ACTION(key, CTA_GET_X_RITERATOR, pTable);
}

/**
 * Convert page iterator to table y axis iterator.
 */
table_axis_iterator y_begin(const void *pTable, table_type_t key)
{
  #define CTA_GET_Y_ITERATOR(size, xDomain, yDomain, pTable) \
      return ((TABLE3D_TYPENAME_BASE(size, xDomain, yDomain)*)pTable)->axisY.begin();
  CONCRETE_TABLE_ACTION(key, CTA_GET_Y_ITERATOR, pTable);
}

table_axis_iterator y_rbegin(const void *pTable, table_type_t key)
{
  #define CTA_GET_Y_RITERATOR(size, xDomain, yDomain, pTable) \
      return ((TABLE3D_TYPENAME_BASE(size, xDomain, yDomain)*)pTable)->axisY.rbegin();
  CONCRETE_TABLE_ACTION(key, CTA_GET_Y_RITERATOR, pTable);
}