#include "test_utils.h"
#include <avr/pgmspace.h>

void __attribute__((noinline)) populateTable(table3d16RpmLoad &table, const table3d_value_t values[], const table3d_axis_t xAxis[], const table3d_axis_t yAxis[]) {
  //
  // NOTE: USE OF ITERATORS HERE IS DELIBERATE. IT INCLUDES THEM IN THE UNIT TESTS, giving
  // them some coverage
  //
  {
    table_axis_iterator itX = table.axisX.begin();
    while (!itX.at_end())
    {
      *itX = *xAxis;
      ++xAxis;
      ++itX;
    }
  }
  {
    table_axis_iterator itY = table.axisY.begin();
    while (!itY.at_end())
    {
      *itY = *yAxis;
      ++yAxis;
      ++itY;
    }
  }

  {
    table_value_iterator itZ = table.values.begin();
    while (!itZ.at_end())
    {
      table_row_iterator itRow = *itZ;
      while (!itRow.at_end())
      {
#if defined(PROGMEM)
        *itRow = pgm_read_byte(values);
#else
        *itRow = *values;
#endif
        ++values;
        ++itRow;
      }
      ++itZ;
    }
  }
}