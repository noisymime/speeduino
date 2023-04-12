#pragma once

#include "table3d_typedefs.h"

// A table location. 
struct coord2d
{
    table3d_axis_t x;
    table3d_axis_t y;
};


struct table3DGetValueCache {
  // Store the upper *index* of the X and Y axis bins that were last hit.
  // This is used to make the next check faster since very likely the x & y values have
  // only changed by a small amount & are in the same bin (or an adjacent bin).
  //
  // It's implicit that the other bin index is max bin index - 1 (a single axis
  // value can't span 2 axis bins). This saves 1 byte.
  //
  // E.g. 6 element x-axis contents:
  //   [ 8| 9|12|15|18|21]
  // indices:
  //     0, 1, 2, 3, 4, 5
  // If lastXBinMax==3, the min index must be 2. I.e. the last X value looked
  // up was between 12<X<=15.
  table3d_dim_t lastXBinMax = 1;
  table3d_dim_t lastYBinMax = 1;

  //Store the last input and output values, again for caching purposes
  coord2d last_lookup = { INT16_MAX, INT16_MAX };
  table3d_value_t lastOutput;
};


static inline void invalidate_cache(table3DGetValueCache *pCache)
{
    pCache->last_lookup.x = INT16_MAX;
}

/*
3D Tables have an origin (0,0) in the top left hand corner. Vertical axis is expressed first.
Eg: 2x2 table
-----
|2 7|
|1 4|
-----

(0,1) = 7
(0,0) = 2
(1,0) = 1

*/
table3d_value_t get3DTableValue(struct table3DGetValueCache *pValueCache, 
                    table3d_dim_t axisSize,
                    const table3d_value_t *pValues,
                    const table3d_axis_t *pXAxis,
                    const table3d_axis_t *pYAxis,
                    table3d_axis_t y, table3d_axis_t x);