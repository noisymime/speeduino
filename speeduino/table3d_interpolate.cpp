#include "table3d_interpolate.h"


// ============================= Axis Bin Searching =========================

static inline bool is_in_bin(const table3d_axis_t &testValue, const table3d_axis_t &min, const table3d_axis_t &max)
{
  return testValue > min && testValue <= max;
}

// Find the axis index for the top of the bin that covers the test value.
// E.g. 4 in { 1, 3, 5, 7, 9 } would be 2
// We assume the axis is in order.
static inline table3d_dim_t find_bin_max(
  table3d_axis_t &value,        // Value to search for
  const table3d_axis_t *pAxis,  // The axis to search
  table3d_dim_t minElement,     // Axis index of the element with the lowest value
  table3d_dim_t maxElement,     // Axis index of the element with the highest value
  table3d_dim_t lastBin)        // The last result from this call - used to speed up searches
{
  // Direction to search (1 coventional, -1 to go backwards from pAxis)
  int8_t stride = maxElement>minElement ? 1 : -1;
  // It's quicker to increment/adjust these pointers than to repeatedly 
  // index the array - minimum 2%, often >5%
  const table3d_axis_t *pMax = nullptr;

  // Check the cached last bin and either side first - it's likely that this will give a hit under
  // real world conditions
  // Check if we're still in the same bin as last time
  pMax = pAxis + lastBin;
  if (is_in_bin(value, *(pMax - stride), *pMax))
  {
    return lastBin;
  }
  // Check the bin above the last one
  pMax = pMax - stride;
  if (lastBin!=(minElement+stride) && is_in_bin(value, *(pMax - stride), *pMax))
  {
    return lastBin-stride;    
  }
  // Check the bin below the last one
  pMax += stride*2;
  if (lastBin!=maxElement && is_in_bin(value, *(pMax - stride), *pMax))
  {
    return lastBin+stride;
  }

  // At or above maximum - clamp to final value
  if (value>=pAxis[maxElement])
  {
    value = pAxis[maxElement];
    return maxElement;
  }
  // At or below minimum - clamp to lowest value
  if (value<=pAxis[minElement])
  {
    value = pAxis[minElement];
    return minElement+stride;
  }

  // No hits above, so run a linear search.
  // We start at the maximum & work down, rather than looping from [0] up to [max]
  // This is because the important tables (fuel and injection) will have the highest
  // RPM at the top of the X axis, so starting there will mean the best case occurs 
  // when the RPM is highest (and hence the CPU is needed most)
  lastBin = maxElement;
  table3d_dim_t stop = minElement+stride;
  pMax = pAxis + lastBin;
  while (lastBin!=stop && !is_in_bin(value, *(pMax - stride), *pMax))
  {
    lastBin -= stride;
    pMax -= stride;
  }
  return lastBin;
}

table3d_dim_t find_xbin(table3d_axis_t &value, const table3d_axis_t *pAxis, table3d_dim_t size, table3d_dim_t lastBin)
{
  return find_bin_max(value, pAxis, 0, size-1, lastBin);
}

table3d_dim_t find_ybin(table3d_axis_t &value, const table3d_axis_t *pAxis, table3d_dim_t size, table3d_dim_t lastBin)
{
  // Y axis is stored in reverse for performance purposes (not sure that's still valid). 
  // The minimum value is at the end & max at the start. So need to adjust for that. 
  return find_bin_max(value, pAxis, size-1, 0, lastBin);
}

// ========================= Fixed point math =========================

typedef uint16_t Q1X8_t;
static constexpr uint8_t Q1X8_INTEGER_SHIFT = 8;
static constexpr Q1X8_t Q1X8_ONE = 1U << Q1X8_INTEGER_SHIFT;
static constexpr Q1X8_t Q1X8_HALF = 1U << (Q1X8_INTEGER_SHIFT-1);

inline Q1X8_t mulQ1X8(Q1X8_t a, Q1X8_t b)
{
    // 1x1 == 1....but the real reason for this is to avoid 16-bit multiplication overflow.
    //
    // We are using uint16_t as our underlying fixed point type. If we follow the regular
    // code path, we'd need to promote to uint32_t to avoid overflow.
    //
    // The overflow can only happen when *both* the X & Y inputs
    // are at the edge of a bin. 
    //
    // This is a rare condition, so most of the time we can use 16-bit mutiplication and gain performance
    if (a==Q1X8_ONE && b==Q1X8_ONE)
    {
        return Q1X8_ONE;
    }
  // Add the equivalent of 0.5 to the final calculation pre-rounding.
  // This will have the effect of rounding to the nearest integer, rather
  // than always rounding down.
  return ((a * b) + Q1X8_HALF) >> Q1X8_INTEGER_SHIFT;
}

// ============================= Axis value to bin % =========================

static inline Q1X8_t compute_bin_position(table3d_axis_t value, const table3d_dim_t &bin, int8_t stride, const table3d_axis_t *pAxis)
{
  table3d_axis_t binMinValue = pAxis[bin-stride];
  if (value==binMinValue) { return 0; }
  table3d_axis_t binMaxValue = pAxis[bin];
  if (value==binMaxValue) { return Q1X8_ONE; }
  table3d_axis_t binWidth = binMaxValue-binMinValue;

  // Since we can have bins of any width, we need to use 
  // 24.8 fixed point to avoid overflow
  uint32_t p = (uint32_t)(value - binMinValue) << Q1X8_INTEGER_SHIFT;
  // But since we are computing the ratio (0 to 1), p is guarenteed to be
  // less than binWidth and thus the division below will result in a value
  // <=1. So we can reduce the data type from 24.8 (uint32_t) to 1.8 (uint16_t)
  return p / binWidth;  
}


// ============================= End internal support functions =========================

//This function pulls a value from a 3D table given a target for X and Y coordinates.
//It performs a 2D linear interpolation as described in: www.megamanual.com/v22manual/ve_tuner.pdf
table3d_value_t get3DTableValue(struct table3DGetValueCache *pValueCache, 
                    table3d_dim_t axisSize,
                    const table3d_value_t *pValues,
                    const table3d_axis_t *pXAxis,
                    const table3d_axis_t *pYAxis,
                    table3d_axis_t Y_in, table3d_axis_t X_in)
{
    //0th check is whether the same X and Y values are being sent as last time. 
    // If they are, this not only prevents a lookup of the axis, but prevents the 
    //interpolation calcs being performed
    if( X_in == pValueCache->last_lookup.x && 
        Y_in == pValueCache->last_lookup.y)
    {
      return pValueCache->lastOutput;
    }

    // Assign this here, as we might modify coords below.
    pValueCache->last_lookup.x = X_in;
    pValueCache->last_lookup.y = Y_in;

    // Figure out where on the axes the incoming coord are
    pValueCache->lastXBinMax = find_xbin(X_in, pXAxis, axisSize, pValueCache->lastXBinMax);
    pValueCache->lastYBinMax = find_ybin(Y_in, pYAxis, axisSize, pValueCache->lastYBinMax);

    /*
    At this point we have the 4 corners of the map where the interpolated value will fall in
    Eg: (yMax,xMin)  (yMax,xMax)

        (yMin,xMin)  (yMin,xMax)

    In the following calculation the table values are referred to by the following variables:
              A          B

              C          D
    */
    table3d_dim_t rowMax = pValueCache->lastYBinMax * axisSize;
    table3d_dim_t rowMin = (pValueCache->lastYBinMax+1) * axisSize;
    table3d_value_t A = pValues[rowMax + pValueCache->lastXBinMax-1];
    table3d_value_t B = pValues[rowMax + pValueCache->lastXBinMax];
    table3d_value_t C = pValues[rowMin + pValueCache->lastXBinMax-1];
    table3d_value_t D = pValues[rowMin + pValueCache->lastXBinMax];

    //Check that all values aren't just the same (This regularly happens with things like the fuel trim maps)
    if( (A == B) && (A == C) && (A == D) ) { pValueCache->lastOutput = A; }
    else
    {
      //Create some normalised position values
      //These are essentially percentages (between 0 and 1) of where the desired value falls between the nearest bins on each axis
      const Q1X8_t p = compute_bin_position(X_in, pValueCache->lastXBinMax, 1, pXAxis);
      const Q1X8_t q = compute_bin_position(Y_in, pValueCache->lastYBinMax, -1, pYAxis);

      Q1X8_t m = mulQ1X8(Q1X8_ONE-p, q);
      Q1X8_t n = mulQ1X8(p, q);
      Q1X8_t o = mulQ1X8(Q1X8_ONE-p, Q1X8_ONE-q);
      Q1X8_t r = mulQ1X8(p, Q1X8_ONE-q);
      pValueCache->lastOutput = ( (A * m) + (B * n) + (C * o) + (D * r) ) >> Q1X8_INTEGER_SHIFT;
    }

    return pValueCache->lastOutput;
}