#pragma once

#include "table3d_typedefs.h"

// ========================= AXIS ITERATION =========================  

typedef uint8_t byte;

// Represents a 16-bit value as a byte
//
// Modelled after the Arduino EERef class
class int16_ref
{
public:

    int16_ref(int16_t &value, uint8_t factor) 
        : _value(value), _factor(factor)
    {
    }

    // Getters
    inline byte operator*() const { return (byte)(_value /_factor); }
    inline explicit operator byte() const { return **this; }
    // Setter
    inline int16_ref &operator=( byte in )  { _value = (int16_t)in * (int16_t)_factor; return *this;  }

private:
    int16_t &_value;
    uint8_t _factor;
};


class table_axis_iterator
{
public:

    inline table_axis_iterator& advance(uint8_t steps)
    {
        _pAxis = _pAxis + (_stride * steps);
        return *this;
    }

    inline table_axis_iterator& operator++()
    {
        return advance(1);
    }

    inline bool at_end() const
    {
        return _pAxis == _pAxisEnd;
    }

    inline int16_ref operator *() const
    {
        return int16_ref(*_pAxis, _axisFactor);
    }
    
    inline table_axis_iterator& reverse()
    {
        table3d_axis_t *_pOldAxis = _pAxis;
        _pAxis = _pAxisEnd - _stride;
        _pAxisEnd = _pOldAxis - _stride;
        _stride = (int8_t)(_stride * -1);
        return *this;
    }

    static table_axis_iterator y_begin(const table3d_axis_t *pAxis, table3d_dim_t size, uint8_t factor)
    {
        table_axis_iterator it;
        it._pAxis = const_cast<table3d_axis_t*>(pAxis)+(size-1);
        it._pAxisEnd = const_cast<table3d_axis_t*>(pAxis)-1;
        it._axisFactor = factor;
        it._stride = -1;
        return it;
    }

    static table_axis_iterator x_begin(const table3d_axis_t *pAxis, table3d_dim_t size, uint8_t factor)
    {
        table_axis_iterator it;
        it._pAxis = const_cast<table3d_axis_t*>(pAxis);
        it._pAxisEnd = const_cast<table3d_axis_t*>(pAxis)+size;
        it._axisFactor = factor;
        it._stride = 1;
        return it;
    }

private:
    table3d_axis_t *_pAxis;
    table3d_axis_t *_pAxisEnd;
    uint8_t _axisFactor;
    int8_t _stride;
};


// ========================= INTRA-ROW ITERATION ========================= 

// A table row is directly iterable & addressable.
class table_row_iterator {
public:

    table_row_iterator(table3d_value_t *pRowStart,  uint8_t rowWidth)
    : pValue(pRowStart), pEnd(pRowStart+rowWidth)
    {
    }

    inline table_row_iterator& advance(uint8_t steps)
    { 
        pValue  = pValue + steps;
        return *this;
    }

    inline table_row_iterator& operator++()
    {
        return advance(1);
    }

    inline bool at_end() const
    {
        return pValue == pEnd;
    }

    inline table3d_value_t& operator*() const
    {
        return *pValue;
    }

    table3d_value_t *pValue;
    table3d_value_t *pEnd;
};


// ========================= INTER-ROW ITERATION ========================= 

class table_value_iterator
{
public:

    table_value_iterator(const table3d_value_t *pValues, table3d_dim_t axisSize)
    : pRowsStart(const_cast<table3d_value_t*>(pValues) + (axisSize*(axisSize-1))),
      pRowsEnd(const_cast<table3d_value_t*>(pValues) - axisSize),
      rowWidth(axisSize)
    {
    }

    inline table_value_iterator& advance(uint8_t steps)
    {
        pRowsStart = pRowsStart - (rowWidth * steps);
        return *this;
    }

    inline table_value_iterator& operator++()
    {
        return advance(1);
    }

    inline table_row_iterator operator*() const
    {
        return table_row_iterator(pRowsStart, rowWidth);
    }

    inline bool at_end() const
    {
        return pRowsStart == pRowsEnd;
    }

private:
    table3d_value_t *pRowsStart;
    table3d_value_t *pRowsEnd;
    uint8_t rowWidth;
};
