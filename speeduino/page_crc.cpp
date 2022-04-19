#include "globals.h"
#include "page_crc.h"
#include "pages.h"
#include "table3d_axis_io.h"

typedef uint32_t (FastCRC32::*pCrcCalc)(const uint8_t *, const uint16_t, bool);

static inline uint32_t compute_raw_crc(const page_iterator_t &entity, pCrcCalc calcFunc)
{
    return (CRC32.*calcFunc)((uint8_t*)entity.pData, entity.size, false);
}

static inline uint32_t compute_row_crc(const table_row_iterator &row, pCrcCalc calcFunc)
{
    return (CRC32.*calcFunc)(&*row, row.size(), false);
}

static inline uint32_t compute_tablevalues_crc(table_value_iterator it, pCrcCalc calcFunc)
{
    uint32_t crc = compute_row_crc(*it, calcFunc);
    ++it;

    while (!it.at_end())
    {
        crc = compute_row_crc(*it, &FastCRC32::crc32_upd);
        ++it;
    }
    return crc;
}

static inline uint32_t compute_tableaxis_crc(table_axis_iterator it, uint32_t crc)
{
    const int16_byte *pConverter = table3d_axis_io::get_converter(it.domain());

    byte values[32]; // Fingers crossed we don't have a table bigger than 32x32
    byte *pValue = values;
    while (!it.at_end())
    {
        *pValue++ = pConverter->to_byte(*it);
        ++it;
    }
    return pValue-values==0 ? crc : CRC32.crc32_upd(values, pValue-values, false);
}

static inline uint32_t compute_table_crc(page_iterator_t &entity, pCrcCalc calcFunc)
{
    return compute_tableaxis_crc(y_begin(entity), 
                compute_tableaxis_crc(x_begin(entity),
                    compute_tablevalues_crc(rows_begin(entity), calcFunc)));
}

static inline uint32_t pad_crc(uint16_t padding, uint32_t crc)
{
    const uint8_t raw_value = 0u;
    while (padding>0)
    {
        crc = CRC32.crc32_upd(&raw_value, 1, false);
        --padding;
    }
    return crc;
}

static inline uint32_t compute_crc(page_iterator_t &entity, pCrcCalc calcFunc)
{
    switch (entity.type)
    {
    case Raw:
        return compute_raw_crc(entity, calcFunc);
        break;

    case Table:
        return compute_table_crc(entity, calcFunc);
        break;

    case NoEntity:
        return pad_crc(entity.size, 0U);
        break;

    default:
        abort();
        break;
    }
}

uint32_t calculatePageCRC32(byte pageNum)
{
  page_iterator_t entity = page_begin(pageNum);
  // Initial CRC calc
  uint32_t crc = compute_crc(entity, &FastCRC32::crc32);

  entity = advance(entity);
  while (entity.type!=End)
  {
    crc = compute_crc(entity, &FastCRC32::crc32_upd /* Note that we are *updating* */);
    entity = advance(entity);
  }
  return ~pad_crc(getPageSize(pageNum) - entity.size, crc);
}