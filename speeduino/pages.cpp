#include "pages.h"
#include "globals.h"
#include "utilities.h"
#include "table3d_axis_io.h"

// Maps from virtual page "addresses" to addresses/bytes of real in memory entities
//
// For TunerStudio:
// 1. Each page has a numeric identifier (0 to N-1)
// 2. A single page is a contiguous block of data.
// So individual bytes are identified by a (page number, offset)
//
// The TS layout is not what is in memory. E.g.
//
//    TS Page 2               |0123456789ABCD|0123456789ABCDEF|
//                                |                       |
//    Arduino In Memory  |--- Entity A ---|         |--- Entity B -----|
//
// Further, the in memory entity may also not be contiguous or in the same
// order that TS expects
//
// So there is a 2 stage mapping:
//  1. Page # + Offset to entity
//  2. Offset to intra-entity byte

// Page sizes as defined in the .ini file
constexpr const uint16_t PROGMEM ini_page_sizes[] = { 0, 128, 288, 288, 128, 288, 128, 240, 384, 192, 192, 288, 192, 128, 288, 256 };

// ========================= Table size calculations =========================
// Note that these should be computed at compile time, assuming the correct
// calling context.

template <class table_t>
inline constexpr uint16_t get_table_value_end()
{
  return table_t::xaxis_t::length*table_t::yaxis_t::length;
}
template <class table_t>
inline constexpr uint16_t get_table_axisx_end()
{
  return get_table_value_end<table_t>()+table_t::xaxis_t::length;
}
template <class table_t>
inline constexpr uint16_t get_table_axisy_end(const table_t *)
{
  return get_table_axisx_end<table_t>()+table_t::yaxis_t::length;
}

// ========================= Intra-table offset to byte class =========================

template<class table_t>
class offset_to_table
{
public:

  // This class encapsulates mapping a linear offset to the various parts of a table
  // and exposing the linear offset as an mutable byte.
  //
  // Tables do not map linearly to the TS page address space, so special 
  // handling is necessary (we do not use the normal array layout for
  // performance reasons elsewhere)
  //
  // We take the offset & map it to a single value, x-axis or y-axis element
  //
  // Using a template here is a performance boost - we can call functions that
  // are specialised per table type, which allows the compiler more optimisation
  // opportunities. See get_table_value().

  offset_to_table(table_t *pTable, uint16_t table_offset)
  : _pTable(pTable),
    _table_offset(table_offset)
  {    
  }

  // Getter
  inline byte operator*() const 
  { 
    switch (get_table_location())
    {
      case table_location_values:
        return get_value_value();
      case table_location_xaxis:
        return table3d_axis_io::to_byte(table_t::xaxis_t::domain, get_xaxis_value());
      case table_location_yaxis:
      default:
        return table3d_axis_io::to_byte(table_t::yaxis_t::domain, get_yaxis_value());
    }
  }

  // Setter
  inline offset_to_table &operator=( byte new_value )
  {
    switch (get_table_location())
    {
      case table_location_values:
        get_value_value() = new_value;
        break;

      case table_location_xaxis:
        get_xaxis_value() = table3d_axis_io::from_byte(table_t::xaxis_t::domain, new_value);
        break;

      case table_location_yaxis:
      default:
        get_yaxis_value() = table3d_axis_io::from_byte(table_t::yaxis_t::domain, new_value);
    }
    invalidate_cache(&_pTable->get_value_cache);
    return *this;
  }  

private: 

  inline byte& get_value_value() const
  {
    return _pTable->values.value_at((uint8_t)_table_offset);
  }

  inline table3d_axis_t& get_xaxis_value() const
  {
    return *(_pTable->axisX.begin().advance(_table_offset - get_table_value_end<table_t>()));
  }

  inline table3d_axis_t& get_yaxis_value() const
  {
    return *(_pTable->axisY.begin().advance(_table_offset - get_table_axisx_end<table_t>()));
  }

  enum table_location {
      table_location_values, table_location_xaxis, table_location_yaxis 
  };
  
  inline table_location get_table_location() const
  {
    if (_table_offset<get_table_value_end<table_t>())
    {
      return table_location_values;
    }
    if (_table_offset<get_table_axisx_end<table_t>())
    {
      return table_location_xaxis;
    }
    return table_location_yaxis;
  }

  table_t *_pTable;
  uint16_t _table_offset;
};

// ========================= Offset to entity byte mapping =========================

inline byte& get_raw_location(page_iterator_t &entity, uint16_t offset)
{
  return *((byte*)entity.pData + (offset-entity.start));
}

inline byte get_table_value(page_iterator_t &entity, uint16_t offset)
{
  #define CTA_GET_TABLE_VALUE(size, xDomain, yDomain, pTable, offset) \
      return *offset_to_table<TABLE3D_TYPENAME_BASE(size, xDomain, yDomain)>((TABLE3D_TYPENAME_BASE(size, xDomain, yDomain)*)pTable, offset);
  CONCRETE_TABLE_ACTION(entity.table_key, CTA_GET_TABLE_VALUE, entity.pData, (offset-entity.start));  
}

inline byte get_value(page_iterator_t &entity, uint16_t offset)
{
  if (Raw==entity.type)
  {
    return get_raw_location(entity, offset);
  }
  if (Table==entity.type)
  {
    return get_table_value(entity, offset);
  }
  return 0U;
}

inline void set_table_value(page_iterator_t &entity, uint16_t offset, byte new_value)
{
  #define CTA_SET_TABLE_VALUE(size, xDomain, yDomain, pTable, offset, new_value) \
      offset_to_table<TABLE3D_TYPENAME_BASE(size, xDomain, yDomain)>((TABLE3D_TYPENAME_BASE(size, xDomain, yDomain)*)pTable, offset) = new_value; break;
  CONCRETE_TABLE_ACTION(entity.table_key, CTA_SET_TABLE_VALUE, entity.pData, (offset-entity.start), new_value);  
}

inline void set_value(page_iterator_t &entity, byte value, uint16_t offset)
{    
  if (Raw==entity.type)
  {
    get_raw_location(entity, offset) = value;
  }
  else if (Table==entity.type)
  {
    set_table_value(entity, offset, value);
  }
}

// ========================= Static page size computation & checking ===================

// This will fail AND print the page number and required size
template <uint8_t pageNum, uint16_t min>
static inline void check_size() {
  static_assert(ini_page_sizes[pageNum] >= min, "Size is off!");
}

// Since pages are a logical contiguous block, we can automatically compute the 
// logical start address of every item: the first one starts at zero, following
// items must start at the end of the previous.
#define _ENTITY_START(entityNum) entity ## entityNum ## Start
#define ENTITY_START_VAR(entityNum) _ENTITY_START(entityNum)
// Compute the start address of the next entity. We need this to be a constexpr
// so we can static assert on it later. So we cannot increment an exiting var.
#define DECLARE_NEXT_ENTITY_START(entityIndex, entitySize) \
  constexpr uint16_t ENTITY_START_VAR( PP_INC(entityIndex) ) = ENTITY_START_VAR(entityIndex)+entitySize;

// ========================= Logical page end processing ===================

// The members of all page_iterator_t instances are compile time constants and
// thus all page_iterator_t instances *could* be compile time constants. 
//
// If we declare them inline as part of return statements, gcc recognises they 
// are constants (even without constexpr). Constants need to be stored somewhere:
// gcc places them in the .data section, which is placed in SRAM :-(. 
//
// So we would end up using several hundred bytes of SRAM. 
//
// Instead we use this (and other) intermediate factory function(s) - it provides a barrier that
// forces GCC to construct the page_iterator_t instance at runtime.
inline const page_iterator_t create_end_iterator(uint8_t pageNum, uint16_t start)
{
  return page_iterator_t {
    .pData = nullptr,
    .table_key = table_type_None,
    .page = pageNum,
    .start = start,
    .size = start,
    .type = End,
  };
}

// Signal the end of a page
#define END_OF_PAGE(pageNum, entityNum) \
  check_size<pageNum, ENTITY_START_VAR(entityNum)>(); \
  return create_end_iterator(pageNum, ENTITY_START_VAR(entityNum)); \

// ========================= Table processing  ===================

inline const page_iterator_t create_table_iterator(void *pTable, table_type_t key, uint8_t pageNum, uint16_t start, uint16_t size)
{
  return page_iterator_t {
    .pData = pTable,
    .table_key = key,
    .page = pageNum,
    .start = start,
    .size = size,
    .type = Table,
  };
}

// If the offset is in range, create a Table entity_t
#define CHECK_TABLE(pageNum, offset, pTable, entityNum) \
  if (offset < ENTITY_START_VAR(entityNum)+get_table_axisy_end(pTable)) \
  { \
    return create_table_iterator(pTable, (pTable)->type_key, \
                                  pageNum, \
                                  ENTITY_START_VAR(entityNum), get_table_axisy_end(pTable)); \
  } \
  DECLARE_NEXT_ENTITY_START(entityNum, get_table_axisy_end(pTable))

// ========================= Raw memory block processing  ===================

inline const page_iterator_t create_raw_iterator(void *pBuffer, uint8_t pageNum, uint16_t start, uint16_t size)
{
  return page_iterator_t {
    .pData = pBuffer,
    .table_key = table_type_None,
    .page = pageNum,
    .start = start,
    .size = size,
    .type = Raw,
  };
}

// If the offset is in range, create a Raw entity_t
#define CHECK_RAW(pageNum, offset, pDataBlock, blockSize, entityNum) \
  if (offset < ENTITY_START_VAR(entityNum)+blockSize) \
  { \
    return create_raw_iterator(pDataBlock, pageNum, ENTITY_START_VAR(entityNum), blockSize);\
  } \
  DECLARE_NEXT_ENTITY_START(entityNum, blockSize)

// ===============================================================================

// Does the heavy lifting of mapping page+offset to an entity
//
// Alternative implementation would be to encode the mapping into data structures
// That uses flash memory, which is scarce. And it was too slow.
static inline __attribute__((always_inline)) // <-- this is critical for performance
page_iterator_t map_page_offset_to_entity(uint8_t pageNumber, uint16_t offset)
{
  // The start address of the 1st entity in any page.
  static constexpr uint16_t ENTITY_START_VAR(0) = 0U;

  switch (pageNumber)
  {
    case 0:
      END_OF_PAGE(0, 0)

    case veMapPage:
    {
      CHECK_TABLE(veMapPage, offset, &fuelTable, 0)
      END_OF_PAGE(veMapPage, 1)
    }

    case ignMapPage: //Ignition settings page (Page 2)
    {
      CHECK_TABLE(ignMapPage, offset, &ignitionTable, 0)
      END_OF_PAGE(ignMapPage, 1)
    }

    case afrMapPage: //Air/Fuel ratio target settings page
    {
      CHECK_TABLE(afrMapPage, offset, &afrTable, 0)
      END_OF_PAGE(afrMapPage, 1)
    }

    case boostvvtPage: //Boost, VVT and staging maps (all 8x8)
    {
      CHECK_TABLE(boostvvtPage, offset, &boostTable, 0)
      CHECK_TABLE(boostvvtPage, offset, &vvtTable, 1)
      CHECK_TABLE(boostvvtPage, offset, &stagingTable, 2)
      END_OF_PAGE(boostvvtPage, 3)
    }

    case seqFuelPage:
    {
      CHECK_TABLE(seqFuelPage, offset, &trim1Table, 0)
      CHECK_TABLE(seqFuelPage, offset, &trim2Table, 1)
      CHECK_TABLE(seqFuelPage, offset, &trim3Table, 2)
      CHECK_TABLE(seqFuelPage, offset, &trim4Table, 3)
      CHECK_TABLE(seqFuelPage, offset, &trim5Table, 4)
      CHECK_TABLE(seqFuelPage, offset, &trim6Table, 5)
      CHECK_TABLE(seqFuelPage, offset, &trim7Table, 6)
      CHECK_TABLE(seqFuelPage, offset, &trim8Table, 7)
      END_OF_PAGE(seqFuelPage, 8)
    }

    case fuelMap2Page:
    {
      CHECK_TABLE(fuelMap2Page, offset, &fuelTable2, 0)
      END_OF_PAGE(fuelMap2Page, 1)
    }

    case wmiMapPage:
    {
      CHECK_TABLE(wmiMapPage, offset, &wmiTable, 0)
      CHECK_TABLE(wmiMapPage, offset, &vvt2Table, 1)
      CHECK_TABLE(wmiMapPage, offset, &dwellTable, 2)
      END_OF_PAGE(wmiMapPage, 3)
    }
    
    case ignMap2Page:
    {
      CHECK_TABLE(ignMap2Page, offset, &ignitionTable2, 0)
      END_OF_PAGE(ignMap2Page, 1)
    }

    case veSetPage: 
    {
      CHECK_RAW(veSetPage, offset, &configPage2, sizeof(configPage2), 0)
      END_OF_PAGE(veSetPage, 1)
    }

    case ignSetPage: 
    {
      CHECK_RAW(ignSetPage, offset, &configPage4, sizeof(configPage4), 0)
      END_OF_PAGE(ignSetPage, 1)
    }
    
    case afrSetPage: 
    {
      CHECK_RAW(afrSetPage, offset, &configPage6, sizeof(configPage6), 0)
      END_OF_PAGE(afrSetPage, 1)
    }

    case canbusPage:  
    {
      CHECK_RAW(canbusPage, offset, &configPage9, sizeof(configPage9), 0)
      END_OF_PAGE(canbusPage, 1)
    }

    case warmupPage: 
    {
      CHECK_RAW(warmupPage, offset, &configPage10, sizeof(configPage10), 0)
      END_OF_PAGE(warmupPage, 1)
    }

    case progOutsPage: 
    {
      CHECK_RAW(progOutsPage, offset, &configPage13, sizeof(configPage13), 0)
      END_OF_PAGE(progOutsPage, 1)
    }

    case boostvvtPage2: //Boost, VVT and staging maps (all 8x8)
    {
      CHECK_TABLE(boostvvtPage2, offset, &boostTableLookupDuty, 0)
      CHECK_RAW(boostvvtPage2, offset, &configPage15, sizeof(configPage15), 1)
      END_OF_PAGE(boostvvtPage2, 2)
    }

    default:
      abort(); // Unknown page number. Not a lot we can do.
      break;
  }
}


// ====================================== External functions  ====================================

uint8_t getPageCount()
{
  return _countof(ini_page_sizes);
}

uint16_t getPageSize(byte pageNum)
{
  return pgm_read_word(&(ini_page_sizes[pageNum]));
}

void setPageValue(byte pageNum, uint16_t offset, byte value)
{
  page_iterator_t entity = map_page_offset_to_entity(pageNum, offset);

  set_value(entity, value, offset);
}

byte getPageValue(byte pageNum, uint16_t offset)
{
  page_iterator_t entity = map_page_offset_to_entity(pageNum, offset);

  return get_value(entity, offset);
}

// Support iteration over a pages entities.
// Check for entity.type==End
page_iterator_t page_begin(byte pageNum)
{
  return map_page_offset_to_entity(pageNum, 0U);
}

page_iterator_t advance(const page_iterator_t &it)
{
    return map_page_offset_to_entity(it.page, it.start+it.size);
}

/**
 * Convert page iterator to table value iterator.
 */
table_value_iterator rows_begin(const page_iterator_t &it)
{
  return rows_begin(it.pData, it.table_key);
}

/**
 * Convert page iterator to table x axis iterator.
 */
table_axis_iterator x_begin(const page_iterator_t &it)
{
  return x_begin(it.pData, it.table_key);
}

/**
 * Convert page iterator to table y axis iterator.
 */
table_axis_iterator y_begin(const page_iterator_t &it)
{
  return y_begin(it.pData, it.table_key);
}
