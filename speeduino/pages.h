#pragma once
#include <Arduino.h>
#include "table3d.h"

/**
 * Page count, as defined in the INI file
 */
uint8_t getPageCount(); 

/**
 * Page size in bytes
 */
uint16_t getPageSize(byte pageNum /**< [in] The page number */ );

// These are the page numbers that the Tuner Studio serial protocol uses to transverse the different map and config pages.
#define veMapPage     2
#define veSetPage     1 //Note that this and the veMapPage were swapped in Feb 2019 as the 'algorithm' field must be declared in the ini before it's used in the fuel table
#define ignMapPage    3
#define ignSetPage    4//Config Page 2
#define afrMapPage    5
#define afrSetPage    6//Config Page 3
#define boostvvtPage  7
#define seqFuelPage   8
#define canbusPage    9//Config Page 9
#define warmupPage    10 //Config Page 10
#define fuelMap2Page  11
#define wmiMapPage    12
#define progOutsPage  13
#define ignMap2Page   14
#define boostvvtPage2 15


// ============================== Per-byte page access ==========================

/**
 * Gets a single value from a page, with data aligned as per the ini file
 */
byte getPageValue(  byte pageNum,       /**< [in] The page number to retrieve data from. */
                    uint16_t offset     /**< [in] The address in the page that should be returned. This is as per the page definition in the ini. */
                    );

/**
 * Sets a single value from a page, with data aligned as per the ini file
 */
void setPageValue(  byte pageNum,       /**< [in] The page number to retrieve data from. */
                    uint16_t offset,    /**< [in] The address in the page that should be returned. This is as per the page definition in the ini. */
                    byte value          /**< [in] The new value */
                    );

// ============================== Page Iteration ==========================

// A logical TS page is actually multiple in memory entities. Allow iteration
// over those entities.

// Type of entity
enum entity_type { 
    Raw,        // A block of memory
    Table,      // A 3D table
    NoEntity,   // No entity, but a valid offset
    End         // The offset was past any known entity for the page
};

// A entity on a logical page.
struct page_iterator_t {
    void *pData;
    table_type_t table_key;
    uint8_t page;   // The page the entity belongs to
    uint16_t start; // The start position of the entity, in bytes, from the start of the page
    uint16_t size;  // Size of the entity in bytes
    entity_type type;
};

/**
 * Initiates iteration over a pages entities.
 * Test `entity.type==End` to determine the end of the page.
 */
page_iterator_t page_begin(byte pageNum /**< [in] The page number to iterate over. */);

/**
 * Moves the iterator to the next sub-entity on the page
 */
page_iterator_t advance(const page_iterator_t &it /**< [in] The current iterator */);

/**
 * Convert page iterator to table value iterator.
 */
table_value_iterator rows_begin(const page_iterator_t &it);

/**
 * Convert page iterator to table x axis iterator.
 */
table_axis_iterator x_begin(const page_iterator_t &it);

/**
 * Convert page iterator to table y axis iterator.
 */
table_axis_iterator y_begin(const page_iterator_t &it);
