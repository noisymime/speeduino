//Backup sram stores data in the battery backuped sram portion. 
//The backup battery is available on the ebay stm32F407VET6 black boards.  
#ifndef BACKUPSRAMASEEPROM_H
#define BACKUPSRAMASEEPROM_H
#if defined(STM32F407xx)
#include <stdint.h>
#include "stm32f407xx.h"

class BackupSramAsEEPROM {
  private: 
    const uint16_t backup_size = 4096; //maximum of 4kb backuped sram available.
    int8_t write_byte( uint8_t *data, uint16_t bytes, uint16_t offset ); 
    int8_t read_byte( uint8_t *data, uint16_t bytes, uint16_t offset ); 
   
  public:
    BackupSramAsEEPROM();
    uint8_t read(uint16_t address);  
    int8_t write(uint16_t address, uint8_t val);
    int8_t update(uint16_t address, uint8_t val);
    uint16_t length();
    template< typename T > T &get( int idx, T &t ){
        uint16_t e = idx;
        uint8_t *ptr = (uint8_t*) &t;
        for( int count = sizeof(T) ; count ; --count, ++e )  *ptr++ = read(e);
        return t;
    }
    template< typename T > const T &put( int idx, const T &t ){        
        const uint8_t *ptr = (const uint8_t*) &t;
        uint16_t e = idx;
        for( int count = sizeof(T) ; count ; --count, ++e )  write(e, *ptr++);
        return t;
    }    
};

extern BackupSramAsEEPROM EEPROM;

#endif
#endif