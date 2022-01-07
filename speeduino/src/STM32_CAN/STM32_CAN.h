/*
This is universal CAN library for STM32 that was made to be used with Speeduino EFI.
It should support all STM32 MCUs that are also supported in stm32duino Arduino_Core_STM32 and supports up to 3x CAN busses.
The library is created because at least currently (year 2021) there is no official CAN library in the STM32 core.
This library is based on several STM32 CAN example libraries linked below and it has been combined with few
things from Teensy FlexCAN library to make it compatible with the CAN features that exist in speeduino for Teensy.
Links to repositories that have helped with this:
https://github.com/nopnop2002/Arduino-STM32-CAN
https://github.com/J-f-Jensen/libraries/tree/master/STM32_CAN
https://github.com/jiauka/STM32F1_CAN

STM32 core: https://github.com/stm32duino/Arduino_Core_STM32
*/
#if HAL_CAN_MODULE_ENABLED
#ifndef STM32_CAN_H
#define STM32_CAN_H

#include <Arduino.h>

// This struct is directly copied from Teensy FlexCAN library to retain compatibility with it. Not all are in use with STM32.
// Source: https://github.com/tonton81/FlexCAN_T4/

typedef struct CAN_message_t {
  uint32_t id = 0;         // can identifier
  uint16_t timestamp = 0;  // time when message arrived
  uint8_t idhit = 0;       // filter that id came from
  struct {
    bool extended = 0;     // identifier is extended (29-bit)
    bool remote = 0;       // remote transmission request packet type
    bool overrun = 0;      // message overrun
    bool reserved = 0;
  } flags;
  uint8_t len = 8;         // length of data
  uint8_t buf[8] = { 0 };  // data
  int8_t mb = 0;           // used to identify mailbox reception
  uint8_t bus = 1;         // used to identify where the message came (CAN1, CAN2 or CAN3)
  bool seq = 0;            // sequential frames
} CAN_message_t;

typedef enum CAN_PINS {DEF, ALT, ALT_2,} CAN_PINS;

typedef enum RXQUEUE_TABLE {
  RX_SIZE_2 = (uint16_t)2,
  RX_SIZE_4 = (uint16_t)4,
  RX_SIZE_8 = (uint16_t)8,
  RX_SIZE_16 = (uint16_t)16,
  RX_SIZE_32 = (uint16_t)32,
  RX_SIZE_64 = (uint16_t)64,
  RX_SIZE_128 = (uint16_t)128,
  RX_SIZE_256 = (uint16_t)256,
  RX_SIZE_512 = (uint16_t)512,
  RX_SIZE_1024 = (uint16_t)1024
} RXQUEUE_TABLE;

typedef enum TXQUEUE_TABLE {
  TX_SIZE_2 = (uint16_t)2,
  TX_SIZE_4 = (uint16_t)4,
  TX_SIZE_8 = (uint16_t)8,
  TX_SIZE_16 = (uint16_t)16,
  TX_SIZE_32 = (uint16_t)32,
  TX_SIZE_64 = (uint16_t)64,
  TX_SIZE_128 = (uint16_t)128,
  TX_SIZE_256 = (uint16_t)256,
  TX_SIZE_512 = (uint16_t)512,
  TX_SIZE_1024 = (uint16_t)1024
} TXQUEUE_TABLE;

/* Teensy FlexCAN uses Mailboxes for different RX filters, but in STM32 there is Filter Banks. These work practically same way,
so the Filter Banks are named as mailboxes in "setMBFilter" -functions, to retain compatibility with Teensy FlexCAN library.
*/
typedef enum CAN_BANK {
  MB0 = 0,
  MB1 = 1,
  MB2 = 2,
  MB3 = 3,
  MB4 = 4,
  MB5 = 5,
  MB6 = 6,
  MB7 = 7,
  MB8 = 8,
  MB9 = 9,
  MB10 = 10,
  MB11 = 11,
  MB12 = 12,
  MB13 = 13,
  MB14 = 14,
  MB15 = 15,
  MB16 = 16,
  MB17 = 17,
  MB18 = 18,
  MB19 = 19,
  MB20 = 20,
  MB21 = 21,
  MB22 = 22,
  MB23 = 23,
  MB24 = 24,
  MB25 = 25,
  MB26 = 26,
  MB27 = 27
} CAN_BANK;

typedef enum CAN_FLTEN {
  ACCEPT_ALL = 0,
  REJECT_ALL = 1
} CAN_FLTEN;

class STM32_CAN {

  public:
    // Default buffer sizes are set to 16. But this can be changed by using constructor in main code.
    STM32_CAN(CAN_TypeDef* canPort, CAN_PINS pins, RXQUEUE_TABLE rxSize = RX_SIZE_16, TXQUEUE_TABLE txSize = TX_SIZE_16);
    // Begin. By default the automatic retransmission is enabled. If it causes problems, use begin(false) to disable it.
    void begin(bool retransmission = true);
    void setBaudRate(uint32_t baud);
    bool write(CAN_message_t &CAN_tx_msg, bool sendMB = false);
    bool read(CAN_message_t &CAN_rx_msg);
    // Manually set STM32 filter bank parameters
    bool setFilter(uint8_t bank_num, uint32_t filter_id, uint32_t mask, uint32_t filter_mode = CAN_FILTERMODE_IDMASK, uint32_t filter_scale = CAN_FILTERSCALE_32BIT, uint32_t fifo = CAN_FILTER_FIFO0);
    // Teensy FlexCAN style "set filter" -functions
    bool setMBFilterProcessing(CAN_BANK bank_num, uint32_t filter_id, uint32_t mask);
    void setMBFilter(CAN_FLTEN input); /* enable/disable traffic for all MBs (for individual masking) */
    void setMBFilter(CAN_BANK bank_num, CAN_FLTEN input); /* set specific MB to accept/deny traffic */
    bool setMBFilter(CAN_BANK bank_num, uint32_t id1); /* input 1 ID to be filtered */
    bool setMBFilter(CAN_BANK bank_num, uint32_t id1, uint32_t id2); /* input 2 ID's to be filtered */

    void enableLoopBack(bool yes = 1);
    void enableSilentMode(bool yes = 1);
    void enableSilentLoopBack(bool yes = 1);
    void enableFIFO(bool status = 1);
    void enableMBInterrupts();
    void disableMBInterrupts();
 
    // These are public because these are also used from interupts.
    typedef struct RingbufferTypeDef {
      volatile uint16_t head;
      volatile uint16_t tail;
      uint16_t size;
      volatile CAN_message_t *buffer;
    } RingbufferTypeDef;
  
    RingbufferTypeDef rxRing;
    RingbufferTypeDef txRing;

    bool addToRingBuffer(RingbufferTypeDef &ring, const CAN_message_t &msg);
    bool removeFromRingBuffer(RingbufferTypeDef &ring, CAN_message_t &msg);

  protected:
    uint16_t sizeRxBuffer;
    uint16_t sizeTxBuffer;
  
  private:
    void      initializeFilters();
    bool      isInitialized() { return rx_buffer != 0; }
    void      initRingBuffer(RingbufferTypeDef &ring, volatile CAN_message_t *buffer, uint32_t size);
    void      initializeBuffers(void);
    bool      isRingBufferEmpty(RingbufferTypeDef &ring);
    uint32_t  ringBufferCount(RingbufferTypeDef &ring);
    void      calculateBaudrate(CAN_HandleTypeDef *CanHandle, int Baudrate);
    uint32_t  getAPB1Clock(void);

    volatile CAN_message_t *rx_buffer;
    volatile CAN_message_t *tx_buffer;
    
    bool     _canIsActive = false;
    CAN_PINS _pins;

    CAN_HandleTypeDef *n_pCanHandle;
    CAN_TypeDef*      _canPort;

};

static STM32_CAN* _CAN1 = nullptr;
static CAN_HandleTypeDef     hcan1;
#ifdef CAN2
static STM32_CAN* _CAN2 = nullptr;
static CAN_HandleTypeDef     hcan2;
#endif
#ifdef CAN3
static STM32_CAN* _CAN3 = nullptr;
static CAN_HandleTypeDef     hcan3;
#endif

#endif
#endif
