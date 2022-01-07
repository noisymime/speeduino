#if HAL_CAN_MODULE_ENABLED
#include "STM32_CAN.h"

STM32_CAN::STM32_CAN( CAN_TypeDef* canPort, CAN_PINS pins, RXQUEUE_TABLE rxSize, TXQUEUE_TABLE txSize ) {

  if (_canIsActive) { return; }
  
  sizeRxBuffer=rxSize;
  sizeTxBuffer=txSize;
  
  if (canPort == CAN1)
  {
    _CAN1 = this;
    n_pCanHandle = &hcan1;
  }
  #ifdef CAN2
  if( canPort == CAN2)
  {
    _CAN2 = this;
    n_pCanHandle = &hcan2;
  }
  #endif
  #ifdef CAN3
  if (canPort == CAN3)
  {
    _CAN3 = this;
    n_pCanHandle = &hcan3;
  }
  #endif

  _canPort = canPort;
  _pins = pins;
}

// Init and start CAN
void STM32_CAN::begin( bool retransmission ) {

  // exit if CAN already is active
  if (_canIsActive) return;
  
  _canIsActive = true;
  
  GPIO_InitTypeDef GPIO_InitStruct;
  
  initializeBuffers();
  
  // Configure CAN
  if (_canPort == CAN1)
  {
    //CAN1
    __HAL_RCC_CAN1_CLK_ENABLE();
    
    if (_pins == ALT)
    {
      __HAL_RCC_GPIOB_CLK_ENABLE();
      #if defined(__HAL_RCC_AFIO_CLK_ENABLE)  // Some MCUs like F1xx uses AFIO to set pins, so if there is AFIO defined, we use that.
      __HAL_AFIO_REMAP_CAN1_2();  // To use PB8/9 pins for CAN1.
      __HAL_RCC_AFIO_CLK_ENABLE();
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  // If AFIO is used, there doesn't seem to be "very high" option for speed, so we use "high" -setting.
      GPIO_InitStruct.Pin = GPIO_PIN_8;
      GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
      GPIO_InitStruct.Pin = GPIO_PIN_9;
      #else  // Without AFIO, this is the way to set the pins for CAN.
      GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
      GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      #endif
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    } 
     if (_pins == DEF)
     {
      __HAL_RCC_GPIOA_CLK_ENABLE();
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      #if defined(__HAL_RCC_AFIO_CLK_ENABLE)
      __HAL_AFIO_REMAP_CAN1_1(); // To use PA11/12 pins for CAN1.
      __HAL_RCC_AFIO_CLK_ENABLE();
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
      GPIO_InitStruct.Pin = GPIO_PIN_11;
      GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
      GPIO_InitStruct.Pin = GPIO_PIN_12;
      #else
      GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
      GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      #endif
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }

     if (_pins == ALT_2)
     {
      __HAL_RCC_GPIOD_CLK_ENABLE();
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      #if defined(__HAL_RCC_AFIO_CLK_ENABLE)
      __HAL_AFIO_REMAP_CAN1_3(); // To use PD0/1 pins for CAN1.
      __HAL_RCC_AFIO_CLK_ENABLE();
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
      GPIO_InitStruct.Pin = GPIO_PIN_0;
      GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
      HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
      GPIO_InitStruct.Pin = GPIO_PIN_1;
      #else
      GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
      GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      #endif
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    }

    // NVIC configuration for CAN1 Reception complete interrupt
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 15, 0); // 15 is lowest possible priority
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn );
    // NVIC configuration for CAN1 Transmission complete interrupt
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 15, 0); // 15 is lowest possible priority
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    
    n_pCanHandle->Instance = CAN1;
  }
#ifdef CAN2  
  else if (_canPort == CAN2)
  {
    //CAN2
    __HAL_RCC_CAN1_CLK_ENABLE(); // CAN1 clock needs to be enabled too, because CAN2 works as CAN1 slave.
    __HAL_RCC_CAN2_CLK_ENABLE();
    if (_pins == ALT)
    {
      __HAL_RCC_GPIOB_CLK_ENABLE();
      #if defined(__HAL_RCC_AFIO_CLK_ENABLE)
      __HAL_AFIO_REMAP_CAN2_ENABLE(); // To use PB5/6 pins for CAN2. Don't ask me why this has different name than for CAN1.
      __HAL_RCC_AFIO_CLK_ENABLE();
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
      GPIO_InitStruct.Pin = GPIO_PIN_5;
      GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
      GPIO_InitStruct.Pin = GPIO_PIN_6;
      #else
      GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
      GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      #endif
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    } 
    if (_pins == DEF) {
      __HAL_RCC_GPIOB_CLK_ENABLE();
      #if defined(__HAL_RCC_AFIO_CLK_ENABLE)
      __HAL_AFIO_REMAP_CAN2_DISABLE(); // To use PB12/13 pins for CAN2.
      __HAL_RCC_AFIO_CLK_ENABLE();
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
      GPIO_InitStruct.Pin = GPIO_PIN_12;
      GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
      GPIO_InitStruct.Pin = GPIO_PIN_13;
      #else
      GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
      GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      #endif
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }

    // NVIC configuration for CAN2 Reception complete interrupt
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 15, 0); // 15 is lowest possible priority
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn );
    // NVIC configuration for CAN2 Transmission complete interrupt
    HAL_NVIC_SetPriority(CAN2_TX_IRQn, 15, 0); // 15 is lowest possible priority
    HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);
    
    n_pCanHandle->Instance = CAN2;
  }
#endif

#ifdef CAN3 
  else if (_canPort == CAN3)
  {
    //CAN3
    __HAL_RCC_CAN3_CLK_ENABLE();
    if (_pins == ALT)
    {
      __HAL_RCC_GPIOB_CLK_ENABLE();
      GPIO_InitStruct.Alternate = GPIO_AF11_CAN3;
      GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    } 
    if (_pins == DEF)
    {
      __HAL_RCC_GPIOA_CLK_ENABLE();
      GPIO_InitStruct.Alternate = GPIO_AF11_CAN3;
      GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_15;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }

    // NVIC configuration for CAN3 Reception complete interrupt
    HAL_NVIC_SetPriority(CAN3_RX0_IRQn, 15, 0); // 15 is lowest possible priority
    HAL_NVIC_EnableIRQ(CAN3_RX0_IRQn );
    // NVIC configuration for CAN3 Transmission complete interrupt
    HAL_NVIC_SetPriority(CAN3_TX_IRQn, 15, 0); // 15 is lowest possible priority
    HAL_NVIC_EnableIRQ(CAN3_TX_IRQn);
    
    n_pCanHandle->Instance = CAN3;
  }
#endif

  n_pCanHandle->Init.TimeTriggeredMode = DISABLE;
  n_pCanHandle->Init.AutoBusOff = DISABLE;
  n_pCanHandle->Init.AutoWakeUp = DISABLE;
  if (retransmission){ n_pCanHandle->Init.AutoRetransmission  = ENABLE; }
  else { n_pCanHandle->Init.AutoRetransmission  = DISABLE; }
  n_pCanHandle->Init.ReceiveFifoLocked  = DISABLE;
  n_pCanHandle->Init.TransmitFifoPriority = ENABLE;
  n_pCanHandle->Init.Mode = CAN_MODE_NORMAL;
}

void STM32_CAN::setBaudRate(uint32_t baud)
{

  // Calculate and set baudrate
  calculateBaudrate( n_pCanHandle, baud );

  // Initializes CAN
  HAL_CAN_Init( n_pCanHandle );
  
  initializeFilters();

  // Start the CAN peripheral
  HAL_CAN_Start( n_pCanHandle );
  
  // Activate CAN RX notification
  HAL_CAN_ActivateNotification( n_pCanHandle, CAN_IT_RX_FIFO0_MSG_PENDING);
 
  // Activate CAN TX notification
  HAL_CAN_ActivateNotification( n_pCanHandle, CAN_IT_TX_MAILBOX_EMPTY);
}

bool STM32_CAN::write(CAN_message_t &CAN_tx_msg, bool sendMB)
{
  bool ret = true;
  uint32_t TxMailbox;
  CAN_TxHeaderTypeDef TxHeader;
 
  __HAL_CAN_DISABLE_IT(n_pCanHandle, CAN_IT_TX_MAILBOX_EMPTY);

  if (CAN_tx_msg.flags.extended == 1) // Extended ID when CAN_tx_msg.flags.extended is 1
  {
      TxHeader.ExtId = CAN_tx_msg.id;
      TxHeader.IDE   = CAN_ID_EXT;
  }
  else // Standard ID otherwise
  {
      TxHeader.StdId = CAN_tx_msg.id;
      TxHeader.IDE   = CAN_ID_STD;
  }
  
  TxHeader.RTR   = CAN_RTR_DATA;
  TxHeader.DLC   = CAN_tx_msg.len;
  TxHeader.TransmitGlobalTime = DISABLE;

  if(HAL_CAN_AddTxMessage( n_pCanHandle, &TxHeader, CAN_tx_msg.buf, &TxMailbox) != HAL_OK)
  {
    /* in normal situation we add up the message to TX ring buffer, if there is no free TX mailbox. But the TX mailbox interrupt is using this same function
    to move the messages from ring buffer to empty TX mailboxes, so for that use case, there is this check */
    if(sendMB != true) 
    {
      if( addToRingBuffer(txRing, CAN_tx_msg) == false )
      {
        ret = false; // no more room
      }
    }
    else { ret = false; }
  }
  __HAL_CAN_ENABLE_IT(n_pCanHandle, CAN_IT_TX_MAILBOX_EMPTY);
  return ret;
}

bool STM32_CAN::read(CAN_message_t &CAN_rx_msg)
{
  bool ret;
  __HAL_CAN_DISABLE_IT(n_pCanHandle, CAN_IT_RX_FIFO0_MSG_PENDING); 
  ret = removeFromRingBuffer(rxRing, CAN_rx_msg);
  __HAL_CAN_ENABLE_IT(n_pCanHandle, CAN_IT_RX_FIFO0_MSG_PENDING); 
  return ret;
}

bool STM32_CAN::setFilter(uint8_t bank_num, uint32_t filter_id, uint32_t mask, uint32_t filter_mode, uint32_t filter_scale, uint32_t fifo)
{
  CAN_FilterTypeDef sFilterConfig;

  sFilterConfig.FilterBank = bank_num;
  sFilterConfig.FilterMode = filter_mode;
  sFilterConfig.FilterScale = filter_scale;
  sFilterConfig.FilterFIFOAssignment = fifo;
  sFilterConfig.FilterActivation = ENABLE;

  if (filter_id <= 0x7FF)
  {
    // Standard ID can be only 11 bits long
    sFilterConfig.FilterIdHigh = (uint16_t) (filter_id << 5);
    sFilterConfig.FilterIdLow = 0;
    sFilterConfig.FilterMaskIdHigh = (uint16_t) (mask << 5);
    sFilterConfig.FilterMaskIdLow = CAN_ID_EXT;
  }
  else
  {
    // Extended ID
    sFilterConfig.FilterIdLow = (uint16_t) (filter_id << 3);
    sFilterConfig.FilterIdLow |= CAN_ID_EXT;
    sFilterConfig.FilterIdHigh = (uint16_t) (filter_id >> 13);
    sFilterConfig.FilterMaskIdLow = (uint16_t) (mask << 3);
    sFilterConfig.FilterMaskIdLow |= CAN_ID_EXT;
    sFilterConfig.FilterMaskIdHigh = (uint16_t) (mask >> 13);
  }

  // Enable filter
  if (HAL_CAN_ConfigFilter( n_pCanHandle, &sFilterConfig ) != HAL_OK)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

void STM32_CAN::setMBFilter(CAN_BANK bank_num, CAN_FLTEN input)
{
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterBank = uint8_t(bank_num);
  if (input = ACCEPT_ALL) { sFilterConfig.FilterActivation = ENABLE; }
  else { sFilterConfig.FilterActivation = DISABLE; }
  
  HAL_CAN_ConfigFilter(n_pCanHandle, &sFilterConfig);
}

void STM32_CAN::setMBFilter(CAN_FLTEN input)
{
  CAN_FilterTypeDef sFilterConfig;
  uint8_t max_bank_num = 27;
  uint8_t min_bank_num = 0;
  #ifdef CAN2
  if (_canPort == CAN1){ max_bank_num = 13;}
  else if (_canPort == CAN2){ min_bank_num = 14;}
  #endif
  for (uint8_t bank_num = min_bank_num ; bank_num <= max_bank_num ; bank_num++)
  {
    sFilterConfig.FilterBank = bank_num;
    if (input = ACCEPT_ALL) { sFilterConfig.FilterActivation = ENABLE; }
    else { sFilterConfig.FilterActivation = DISABLE; }
    HAL_CAN_ConfigFilter(n_pCanHandle, &sFilterConfig);
  }
}

bool STM32_CAN::setMBFilterProcessing(CAN_BANK bank_num, uint32_t filter_id, uint32_t mask)
{
  // just convert the MB number enum to bank number.
  return setFilter(uint8_t(bank_num), filter_id, mask);
}

bool STM32_CAN::setMBFilter(CAN_BANK bank_num, uint32_t id1)
{
  // by setting the mask to 0x1FFFFFFF we only filter the ID set as Filter ID.
  return setFilter(uint8_t(bank_num), id1, 0x1FFFFFFF);
}

bool STM32_CAN::setMBFilter(CAN_BANK bank_num, uint32_t id1, uint32_t id2)
{
  // if we set the filter mode as IDLIST, the mask becomes filter ID too. So we can filter two totally independent IDs in same bank.
  return setFilter(uint8_t(bank_num), id1, id2, CAN_FILTERMODE_IDLIST);
}

// TBD, do this using "setFilter" -function
void STM32_CAN::initializeFilters()
{
  CAN_FilterTypeDef sFilterConfig;
  // We set first bank to accept all RX messages
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  #ifdef CAN2
  // Filter banks from 14 to 27 are for Can2, so first for Can2 is bank 14. This is not relevant for devices with only one CAN
  if (_canPort == CAN1)
  {
    sFilterConfig.SlaveStartFilterBank = 14;
  }
  if (_canPort == CAN2)
  {
    sFilterConfig.FilterBank = 14;
  } 
  #endif
  
  HAL_CAN_ConfigFilter(n_pCanHandle, &sFilterConfig);
}

void STM32_CAN::initializeBuffers()
{
    if(isInitialized()) { return; }
  
    // set up the transmit and receive ring buffers
    if(tx_buffer==0)
    {
      tx_buffer=new CAN_message_t[sizeTxBuffer];
    }
    initRingBuffer(txRing, tx_buffer, sizeTxBuffer);

    if(rx_buffer==0)
    {
      rx_buffer=new CAN_message_t[sizeRxBuffer];
    }
    initRingBuffer(rxRing, rx_buffer, sizeRxBuffer);
}

void STM32_CAN::initRingBuffer(RingbufferTypeDef &ring, volatile CAN_message_t *buffer, uint32_t size)
{
    ring.buffer = buffer;
    ring.size = size;
    ring.head = 0;
    ring.tail = 0;
}

bool STM32_CAN::addToRingBuffer(RingbufferTypeDef &ring, const CAN_message_t &msg)
{
    uint16_t nextEntry;
    nextEntry =(ring.head + 1) % ring.size;

    // check if the ring buffer is full
    if(nextEntry == ring.tail)
	{
        return(false);
    }

    // add the element to the ring */
    memcpy((void *)&ring.buffer[ring.head],(void *)&msg, sizeof(CAN_message_t));

    // bump the head to point to the next free entry
    ring.head = nextEntry;

    return(true);
}
bool STM32_CAN::removeFromRingBuffer(RingbufferTypeDef &ring, CAN_message_t &msg)
{
    // check if the ring buffer has data available
    if(isRingBufferEmpty(ring) == true)
    {
        return(false);
    }

    // copy the message
    memcpy((void *)&msg,(void *)&ring.buffer[ring.tail], sizeof(CAN_message_t));

    // bump the tail pointer
    ring.tail =(ring.tail + 1) % ring.size;
    return(true);
}

bool STM32_CAN::isRingBufferEmpty(RingbufferTypeDef &ring)
{
    if(ring.head == ring.tail)
	{
        return(true);
    }

    return(false);
}

uint32_t STM32_CAN::ringBufferCount(RingbufferTypeDef &ring)
{
    int32_t entries;
    entries = ring.head - ring.tail;

    if(entries < 0)
    {
        entries += ring.size;
    }
    return((uint32_t)entries);
}

void STM32_CAN::calculateBaudrate(CAN_HandleTypeDef *CanHandle, int baud)
{
  /* this function calculates the needed Sync Jump Width, Time segments 1 and 2 and prescaler values based on the set baud rate and APB1 clock.
  This could be done faster if needed by calculating these values beforehand and just using fixed values from table.
  The function has been optimized to give values that have sample-point between 75-94%. If some other sample-point percentage is needed, this needs to be adjusted.
  More info about this topic here: http://www.bittiming.can-wiki.info/
  */
  int sjw = 1;
  int bs1 = 5; // optimization. bs1 smaller than 5 does give too small sample-point percentages.
  int bs2 = 1;
  int prescaler = 1;
  uint32_t _SyncJumpWidth;
  uint32_t _TimeSeg1;
  uint32_t _TimeSeg2;
  uint32_t _Prescaler;
  
  bool shouldBrake = false;
  
  uint32_t frequency = getAPB1Clock();
  
  for (; sjw <= 4 && !shouldBrake;)
  {
    for (; prescaler <= 1024 && !shouldBrake;)
    {
      for (; bs2 <= 3 && !shouldBrake;)  // Time segment 2 can get up to 8, but that causes too small sample-point percentages, so this is limited to 3.
      {
        for (; bs1 <= 15 && !shouldBrake;)  // Time segment 1 can get up to 16, but that causes too big sample-point percenages, so this is limited to 15.
        {
          int calcBaudrate = (int)(frequency / (prescaler * (sjw + bs1 + bs2)));
          
          if (calcBaudrate == baud)
          {
            switch (sjw)
            {
              case 1:
                _SyncJumpWidth = CAN_SJW_1TQ;
                break;
              case 2:
                _SyncJumpWidth = CAN_SJW_2TQ;
                break;
              case 3:
                _SyncJumpWidth = CAN_SJW_3TQ;
                break;
              case 4:
                _SyncJumpWidth = CAN_SJW_4TQ;
                break;
              default:
                // should not happen
                _SyncJumpWidth = CAN_SJW_1TQ;
                break;
            }

            switch (bs1)
            {
              case 1:
                _TimeSeg1 = CAN_BS1_1TQ;
                break;
              case 2:
                _TimeSeg1 = CAN_BS1_2TQ;
                break;
              case 3:
                _TimeSeg1 = CAN_BS1_3TQ;
                break;
              case 4:
                _TimeSeg1 = CAN_BS1_4TQ;
                break;
              case 5:
                _TimeSeg1 = CAN_BS1_5TQ;
                break;
              case 6:
                _TimeSeg1 = CAN_BS1_6TQ;
                break;
              case 7:
                _TimeSeg1 = CAN_BS1_7TQ;
                break;
              case 8:
                _TimeSeg1 = CAN_BS1_8TQ;
                break;
              case 9:
                _TimeSeg1 = CAN_BS1_9TQ;
                break;
              case 10:
                _TimeSeg1 = CAN_BS1_10TQ;
                break;
              case 11:
                _TimeSeg1 = CAN_BS1_11TQ;
                break;
              case 12:
                _TimeSeg1 = CAN_BS1_12TQ;
                break;
              case 13:
                _TimeSeg1 = CAN_BS1_13TQ;
                break;
              case 14:
                _TimeSeg1 = CAN_BS1_14TQ;
                break;
              case 15:
                _TimeSeg1 = CAN_BS1_15TQ;
                break;
              case 16:
                _TimeSeg1 = CAN_BS1_16TQ;
                break;
              default:
                // should not happen
                _TimeSeg1 = CAN_BS1_1TQ;
                break;
            }

            switch (bs2)
            {
              case 1:
                _TimeSeg2 = CAN_BS2_1TQ;
                break;
              case 2:
                _TimeSeg2 = CAN_BS2_2TQ;
                break;
              case 3:
                _TimeSeg2 = CAN_BS2_3TQ;
                break;
              case 4:
                _TimeSeg2 = CAN_BS2_4TQ;
                break;
              case 5:
                _TimeSeg2 = CAN_BS2_5TQ;
                break;
              case 6:
                _TimeSeg2 = CAN_BS2_6TQ;
                break;
              case 7:
                _TimeSeg2 = CAN_BS2_7TQ;
                break;
              case 8:
                _TimeSeg2 = CAN_BS2_8TQ;
                break;
              default:
                // should not happen
                _TimeSeg2 = CAN_BS2_1TQ;
                break;
            }
            _Prescaler = prescaler;

            shouldBrake = true;
          }
          bs1++;
        }
        if (!shouldBrake)
        {
          bs1 = 5;
          bs2++;
        }
      }
      if (!shouldBrake)
      {
        bs1 = 5;
        bs2 = 1;
        prescaler++;
      }
    }
    if (!shouldBrake)
    {
      bs1 = 5;
      sjw++;
    }
  }

  CanHandle->Init.SyncJumpWidth = _SyncJumpWidth;
  CanHandle->Init.TimeSeg1 = _TimeSeg1;
  CanHandle->Init.TimeSeg2 = _TimeSeg2;
  CanHandle->Init.Prescaler = _Prescaler;
}

uint32_t STM32_CAN::getAPB1Clock()
{
  RCC_ClkInitTypeDef clkInit;
  uint32_t flashLatency;
  HAL_RCC_GetClockConfig(&clkInit, &flashLatency);

  uint32_t hclkClock = HAL_RCC_GetHCLKFreq();
  uint8_t clockDivider = 1;
  switch (clkInit.APB1CLKDivider)
  {
  case RCC_HCLK_DIV1:
    clockDivider = 1;
    break;
  case RCC_HCLK_DIV2:
    clockDivider = 2;
    break;
  case RCC_HCLK_DIV4:
    clockDivider = 4;
    break;
  case RCC_HCLK_DIV8:
    clockDivider = 8;
    break;
  case RCC_HCLK_DIV16:
    clockDivider = 16;
    break;
  default:
    // should not happen
    break;
  }

  uint32_t apb1Clock = hclkClock / clockDivider;

  return apb1Clock;
}

void STM32_CAN::enableMBInterrupts()
{
  if (n_pCanHandle->Instance == CAN1) 
  {
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
  }
#ifdef CAN2
  else if (n_pCanHandle->Instance == CAN2) 
  {
    HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);
  }
#endif
#ifdef CAN3
  else if (n_pCanHandle->Instance == CAN3) 
  {
    HAL_NVIC_EnableIRQ(CAN3_TX_IRQn);
  }
#endif
}

void STM32_CAN::disableMBInterrupts()
{
  if (n_pCanHandle->Instance == CAN1) 
  {
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
  }
#ifdef CAN2
  else if (n_pCanHandle->Instance == CAN2)
  {
    HAL_NVIC_DisableIRQ(CAN2_TX_IRQn);
  }
#endif
#ifdef CAN3
  else if (n_pCanHandle->Instance == CAN3)
  {
    HAL_NVIC_DisableIRQ(CAN3_TX_IRQn);
  }
#endif
}

void STM32_CAN::enableLoopBack( bool yes ) {
  if (yes) { n_pCanHandle->Init.Mode = CAN_MODE_LOOPBACK; }
  else { n_pCanHandle->Init.Mode = CAN_MODE_NORMAL; }
}

void STM32_CAN::enableSilentMode( bool yes ) {
  if (yes) { n_pCanHandle->Init.Mode = CAN_MODE_SILENT; }
  else { n_pCanHandle->Init.Mode = CAN_MODE_NORMAL; }
}

void STM32_CAN::enableSilentLoopBack( bool yes ) {
  if (yes) { n_pCanHandle->Init.Mode = CAN_MODE_SILENT_LOOPBACK; }
  else { n_pCanHandle->Init.Mode = CAN_MODE_NORMAL; }
}

void STM32_CAN::enableFIFO(bool status)
{
  //Nothing to do here. The FIFO is on by default. This is just to work with code made for Teensy FlexCan.
}

/* Interupt functions 
-----------------------------------------------------------------------------------------------------------------------------------------------------------------
*/

// There is 3 TX mailboxes. Each one has own transmit complete callback function, that we use to pull next message from TX ringbuffer to be sent out in TX mailbox.
extern "C" void HAL_CAN_TxMailbox0CompleteCallback( CAN_HandleTypeDef *CanHandle )
{
  CAN_message_t txmsg;
  // use correct CAN instance
  if (CanHandle->Instance == CAN1) 
  {
    if (_CAN1->removeFromRingBuffer(_CAN1->txRing, txmsg))
    {
      _CAN1->write(txmsg, true);
    }
  }
#ifdef CAN2
  else if (CanHandle->Instance == CAN2) 
  {
    if (_CAN2->removeFromRingBuffer(_CAN2->txRing, txmsg))
    {
      _CAN2->write(txmsg, true);
    }
  }
#endif
#ifdef CAN3
  else if (CanHandle->Instance == CAN3) 
  {
    if (_CAN3->removeFromRingBuffer(_CAN3->txRing, txmsg))
    {
      _CAN3->write(txmsg, true);
    }
  }
#endif
}

extern "C" void HAL_CAN_TxMailbox1CompleteCallback( CAN_HandleTypeDef *CanHandle )
{
  CAN_message_t txmsg;
  // use correct CAN instance
  if (CanHandle->Instance == CAN1) 
  {
    if (_CAN1->removeFromRingBuffer(_CAN1->txRing, txmsg))
    {
      _CAN1->write(txmsg, true);
    }
  }
#ifdef CAN2
  else if (CanHandle->Instance == CAN2) 
  {
    if (_CAN2->removeFromRingBuffer(_CAN2->txRing, txmsg))
    {
      _CAN2->write(txmsg, true);
    }
  }
#endif
#ifdef CAN3
  else if (CanHandle->Instance == CAN3) 
  {
    if (_CAN3->removeFromRingBuffer(_CAN3->txRing, txmsg))
    {
      _CAN3->write(txmsg, true);
    }
  }
#endif
}

extern "C" void HAL_CAN_TxMailbox2CompleteCallback( CAN_HandleTypeDef *CanHandle )
{
  CAN_message_t txmsg;
  // use correct CAN instance
  if (CanHandle->Instance == CAN1) 
  {
    if (_CAN1->removeFromRingBuffer(_CAN1->txRing, txmsg))
    {
      _CAN1->write(txmsg, true);
    }
  }
#ifdef CAN2
  else if (CanHandle->Instance == CAN2) 
  {
    if (_CAN2->removeFromRingBuffer(_CAN2->txRing, txmsg))
    {
      _CAN2->write(txmsg, true);
    }
  }
#endif
#ifdef CAN3
  else if (CanHandle->Instance == CAN3) 
  {
    if (_CAN3->removeFromRingBuffer(_CAN3->txRing, txmsg))
    {
      _CAN3->write(txmsg, true);
    }
  }
#endif
}

// This is called by RX0_IRQHandler when there is message at RX FIFO0 buffer
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
  CAN_message_t rxmsg;
  CAN_RxHeaderTypeDef   RxHeader;
  
  // move the message from RX FIFO0 to RX ringbuffer
  if (HAL_CAN_GetRxMessage( CanHandle, CAN_RX_FIFO0, &RxHeader, rxmsg.buf ) == HAL_OK)
  {
    if ( RxHeader.IDE == CAN_ID_STD )
    {
      rxmsg.id = RxHeader.StdId;
      rxmsg.flags.extended = 0;
    }
    else
    {
      rxmsg.id = RxHeader.ExtId;
      rxmsg.flags.extended = 1;
    }

    rxmsg.flags.remote = RxHeader.RTR;
    rxmsg.mb           = RxHeader.FilterMatchIndex;
    rxmsg.timestamp    = RxHeader.Timestamp;
    rxmsg.len          = RxHeader.DLC;

    // use correct ring buffer based on CAN instance
    if (CanHandle->Instance == CAN1) 
    {
      rxmsg.bus = 1;
      _CAN1->addToRingBuffer(_CAN1->rxRing, rxmsg);
    }
#ifdef CAN2
    else if (CanHandle->Instance == CAN2) 
    {
      rxmsg.bus = 2;
      _CAN2->addToRingBuffer(_CAN2->rxRing, rxmsg);
    }
#endif
#ifdef CAN3
    else if (CanHandle->Instance == CAN3) 
    {
      rxmsg.bus = 3;
      _CAN3->addToRingBuffer(_CAN3->rxRing, rxmsg);
    }
#endif
  }
}

// RX IRQ handlers
extern "C" void CAN1_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan1);
}

#ifdef CAN2
extern "C" void CAN2_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan2);
}
#endif
#ifdef CAN3
extern "C" void CAN3_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan3);
}
#endif

// TX IRQ handlers
extern "C" void CAN1_TX_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan1);
}

#ifdef CAN2
extern "C" void CAN2_TX_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan2);
}
#endif
#ifdef CAN3
extern "C" void CAN3_TX_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan3);
}
#endif
#endif
