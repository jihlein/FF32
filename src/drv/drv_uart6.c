/*
  August 2013

  Focused Flight32 Rev -

  Copyright (c) 2013 John Ihlein.  All rights reserved.

  Open Source STM32 Based Multicopter Controller Software

  Designed to run on the AQ32 Flight Control Board

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)Paparazzi UAV
  5)S.O.H. Madgwick
  6)UAVX

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////

/*
    DMA UART routines idea lifted from AutoQuad
    Copyright © 2011  Bill Nesbitt
*/

///////////////////////////////////////////////////////////////////////////////
// UART3 Defines and Variables
///////////////////////////////////////////////////////////////////////////////

#define UART6_TX_PIN        GPIO_Pin_6
#define UART6_RX_PIN        GPIO_Pin_7
#define UART6_GPIO          GPIOC
#define UART6_TX_PINSOURCE  GPIO_PinSource6
#define UART6_RX_PINSOURCE  GPIO_PinSource7

#define UART6_BUFFER_SIZE   2048

// Receive buffer, circular DMA
volatile uint8_t rx6Buffer[UART6_BUFFER_SIZE];
uint32_t rx6DMAPos = 0;

volatile uint8_t  tx6Buffer[UART6_BUFFER_SIZE];
volatile uint16_t tx6BufferTail = 0;
volatile uint16_t tx6BufferHead = 0;

volatile uint8_t  tx6DmaEnabled = false;

///////////////////////////////////////////////////////////////////////////////
// UART6 Transmit via DMA
///////////////////////////////////////////////////////////////////////////////

static void uart6TxDMA(void)
{
	if ((tx6DmaEnabled == true) || (tx6BufferHead == tx6BufferTail))  // Ignore call if already active or no new data in buffer
        return;

    DMA2_Stream6->M0AR = (uint32_t)&tx6Buffer[tx6BufferTail];

    if (tx6BufferHead > tx6BufferTail)
    {
	    DMA_SetCurrDataCounter(DMA2_Stream6, tx6BufferHead - tx6BufferTail);
	    tx6BufferTail = tx6BufferHead;
    }
    else
    {
	    DMA_SetCurrDataCounter(DMA2_Stream6, UART6_BUFFER_SIZE - tx6BufferTail);
	    tx6BufferTail = 0;
    }

    tx6DmaEnabled = true;

    DMA_Cmd(DMA2_Stream6, ENABLE);
}

///////////////////////////////////////////////////////////////////////////////
// UART6 TX Complete Interrupt Handler
///////////////////////////////////////////////////////////////////////////////

void DMA2_Stream6_IRQHandler(void)
{
    DMA_ClearITPendingBit(DMA2_Stream6, DMA_IT_TCIF6);

    tx6DmaEnabled = false;

    uart6TxDMA();
}

///////////////////////////////////////////////////////////////////////////////
// UART6 Initialization
///////////////////////////////////////////////////////////////////////////////

enum { expandEvr = 0 };

void uart6ListenerCB(evr_t e)
{
    if (expandEvr)
        uart6PrintF("EVR-%s %8.3fs %s (%04X)\n", evrToSeverityStr(e.evr), (float)e.time/1000., evrToStr(e.evr), e.reason);
    else
        uart6PrintF("EVR:%08X %04X %04X\n", e.time, e.evr, e.reason);
}

///////////////////////////////////////

void uart6Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    DMA_InitTypeDef   DMA_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    GPIO_PinAFConfig(UART6_GPIO, UART6_TX_PINSOURCE, GPIO_AF_USART6);

    GPIO_InitStructure.GPIO_Pin   = UART6_TX_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    GPIO_Init(UART6_GPIO, &GPIO_InitStructure);

    // DMA TX Interrupt
    NVIC_InitStructure.NVIC_IRQChannel                   = DMA2_Stream6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;

    NVIC_Init(&NVIC_InitStructure);

    USART_InitStructure.USART_BaudRate            = 115200;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_Mode                = USART_Mode_Tx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    USART_Init(USART6, &USART_InitStructure);

    // Transmit DMA
    DMA_DeInit(DMA2_Stream6);

    DMA_InitStructure.DMA_Channel            = DMA_Channel_5;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART6->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)tx6Buffer;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize         = UART6_BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;
    DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;

    DMA_Init(DMA2_Stream6, &DMA_InitStructure);

    DMA_SetCurrDataCounter(DMA2_Stream6, 0);

    DMA_ITConfig(DMA2_Stream6, DMA_IT_TC, ENABLE);

    USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE);

    if (eepromConfig.receiverType != SPEKTRUM)
    {
        ///////////////////////////////

        // Turn off USART6 features used by Spektrum driver

        USART_ITConfig(USART6, USART_IT_RXNE, DISABLE);

        NVIC_InitStructure.NVIC_IRQChannel    = USART6_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
        NVIC_Init(&NVIC_InitStructure);

        ///////////////////////////////

    	GPIO_PinAFConfig(UART6_GPIO, UART6_RX_PINSOURCE, GPIO_AF_USART6);

    	GPIO_InitStructure.GPIO_Pin   = UART6_RX_PIN;
      //GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
      //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
      //GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    	GPIO_Init(UART6_GPIO, &GPIO_InitStructure);

    	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    	USART_Init(USART6, &USART_InitStructure);

    	// Receive DMA into a circular buffer

    	DMA_DeInit(DMA2_Stream1);

      //DMA_InitStructure.DMA_Channel            = DMA_Channel_5;
      //DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART6->DR;
    	DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)rx6Buffer;
    	DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
      //DMA_InitStructure.DMA_BufferSize         = UART6_BUFFER_SIZE;
      //DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
      //DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
      //DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
      //DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    	DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
      //DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;
      //DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
      //DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;
      //DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
      //DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;

    	DMA_Init(DMA2_Stream1, &DMA_InitStructure);

    	DMA_Cmd(DMA2_Stream1, ENABLE);

    	USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);

    	rx6DMAPos = DMA_GetCurrDataCounter(DMA2_Stream1);
    }

    USART_Cmd(USART6, ENABLE);

    evrRegisterListener(uart6ListenerCB);
}

///////////////////////////////////////////////////////////////////////////////
// UART6 Available
///////////////////////////////////////////////////////////////////////////////

uint32_t uart6Available(void)
{
    return (DMA_GetCurrDataCounter(DMA2_Stream1) != rx6DMAPos) ? true : false;
}

///////////////////////////////////////////////////////////////////////////////
// UART6 Clear Buffer
///////////////////////////////////////////////////////////////////////////////

void uart6ClearBuffer(void)
{
    rx6DMAPos = DMA_GetCurrDataCounter(DMA2_Stream1);
}

///////////////////////////////////////////////////////////////////////////////
// UART6 Number of Characters Available
///////////////////////////////////////////////////////////////////////////////

uint16_t uart6NumCharsAvailable(void)
{
	int32_t number;

	number = rx6DMAPos - DMA_GetCurrDataCounter(DMA2_Stream1);

	if (number >= 0)
	    return (uint16_t)number;
	else
	    return (uint16_t)(UART6_BUFFER_SIZE + number);
}

///////////////////////////////////////////////////////////////////////////////
// UART6 Read
///////////////////////////////////////////////////////////////////////////////

uint8_t uart6Read(void)
{
    uint8_t ch;

    ch = rx6Buffer[UART6_BUFFER_SIZE - rx6DMAPos];
    // go back around the buffer
    if (--rx6DMAPos == 0)
	    rx6DMAPos = UART6_BUFFER_SIZE;

    return ch;
}

///////////////////////////////////////////////////////////////////////////////
// UART6 Read Poll
///////////////////////////////////////////////////////////////////////////////

uint8_t uart6ReadPoll(void)
{
    while (!uart6Available()); // wait for some bytes
    return uart6Read();
}

///////////////////////////////////////////////////////////////////////////////
// UART6 Write
///////////////////////////////////////////////////////////////////////////////

void uart6Write(uint8_t ch)
{
    tx6Buffer[tx6BufferHead] = ch;
    tx6BufferHead = (tx6BufferHead + 1) % UART6_BUFFER_SIZE;

    uart6TxDMA();
}

///////////////////////////////////////////////////////////////////////////////
// UART6 Print
///////////////////////////////////////////////////////////////////////////////

void uart6Print(char *str)
{
    while (*str)
    {
    	tx6Buffer[tx6BufferHead] = *str++;
    	tx6BufferHead = (tx6BufferHead + 1) % UART6_BUFFER_SIZE;
    }

	uart6TxDMA();
}

///////////////////////////////////////////////////////////////////////////////
// UART6 Print Formatted - Print formatted string to UART6
// From Ala42
///////////////////////////////////////////////////////////////////////////////

void uart6PrintF(const char * fmt, ...)
{
	char buf[256];

	va_list  vlist;
	va_start (vlist, fmt);

	vsnprintf(buf, sizeof(buf), fmt, vlist);
	uart6Print(buf);
	va_end(vlist);
}

///////////////////////////////////////////////////////////////////////////////
// UART6 Print Binary String
///////////////////////////////////////////////////////////////////////////////

void uart6PrintBinary(uint8_t *buf, uint16_t length)
{
    uint16_t i;

   for (i = 0; i < length; i++)
    {
    	tx6Buffer[tx6BufferHead] = buf[i];
    	tx6BufferHead = (tx6BufferHead + 1) % UART6_BUFFER_SIZE;
    }

	uart6TxDMA();
}

///////////////////////////////////////////////////////////////////////////////
