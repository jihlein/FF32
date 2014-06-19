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
// Defines and Variables
///////////////////////////////////////////////////////////////////////////////

#define AGL_SCALE_FACTOR  0.00017241f  // uSecs to meters, 1 cm = 58 uSec

static uint8_t  aglState      = 0;     // 0 = looking for rising edge, 1 = looking for falling edge
static uint16_t aglRiseTime   = 0;     // Timer value at rising edge of pulse
static uint16_t aglPulseWidth = 0;     // Computed pulse width

///////////////////////////////////////

#define RX_PULSE_1p5MS 1500  // 1.5 ms pulse width

static uint16_t pulseWidth[8] = { 0, };

///////////////////////////////////////////////////////////////////////////////
// Interrupt Handler
///////////////////////////////////////////////////////////////////////////////

void TIM1_CC_IRQHandler(void)
{
    ///////////////////////////////////

    // AGL Handler

    uint32_t inputCaptureValue = 0;
    uint16_t tmpccer = 0;

    if (TIM_GetITStatus(TIM1, TIM_IT_CC3) == SET)
    {
        inputCaptureValue = (uint16_t)TIM_GetCapture3(TIM1);

        TIM_ClearITPendingBit(TIM1, TIM_IT_CC3);

        if (aglState == 0)
        {
            // inputCaptureValue has rising edge timer value

            aglRiseTime = inputCaptureValue;

            // Switch states
            aglState = 1;

            // Disable CC Channel 3, Reset the CC3E Bit
            TIM1->CCER &= (uint16_t)~TIM_CCER_CC3E;

            tmpccer  = TIM1->CCER;

            // Select the Polarity and set the CC3E Bit
			tmpccer &= (uint16_t)~(TIM_CCER_CC3P                 | TIM_CCER_CC3NP);
            tmpccer |= (uint16_t) ((TIM_ICPolarity_Falling << 8) | TIM_CCER_CC3E);

            // Write to TIM1 CCER registers
			TIM1->CCER = tmpccer;
        }
        else
        {
            // inputCaptureValue has falling edge timer value

            // Compute capture
            if (inputCaptureValue > aglRiseTime)
                aglPulseWidth = (inputCaptureValue - aglRiseTime);
            else
                aglPulseWidth = ((0xFFFF - aglRiseTime) + inputCaptureValue);

            // Switch state
            aglState = 0;

            // Disable CC Channel 3, Reset the CC3E Bit
            TIM1->CCER &= (uint16_t)~TIM_CCER_CC3E;

            tmpccer = TIM1->CCER;

            // Select the Polarity and set the CC1E Bit
			tmpccer &= (uint16_t)~(TIM_CCER_CC3P                | TIM_CCER_CC3NP);
            tmpccer |= (uint16_t) ((TIM_ICPolarity_Rising << 8) | TIM_CCER_CC3E);

            // Write to TIM1 CCER registers
			TIM1->CCER = tmpccer;
        }
    }

    ///////////////////////////////////

    // PPM RX Handler

    uint16_t diff;

    static uint16_t now;
    static uint16_t last = 0;
    static uint8_t  chan = 0;

    if (TIM_GetITStatus(TIM1, TIM_IT_CC4) == SET)
    {
        last = now;
        now  = TIM_GetCapture1(TIM1);
        rcActive = true;

        TIM_ClearITPendingBit(TIM1, TIM_IT_CC4);

        diff = now - last;

        if (diff > 2700)       // Per http://www.rcgroups.com/forums/showpost.php?p=21996147&postcount=3960
        {                      // "So, if you use 2.5ms or higher as being the reset for the PPM stream start,
            chan = 0;          // you will be fine. I use 2.7ms just to be safe."
        }
        else
        {
            if (diff > 750 && diff < 2250 && chan < 8)    // 750 to 2250 ms is our 'valid' channel range
            {
                pulseWidth[chan] = diff;
            }
            chan++;
        }
    }

    ///////////////////////////////////
}

///////////////////////////////////////////////////////////////////////////////
// Initialization
///////////////////////////////////////////////////////////////////////////////

void agl_ppmRxInit(void)
{
    GPIO_InitTypeDef         GPIO_InitStructure;
    NVIC_InitTypeDef         NVIC_InitStructure;

    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef        TIM_ICInitStructure;

    uint8_t i;

    ///////////////////////////////////

    // AGL TIM1_CH3 PE13

	aglPulseWidth = 0;

	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

    GPIO_Init(GPIOE, &GPIO_InitStructure);

    ///////////////////////////////////

    // PPM TIM1_CH4 PE14

    if (eepromConfig.receiverType == PPM)
    {
		// preset channels to center
	    for (i = 0; i < 8; i++)
	        pulseWidth[i] = RX_PULSE_1p5MS;

        GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);

        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_14;

        GPIO_Init(GPIOE, &GPIO_InitStructure);
    }

    ///////////////////////////////////

    NVIC_InitStructure.NVIC_IRQChannel                   = TIM1_CC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;

    NVIC_Init(&NVIC_InitStructure);

    TIM_TimeBaseStructure.TIM_Prescaler         = 84 - 1;
	TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period            = 0xFFFF;
	TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0000;

    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	///////////////////////////////////

	// AGL

	TIM_ICInitStructure.TIM_Channel     = TIM_Channel_3;
    TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter    = 0x00;

    TIM_ICInit(TIM1, &TIM_ICInitStructure);

    TIM_ITConfig(TIM1, TIM_IT_CC3, ENABLE);

    ///////////////////////////////////

    // PPM

    if (eepromConfig.receiverType == PPM)
    {
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;

        TIM_ICInit(TIM1, &TIM_ICInitStructure);

        TIM_ITConfig(TIM1, TIM_IT_CC4, ENABLE);
    }

    ///////////////////////////////////

    TIM_Cmd(TIM1, ENABLE);

	///////////////////////////////////
}

///////////////////////////////////////////////////////////////////////////////
// AGL Read
///////////////////////////////////////////////////////////////////////////////

float aglRead(void)
{
    return constrain((float)aglPulseWidth * AGL_SCALE_FACTOR, 0.0f, 7.0f);
}

///////////////////////////////////////////////////////////////////////////////
// PPM Receiver Read
///////////////////////////////////////////////////////////////////////////////

uint16_t ppmRxRead(uint8_t channel)
{
    return pulseWidth[channel] << 1;
}

///////////////////////////////////////////////////////////////////////////////
