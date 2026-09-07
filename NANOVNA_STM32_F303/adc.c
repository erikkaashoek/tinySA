/*
 * Copyright (c) 2014-2015, TAKAHASHI Tomohiro (TTRFTECH) edy555@gmail.com
 * All rights reserved.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * The software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */
#include "ch.h"
#include "hal.h"
#include "nanovna.h"

#define ADC_FULL_SCALE  3300
#define F303_ADC_VREF_ALWAYS_ON

#define ADC_CHSELR_VREFINT      ADC_CHANNEL_IN18
#define ADC_CHSELR_VBAT         ADC_CHANNEL_IN17

#define ADC_TOUCH_SMP_TIME           ADC_SMPR_SMP_1P5
#define ADC_TOUCH_XY_SMP_TIME        ADC_SMPR_SMP_601P5
#define ADC_VBAT_SMP_TIME            ADC_SMPR_SMP_601P5

#define ADC_GRP_NUM_CHANNELS_VBAT   2
static adcsample_t samplesVBAT[ADC_GRP_NUM_CHANNELS_VBAT];
static adcsample_t samples[1];

static const ADCConversionGroup adcgrpcfgVBAT = {
  .circular = false,
  .num_channels = ADC_GRP_NUM_CHANNELS_VBAT,
  .end_cb = NULL,
  .error_cb = NULL,
  .cfgr = ADC_CFGR_CONT | ADC_CFGR_RES_12BITS,
  .tr1 = ADC_TR(0, 0),
  .tr2 = ADC_TR_DISABLED,
  .tr3 = ADC_TR_DISABLED,
  .awd2cr = 0U,
  .awd3cr = 0U,
  .smpr = {0, ADC_SMPR2_SMP_AN16(ADC_VBAT_SMP_TIME) |
              ADC_SMPR2_SMP_AN17(ADC_VBAT_SMP_TIME)},
  .sqr = {ADC_SQR1_SQ1_N(ADC_CHANNEL_IN17) |
          ADC_SQR1_SQ2_N(ADC_CHANNEL_IN18), 0, 0, 0}
};

static const ADCConversionGroup adcgrpcfgVersion = {
  .circular = false,
  .num_channels = 1,
  .end_cb = NULL,
  .error_cb = NULL,
  .cfgr = ADC_CFGR_RES_12BITS,
  .tr1 = ADC_TR(0, 0),
  .tr2 = ADC_TR_DISABLED,
  .tr3 = ADC_TR_DISABLED,
  .awd2cr = 0U,
  .awd3cr = 0U,
  .smpr = {ADC_SMPR1_SMP_AN1(ADC_TOUCH_XY_SMP_TIME), 0},
  .sqr = {ADC_SQR1_SQ1_N(ADC_CHANNEL_IN1), 0, 0, 0}
};

static const ADCConversionGroup adcgrpcfgTouch = {
  .circular = true,
  .num_channels = 1,
  .end_cb = NULL,
  .error_cb = NULL,
  .cfgr = ADC_CFGR_EXTEN_0 | ADC_CFGR_EXTSEL_3 | ADC_CFGR_EXTSEL_0 |
          ADC_CFGR_AWD1EN,
  .tr1 = ADC_TR(0, TOUCH_THRESHOLD),
  .tr2 = ADC_TR_DISABLED,
  .tr3 = ADC_TR_DISABLED,
  .awd2cr = 0U,
  .awd3cr = 0U,
  .smpr = {ADC_SMPR1_SMP_AN4(ADC_TOUCH_SMP_TIME), 0},
  .sqr = {ADC_SQR1_SQ1_N(ADC_CHANNEL_IN4), 0, 0, 0}
};

static ADCConversionGroup adcgrpcfgXY = {
  .circular = false,
  .num_channels = 1,
  .end_cb = NULL,
  .error_cb = NULL,
  .cfgr = ADC_CFGR_RES_12BITS,
  .tr1 = ADC_TR(0, 0),
  .tr2 = ADC_TR_DISABLED,
  .tr3 = ADC_TR_DISABLED,
  .awd2cr = 0U,
  .awd3cr = 0U,
  .smpr = {ADC_SMPR1_SMP_AN3(ADC_TOUCH_XY_SMP_TIME) |
           ADC_SMPR1_SMP_AN4(ADC_TOUCH_XY_SMP_TIME), 0},
  .sqr = {ADC_SQR1_SQ1_N(ADC_CHANNEL_IN3), 0, 0, 0}
};

void adc_init(void)
{
  adcStart(&ADCD2, NULL);
  adcStart(&ADCD1, NULL);
  #ifdef F303_ADC_VREF_ALWAYS_ON
  adcSTM32EnableVBAT(&ADCD1);
  adcSTM32EnableVREF(&ADCD1);
//  adcSTM32EnableTS(&ADCD1);
  #endif
}

uint16_t adc_single_read(uint32_t chsel)
{
  /* ADC setup */
//  adcStart(&ADCD2, NULL);
  adcgrpcfgXY.sqr[0] = ADC_SQR1_SQ1_N(chsel);
  adcConvert(&ADCD2, &adcgrpcfgXY, samples, 1);
  return(samples[0]);
}

uint16_t adc1_single_read(uint32_t chsel)
{
  (void)chsel;
  /* ADC setup */
  adcConvert(&ADCD1, &adcgrpcfgVersion, samples, 1);
  return(samples[0]);
}


int16_t adc_vbat_read(void)
{
// Vbat measure averange count = 2^VBAT_AVERAGE
#define VBAT_AVERAGE 4
// Measure vbat every 5 second
#define VBAT_MEASURE_INTERVAL   TIME_S2I(5)

  static int16_t   vbat_raw = 0;
  static systime_t vbat_time = -VBAT_MEASURE_INTERVAL-1;

  systime_t _time = chVTGetSystemTimeX();
  if (_time - vbat_time < VBAT_MEASURE_INTERVAL)
    goto return_cached;
  vbat_time = _time;
  uint16_t VREFINT_CAL = (*((uint16_t*)0x1FFFF7BA));
  uint32_t vrefint = 0;
  uint32_t vbat = 0;
//  const uint16_t V25 = 1750;// when V25=1.41V at ref 3.3V
//  const uint16_t Avg_Slope = 5; //when avg_slope=4.3mV/C at ref 3.3V
//  uint16_t temperature_cal1 = *((uint16_t*) ((uint32_t)0x1FFFF7B8U));
//                            /* Internal temperature sensor, address of parameter TS_CAL1: On STM32F3,
//                               temperature sensor ADC raw data acquired at temperature  25 DegC (tolerance: +-5 DegC),
//                               Vref+ = 3.3 V (tolerance: +-10 mV). */
//  uint16_t temperature_cal2 = *((uint16_t*) ((uint32_t)0x1FFFF7C2U));
//                            /* Internal temperature sensor, address of parameter TS_CAL2: On STM32F3,
//                               temperature sensor ADC raw data acquired at temperature 110 DegC (tolerance: +-5 DegC),
//                               Vref+ = 3.3 V (tolerance: +-10 mV). */
//  float avg_slope = ((float)(temperature_cal1 - temperature_cal2))/(110-25);
//  float ts;
#ifndef F303_ADC_VREF_ALWAYS_ON
  adcSTM32EnableVBAT(&ADCD1);
  adcSTM32EnableVREF(&ADCD1);
//  adcSTM32EnableTS(&ADCD1);
#endif
  for (uint16_t i = 0; i < 1<<VBAT_AVERAGE; i++){
    adcConvert(&ADCD1, &adcgrpcfgVBAT, samplesVBAT, 1);
    vbat+= samplesVBAT[0];
    vrefint+= samplesVBAT[1];
  }
  vbat>>=VBAT_AVERAGE;
  vrefint>>=VBAT_AVERAGE;
#ifndef F303_ADC_VREF_ALWAYS_ON
  adcSTM32DisableVBAT(&ADCD1);
  adcSTM32DisableVREF(&ADCD1);
//  adcSTM32DisableTS(&ADCD1);
#endif

//  ts = samplesVBAT[2];
//  uint16_t vts = (ADC_FULL_SCALE * VREFINT_CAL * ts / (vrefint * ((1<<12)-1)));
//  uint16_t TemperatureC2 = (uint16_t)((V25-ts)/Avg_Slope+25);
//  uint16_t TemperatureC = (uint16_t)((V25-ts)/avg_slope+25);

  // vbat_raw = (3000 * 2 * vbat / 4095) * (VREFINT_CAL / vrefint)
  // uint16_t vbat_raw = (ADC_FULL_SCALE * VREFINT_CAL * (float)vbat * 2 / (vrefint * ((1<<12)-1)));
  // For speed divide not on 4095, divide on 4096, get little error, but no matter
  vbat_raw = ((ADC_FULL_SCALE * 2 * vbat)>>12) * VREFINT_CAL / vrefint;
return_cached:
  if (vbat_raw < 100) {
    // maybe D2 is not installed
    return -1;
  }
  return vbat_raw + config.vbat_offset;
}

void adc_start_analog_watchdog(void)
{
  adcStartConversion(&ADCD2, &adcgrpcfgTouch, samples, 1);
}

void adc_stop_analog_watchdog(void)
{
  adcStopConversion(&ADCD2);
}

static inline void adc_interrupt(void)
{
  uint32_t isr = ADC2->ISR;
  ADC2->ISR = isr;
  if (isr & ADC_ISR_OVR) {
//    ADC overflow condition, this could happen only if the DMA is unable to read data fast enough.
  }
  if (isr & ADC_ISR_AWD1) {
    /* Analog watchdog error.*/
    handle_touch_interrupt();
  }
}

OSAL_IRQ_HANDLER(STM32_ADC2_HANDLER)
{
  OSAL_IRQ_PROLOGUE();

  adc_interrupt();

  OSAL_IRQ_EPILOGUE();
}

#if 0
uint16_t adc_multi_read(uint32_t chsel, uint16_t *result, uint32_t count)
{
  /* ADC setup */
  VNA_ADC->ISR    = VNA_ADC->ISR;
  VNA_ADC->IER    = 0;
  VNA_ADC->TR     = ADC_TR(0, 0);
  VNA_ADC->SMPR   = ADC_SMPR_SMP_1P5;
  VNA_ADC->CFGR  = ADC_CFGR_RES_12BITS;
  VNA_ADC->CHSELR = chsel;


//  palSetPadMode(GPIOA, 10, PAL_MODE_OUTPUT_PUSHPULL);

  do{
#if 0
    if (count < 145)
      palSetPad(GPIOA, 10);
    else
      palClearPad(GPIOA, 10);
#endif
    VNA_ADC->CR |= ADC_CR_ADSTART; // ADC conversion start.
//    while (VNA_ADC->CR & ADC_CR_ADSTART)
    while(!(VNA_ADC->ISR & ADC_ISR_EOC));
      ;
    *(result++) =VNA_ADC->DR;
  }while(--count);
  return count;
}

int16_t adc_buf_read(uint32_t chsel, uint16_t *result, uint32_t count)
{

  adc_stop();

#if 0
  // drive high to low on Y line (coordinates from left to right)
  palSetPad(GPIOB, GPIOB_YN);
  palClearPad(GPIOA, GPIOA_YP);
  // Set Y line as output
  palSetPadMode(GPIOB, GPIOB_YN, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOA, GPIOA_YP, PAL_MODE_OUTPUT_PUSHPULL);
  // Set X line as input
  palSetPadMode(GPIOB, GPIOB_XN, PAL_MODE_INPUT);        // Hi-z mode
  palSetPadMode(GPIOA, GPIOA_XP, PAL_MODE_INPUT_ANALOG); // <- ADC_TOUCH_X channel
    uint16_t res = adc_multi_read(ADC_TOUCH_X, result, count);
#else
//  palSetPadMode(GPIOA, 9, PAL_MODE_INPUT_ANALOG);
  uint16_t res = adc_multi_read(chsel, result, count); // ADC_CHSELR_CHSEL9
#endif
  touch_start_watchdog();
  return res;
}

#endif
