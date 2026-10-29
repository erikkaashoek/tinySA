/* Copyright (c) 2020, Erik Kaashoek erik@kaashoek.com
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
#include <math.h>
#include "si4432.h"

#define _FAST_SHIFT_

#pragma GCC push_options
#pragma GCC optimize ("O2")

#define SPI2_CLK_HIGH   palSetPad(GPIOB, GPIO_SPI2_CLK)
#define SPI2_CLK_LOW    palClearPad(GPIOB, GPIO_SPI2_CLK)

#define SPI2_SDI_HIGH   palSetPad(GPIOB, GPIO_SPI2_SDI)
#define SPI2_SDI_LOW    palClearPad(GPIOB, GPIO_SPI2_SDI)
#define SPI2_RESET      palClearPort(GPIOB, (1<<GPIO_SPI2_CLK)|(1<<GPIO_SPI2_SDI))

#define SPI2_SDO       ((palReadPort(GPIOB)>>GPIO_SPI2_SDO)&1)
#define SPI2_portSDO   (palReadPort(GPIOB)&(1<<GPIO_SPI2_SDO))

#define CS_PE_HIGH      palSetPad(GPIOC, GPIO_PE_SEL)
#define CS_PE_LOW      palClearPad(GPIOC, GPIO_PE_SEL)


//#define MAXLOG 1024
//unsigned char SI4432_logging[MAXLOG];
//volatile int log_index = 0;

//#define SI4432_log(X)   { if (log_index < MAXLOG)  SI4432_logging[log_index++] = X; }
#define SI4432_log(X)

static void shiftOut(uint8_t val)
{
//  SI4432_log(SI4432_Sel);
//  SI4432_log(val);
#ifndef _FAST_SHIFT_
  uint16_t i = 8;
  do {
    if (val & 0x80)
      SPI2_SDI_HIGH;
    SPI2_CLK_HIGH;
    SPI2_RESET;
    val<<=1;
  }while(--i);
#else
  if (val & 0x80) SPI2_SDI_HIGH;
  SPI2_CLK_HIGH;
  SPI2_RESET;
  if (val & 0x40) SPI2_SDI_HIGH;
  SPI2_CLK_HIGH;
  SPI2_RESET;
  if (val & 0x20) SPI2_SDI_HIGH;
  SPI2_CLK_HIGH;
  SPI2_RESET;
  if (val & 0x10) SPI2_SDI_HIGH;
  SPI2_CLK_HIGH;
  SPI2_RESET;
  if (val & 0x08) SPI2_SDI_HIGH;
  SPI2_CLK_HIGH;
  SPI2_RESET;
  if (val & 0x04) SPI2_SDI_HIGH;
  SPI2_CLK_HIGH;
  SPI2_RESET;
  if (val & 0x02) SPI2_SDI_HIGH;
  SPI2_CLK_HIGH;
  SPI2_RESET;
  if (val & 0x01) SPI2_SDI_HIGH;
  SPI2_CLK_HIGH;
  SPI2_RESET;
#endif
}

static uint8_t shiftIn(void)
{
  uint32_t value = 0;
#ifndef _FAST_SHIFT_
  uint16_t i = 8;
  do {
    value<<=1;
    SPI2_CLK_HIGH;
    value|=SPI2_portSDO;
    SPI2_CLK_LOW;
  }while(--i);
#else
  SPI2_CLK_HIGH;
  value =SPI2_portSDO;
  SPI2_CLK_LOW;
  value<<=1;
  SPI2_CLK_HIGH;
  value|=SPI2_portSDO;
  SPI2_CLK_LOW;
  value<<=1;
  SPI2_CLK_HIGH;
  value|=SPI2_portSDO;
  SPI2_CLK_LOW;
  value<<=1;
  SPI2_CLK_HIGH;
  value|=SPI2_portSDO;
  SPI2_CLK_LOW;
  value<<=1;
  SPI2_CLK_HIGH;
  value|=SPI2_portSDO;
  SPI2_CLK_LOW;
  value<<=1;
  SPI2_CLK_HIGH;
  value|=SPI2_portSDO;
  SPI2_CLK_LOW;
  value<<=1;
  SPI2_CLK_HIGH;
  value|=SPI2_portSDO;
  SPI2_CLK_LOW;
  value<<=1;
  SPI2_CLK_HIGH;
  value|=SPI2_portSDO;
  SPI2_CLK_LOW;
#endif
  return value>>GPIO_SPI2_SDO;
}

static inline void shiftInBuf(uint16_t sel, uint8_t addr, deviceRSSI_t *buf, uint16_t size, uint16_t delay) {
  int idx = 0;
  do{
   palClearPad(GPIOC, sel);
    uint32_t value = addr;
    uint16_t i = 8;
    do {
      if (value & 0x80) SPI2_SDI_HIGH;
      SPI2_CLK_HIGH;
      SPI2_RESET;
      value<<=1;
    }while(--i);
    value = 0;
    i = 8;
    do {
      SPI2_CLK_HIGH;
      value<<=1;
      value|=SPI2_portSDO;
      SPI2_CLK_LOW;
    }while(--i);
    buf[idx]=value>>GPIO_SPI2_SDO;
    palSetPad(GPIOC, sel);
    if (++idx >=size) return;
    if (delay)
      my_microsecond_delay(delay);
  }while(1);
}

#ifdef __SI4432__
#define CS_SI0_HIGH     palSetPad(GPIOC, GPIO_RX_SEL)
#define CS_SI1_HIGH     palSetPad(GPIOC, GPIO_LO_SEL)

#define RF_POWER_HIGH   palSetPad(GPIOB, GPIO_RF_PWR)


#define CS_SI0_LOW     palClearPad(GPIOC, GPIO_RX_SEL)
#define CS_SI1_LOW     palClearPad(GPIOC, GPIO_LO_SEL)


const uint16_t SI_nSEL[MAX_SI4432+1] = { GPIO_RX_SEL, GPIO_LO_SEL, 0}; // #3 is dummy!!!!!!

uint16_t SI4432_Sel = GPIO_RX_SEL;         // currently selected SI4432
uint8_t SI4432_rbw_selected = 0;

// volatile int SI4432_guard = 0;

#ifdef __SI4432_H__
#define SELECT_DELAY 10

void SI4432_shiftOutDword(uint32_t buf, uint16_t size) {
  uint16_t port = SI_nSEL[SI4432_Sel];
  palClearPad(GPIOC, port);
  do{
    shiftOut(buf);
    buf>>=8;
  }while(--size);
  palSetPad(GPIOC, port);
}

#ifndef SI4432_Write_Byte
void SI4432_Write_Byte(uint8_t ADR, uint8_t DATA )
{
  uint16_t port = SI_nSEL[SI4432_Sel];
//  if (SI4432_guard)
//    while(1) ;
//  SI4432_guard = 1;
//  SPI2_CLK_LOW;
  palClearPad(GPIOC, port);
//  chThdSleepMicroseconds(SELECT_DELAY);
  ADR |= 0x80 ; // RW = 1
  shiftOut( ADR );
  shiftOut( DATA );
  palSetPad(GPIOC, port);
//  SI4432_guard = 0;
}

void SI4432_Write_2_Byte(uint8_t ADR, uint8_t DATA1, uint8_t DATA2)
{
  uint16_t port = SI_nSEL[SI4432_Sel];
//  if (SI4432_guard)
//    while(1) ;
//  SI4432_guard = 1;
//  SPI2_CLK_LOW;
  palClearPad(GPIOC, port);
//  chThdSleepMicroseconds(SELECT_DELAY);
  ADR |= 0x80 ; // RW = 1
  shiftOut( ADR );
  shiftOut( DATA1 );
  shiftOut( DATA2 );
  palSetPad(GPIOC, port);
//  SI4432_guard = 0;
}
#endif

#ifndef SI4432_Write_3_Byte
void SI4432_Write_3_Byte(uint8_t ADR, uint8_t DATA1, uint8_t DATA2, uint8_t DATA3)
{
  uint16_t port = SI_nSEL[SI4432_Sel];
//  if (SI4432_guard)
//    while(1) ;
//  SI4432_guard = 1;
//  SPI2_CLK_LOW;
  palClearPad(GPIOC, port);
//  chThdSleepMicroseconds(SELECT_DELAY);
  ADR |= 0x80 ; // RW = 1
  shiftOut( ADR );
  shiftOut( DATA1 );
  shiftOut( DATA2 );
  shiftOut( DATA3 );
  palSetPad(GPIOC, port);
//  SI4432_guard = 0;
}
#endif

uint8_t SI4432_Read_Byte(uint8_t ADR)
{
  uint8_t DATA ;
  uint16_t port = SI_nSEL[SI4432_Sel];
//  if (SI4432_guard)
//    while(1) ;
//  SI4432_guard = 1;
//  SPI2_CLK_LOW;
  palClearPad(GPIOC, port);
  shiftOut(ADR);
  DATA = shiftIn();
  palSetPad(GPIOC, port);
//  SI4432_guard = 0;
  return DATA ;
}

void SI4432_Reset(void)
{
  int count = 0;
  SI4432_Read_Byte (SI4432_INT_STATUS1);    // Clear pending interrupts
  SI4432_Read_Byte (SI4432_INT_STATUS2);
  // always perform a system reset (don't send 0x87)
  SI4432_Write_Byte(SI4432_STATE, 0x80);
  chThdSleepMilliseconds(10);
  // wait for chiprdy bit
  while (count++ < 100 && ( SI4432_Read_Byte (SI4432_INT_STATUS2) & 0x02 ) == 0) {
    chThdSleepMilliseconds(10);
  }
}

void SI4432_Drive(int d)
{
  SI4432_Write_Byte(SI4432_TX_POWER, (uint8_t) (0x18+(d & 7)));
}

void SI4432_Transmit(int d)
{
  int count = 0;
  SI4432_Drive(d);
  if (( SI4432_Read_Byte(SI4432_DEV_STATUS) & 0x03 ) == 2)
    return; // Already in transmit mode
  chThdSleepMilliseconds(3);
  SI4432_Write_Byte(SI4432_STATE, 0x02);
  chThdSleepMilliseconds(3);
  SI4432_Write_Byte(SI4432_STATE, 0x0b);
  chThdSleepMilliseconds(10);
  while (count++ < 100 && ( SI4432_Read_Byte(SI4432_DEV_STATUS) & 0x03 ) != 2) {
    chThdSleepMilliseconds(10);
  }
}

void SI4432_Receive(void)
{
  int count = 0;
  if (( SI4432_Read_Byte (SI4432_DEV_STATUS) & 0x03 ) == 1)
    return; // Already in receive mode
  chThdSleepMilliseconds(3);
  SI4432_Write_Byte(SI4432_STATE, 0x02);
  chThdSleepMilliseconds(3);
  SI4432_Write_Byte(SI4432_STATE, 0x07);
  chThdSleepMilliseconds(10);
  while (count++ < 100 && ( SI4432_Read_Byte(SI4432_DEV_STATUS) & 0x03 ) != 1) {
    chThdSleepMilliseconds(5);
  }
}

// User asks for an RBW of WISH, go through table finding the last triple
// for which WISH is greater than the first entry, use those values,
// Return the first entry of the following triple for the RBW actually achieved
#define IF_BW(dwn3, ndec, filset) (((dwn3)<<7)|((ndec)<<4)|(filset))
typedef struct {
  uint8_t  reg;                   // IF_BW(dwn3, ndec, filset)
  int8_t   RSSI_correction_x_10;  // Correction * 10
  uint16_t RBWx10;                // RBW * 10 in kHz
}RBW_t; // sizeof(RBW_t) = 4 bytes
static const RBW_t RBW_choices[] =
{
// BW register    corr  freq
//                              {IF_BW(0,5,1),0,26},
//                              {IF_BW(0,5,2),0,28},
#if 0
                              {IF_BW(0,5,3),3,31},
                              {IF_BW(0,5,4),-3,32},
                              {IF_BW(0,5,5),6,37},
                              {IF_BW(0,5,6),5,42},
                              {IF_BW(0,5,7),5,45},
                              {IF_BW(0,4,1),0,49},
                              {IF_BW(0,4,2),0,54},
                              {IF_BW(0,4,3),0,59},
                              {IF_BW(0,4,4),0,61},
                              {IF_BW(0,4,5),5,72},
                              {IF_BW(0,4,6),5,82},
                              {IF_BW(0,4,7),5,88},
                              {IF_BW(0,3,1),0,95},
                              {IF_BW(0,3,2),0,106},
                              {IF_BW(0,3,3),2,115},
                              {IF_BW(0,3,4),0,121},
                              {IF_BW(0,3,5),5,142},
                              {IF_BW(0,3,6),5,162},
                              {IF_BW(0,3,7),5,175},
                              {IF_BW(0,2,1),0,189},
                              {IF_BW(0,2,2),0,210},
                              {IF_BW(0,2,3),3,227},
                              {IF_BW(0,2,4),0,240},
                              {IF_BW(0,2,5),5,282},
                              {IF_BW(0,2,6),5,322},
                              {IF_BW(0,2,7),5,347},
                              {IF_BW(0,1,1),0,377},
                              {IF_BW(0,1,2),0,417},
                              {IF_BW(0,1,3),1,452},
                              {IF_BW(0,1,4),0,479},
                              {IF_BW(0,1,5),5,562},
                              {IF_BW(0,1,6),5,641},
                              {IF_BW(0,1,7),5,692},
                              {IF_BW(0,0,1),0,752},
                              {IF_BW(0,0,2),0,832},
                              {IF_BW(0,0,3),0,900},
                              {IF_BW(0,0,4),-1,953},
                              {IF_BW(0,0,5),9,1121},
                              {IF_BW(0,0,6),2,1279},
                              {IF_BW(0,0,7),5,1379},
                              {IF_BW(1,1,4),20,1428},
                              {IF_BW(1,1,5),26,1678},
                              {IF_BW(1,1,9),-50,1811},
                              {IF_BW(1,0,15),-100,1915},
                              {IF_BW(1,0,1),20,2251},
                              {IF_BW(1,0,2),22,2488},
                              {IF_BW(1,0,3),21,2693},
                              {IF_BW(1,0,4),15,2849},
                              {IF_BW(1,0,8),-15,3355},
                              {IF_BW(1,0,9),-53,3618},
                              {IF_BW(1,0,10),-15,4202},
                              {IF_BW(1,0,11),-13,4684},
                              {IF_BW(1,0,12),-20,5188},
                              {IF_BW(1,0,13),-14,5770},
                              {IF_BW(1,0,14),-9,6207},
#else
                              {IF_BW(0,5,3),1,31},
                              {IF_BW(0,5,4),-4,32},
                              {IF_BW(0,5,5),1,37},
                              {IF_BW(0,5,6),-3,42},
                              {IF_BW(0,5,7),-3,45},
                              {IF_BW(0,4,1),-4,49},
                              {IF_BW(0,4,2),-4,54},
                              {IF_BW(0,4,3),-3,59},
                              {IF_BW(0,4,4),-4,61},
                              {IF_BW(0,4,5),1,72},
                              {IF_BW(0,4,6),1,82},
                              {IF_BW(0,4,7),1,88},
                              {IF_BW(0,3,1),-4,95},
                              {IF_BW(0,3,2),-4,106},
                              {IF_BW(0,3,3),-3,115},
                              {IF_BW(0,3,4),-8,121},
                              {IF_BW(0,3,5),1,142},
                              {IF_BW(0,3,6),1,162},
                              {IF_BW(0,3,7),1,175},
                              {IF_BW(0,2,1),-4,189},
                              {IF_BW(0,2,2),-4,210},
                              {IF_BW(0,2,3),1,227},
                              {IF_BW(0,2,4),-4,240},
                              {IF_BW(0,2,5),1,282},
                              {IF_BW(0,2,6),1,322},
                              {IF_BW(0,2,7),1,347},
                              {IF_BW(0,1,1),-4,377},
                              {IF_BW(0,1,2),-4,417},
                              {IF_BW(0,1,3),-3,452},
                              {IF_BW(0,1,4),-4,479},
                              {IF_BW(0,1,5),1,562},
                              {IF_BW(0,1,6),1,641},
                              {IF_BW(0,1,7),1,692},
                              {IF_BW(0,0,1),-4,752},
                              {IF_BW(0,0,2),-4,832},
                              {IF_BW(0,0,3),-3,900},
                              {IF_BW(0,0,4),-4,953},
                              {IF_BW(0,0,5),6,1121},
                              {IF_BW(0,0,6),1,1279},
                              {IF_BW(0,0,7),1,1379},
                              {IF_BW(1,1,4),16,1428},
                              {IF_BW(1,1,5),22,1678},
                              {IF_BW(1,1,9),-53,1811},
                              {IF_BW(1,0,15),-104,1915},
                              {IF_BW(1,0,1),16,2251},
                              {IF_BW(1,0,2),21,2488},
                              {IF_BW(1,0,3),17,2693},
                              {IF_BW(1,0,4),11,2849},
                              {IF_BW(1,0,8),-19,3355},
                              {IF_BW(1,0,9),-54,3618},
                              {IF_BW(1,0,10),-14,4202},
                              {IF_BW(1,0,11),-14,4684},
                              {IF_BW(1,0,12),-23,5188},
                              {IF_BW(1,0,13),-14,5770},
                              {IF_BW(1,0,14),-9,6207},

#endif

};

const uint8_t SI4432_RBW_count = ARRAY_COUNT(RBW_choices);

static pureRSSI_t SI4432_RSSI_correction = float_TO_PURE_RSSI(-120);

uint16_t force_rbw(int i)
{
  SI4432_Write_Byte(SI4432_IF_FILTER_BW, RBW_choices[i].reg);                     // Write RBW settings to Si4432
  SI4432_RSSI_correction = float_TO_PURE_RSSI(RBW_choices[i].RSSI_correction_x_10 - 1200)/10;  // Set RSSI correction
//  SI4432_RSSI_correction = float_TO_PURE_RSSI( - 1200)/10;  // Set RSSI correction
  SI4432_rbw_selected = i;
  return RBW_choices[i].RBWx10;                                                   // RBW achieved by Si4432 in kHz * 10
}

uint16_t set_rbw(uint16_t WISH)  {
  uint16_t i;
  for (i=0; i <ARRAY_COUNT(RBW_choices) - 1; i++)
    if (WISH <= RBW_choices[i].RBWx10) 
      break; 
  return force_rbw(i);
}


uint8_t SI4432_frequency_changed = false;
uint8_t SI4432_offset_changed = false;

// #define __CACHE_BAND__  // Is not reliable!!!!!!

#ifdef __CACHE_BAND__
static uint8_t old_freq_band[2] = {-1,-1};
static uint8_t written[2]= {0,0};
#endif

void SI4432_Set_Frequency ( freq_t Freq ) {
//  int mode = SI4432_Read_Byte(0x02) & 0x03;           // Disabled as unreliable
//  SI4432_Write_Byte(0x07, 0x02);    // Switch to tune mode

//  Freq = (Freq / 1000 ) * 1000; // force freq to 1000 grid

  uint8_t hbsel;
  if (0) shell_printf("%d: Freq %q\r\n", SI4432_Sel, Freq);
  if (Freq >= 480000000U) {
    hbsel = 1<<5;
    Freq>>=1;
  } else {
    hbsel = 0;
  }
  uint8_t sbsel = 1 << 6;
  uint32_t N = (Freq / config.setting_frequency_10mhz - 24)&0x1F;
  uint32_t K = Freq % config.setting_frequency_10mhz;
  if (N>=24) {
    N=23;
    K = 9999999;
  }
  uint32_t Carrier = (K<<2) / 625;
  uint8_t Freq_Band = N | hbsel | sbsel;
//  int count = 0;
//  my_microsecond_delay(200);
//  int s;
//  while (count++ < 100 && ( (s = SI4432_Read_Byte ( 0x02 )) & 0x03 ) != 0) {
//    my_microsecond_delay(100);
//  }

#ifdef __CACHE_BAND__
  if (old_freq_band[SI4432_Sel] == Freq_Band) {
    if (written[SI4432_Sel] < 4) {
      SI4432_Write_Byte(SI4432_FREQBAND, Freq_Band );
      written[SI4432_Sel]++;
    }
    SI4432_Write_Byte(SI4432_FREQCARRIER_H, (Carrier>>8) & 0xFF );
    SI4432_Write_Byte(SI4432_FREQCARRIER_L, Carrier & 0xFF  );
  } else {
#endif
#if 0       // Do not use multi byte write
    SI4432_Write_Byte(SI4432_FREQBAND, Freq_Band );                          // Freq band must be written first !!!!!!!!!!!!
    SI4432_Write_Byte(SI4432_FREQCARRIER_H, (Carrier>>8) & 0xFF );
    SI4432_Write_Byte(SI4432_FREQCARRIER_L, Carrier & 0xFF  );
#else
    SI4432_Write_3_Byte (SI4432_FREQBAND, Freq_Band, (Carrier>>8) & 0xFF, Carrier & 0xFF);
#endif
#ifdef __CACHE_BAND__
    old_freq_band[SI4432_Sel] = Freq_Band;
    written[SI4432_Sel] = 0;
  }
#endif
  SI4432_frequency_changed = true;
//  if (mode == 1)        // RX mode            Disabled as unreliable
//    SI4432_Write_Byte( 0x07, 0x07);
//  else
//    SI4432_Write_Byte( 0x07, 0x0B);
}

uint32_t SI4432_step_delay = 1500;
//extern int setting.repeat;

#ifdef __FAST_SWEEP__
extern deviceRSSI_t age[POINTS_COUNT];
static uint16_t buf_index = 0;
static uint16_t buf_end = 0;
static bool  buf_read = false;

#if 0
int SI4432_is_fast_mode(void)
{
  return buf_read;
}
#endif

#ifdef __ULTRA__
void enable_ultra(int s)
{
static int old_ultra = -1;
  if (s != old_ultra) {
#ifdef TINYSA4
    if (s)
      palClearLine(LINE_ULTRA);
    else
      palSetLine(LINE_ULTRA);
#endif
    old_ultra = s;
  }
}
#endif

//--------------------------- Trigger -------------------
// ************** trigger mode if need
#if 0
// trigger on measure 4 point
#define T_POINTS            4
#define T_LEVEL_UNDEF       (1<<(16-T_POINTS)) // should drop after 4 shifts left
#define T_LEVEL_BELOW       1
#define T_LEVEL_ABOVE       0
// Trigger mask, should have width T_POINTS bit
#define T_DOWN_MASK         (0b0011)           // 2 from up 2 to bottom
#define T_UP_MASK           (0b1100)           // 2 from bottom 2 to up
#define T_LEVEL_CLEAN       ~(1<<T_POINTS)     // cleanup old trigger data
#else
    // trigger on measure 2 point
#define T_POINTS            2
#define T_LEVEL_UNDEF       (1<<(16-T_POINTS)) // should drop after 2 shifts left
#define T_LEVEL_BELOW       1
#define T_LEVEL_ABOVE       0
    // Trigger mask, should have width T_POINTS bit
#define T_DOWN_MASK         (0b0001)           // 1 from up 1 to bottom
#define T_UP_MASK           (0b0010)           // 1 from bottom 1 to up
#define T_LEVEL_CLEAN       ~(1<<T_POINTS)     // cleanup old trigger data
#endif

enum { ST_ARMING, ST_WAITING, ST_FILLING };

void SI4432_trigger_fill(int s, uint8_t trigger_lvl, int up_direction, int trigger_mode)
{
  SI4432_Sel = s;
  uint8_t rssi;
  uint16_t sel = SI_nSEL[SI4432_Sel];
  uint32_t t = setting.additional_step_delay_us;
  systime_t measure = chVTGetSystemTimeX();
  int waiting = ST_ARMING;
//  __disable_irq();
  int i = 0;
  uint16_t t_mode = up_direction ? T_UP_MASK : T_DOWN_MASK;
  uint16_t data_level = T_LEVEL_UNDEF;
  do {
    if (operation_requested)                        // allow aborting a wait for trigger
      return;                                       // abort
    palClearPad(GPIOC, sel);
    shiftOut(SI4432_REG_RSSI);
    age[i++] = rssi = shiftIn();
    palSetPad(GPIOC, sel);
    if (i >= sweep_points)
      i = 0;
    switch (waiting) {
    case ST_ARMING:
      if (i == sweep_points-1) {
        waiting = ST_WAITING;
        setting.measure_sweep_time_us = sa_ST2US(chVTGetSystemTimeX() - measure);
      }
      break;
    case ST_WAITING:
      // Store data level bitfield (remember only last 2 states)
      // T_LEVEL_UNDEF mode bit drop after 2 shifts
#if 0
      if (rssi < trigger_lvl) {
        data_level = ((data_level<<1) | (T_LEVEL_BELOW))&(T_LEVEL_CLEAN);
      } else {
        data_level = ((data_level<<1) | (T_LEVEL_ABOVE))&(T_LEVEL_CLEAN);
      }
#else
      data_level = ((data_level<<1) | (rssi < trigger_lvl ? T_LEVEL_BELOW : T_LEVEL_ABOVE))&(T_LEVEL_CLEAN);
#endif
      if (data_level == t_mode) {  // wait trigger
 //     if (i == 128) {  // wait trigger
        waiting = ST_FILLING;
        switch (trigger_mode) {
        case T_PRE:                // Trigger at the begin of the scan
          buf_index = i;
          goto fill_rest;
          break;
        case T_POST:               // Trigger at the end of the scan
          buf_index = i;
          goto done;
          break;
        case T_MID:                // Trigger in the middle of the scan
          buf_index = i + sweep_points/2;
          if (buf_index >= sweep_points)
            buf_index -= sweep_points;
          break;
        }
      }
      break;
    case ST_FILLING:
      if (i == buf_index)
        goto done;
    }
fill_rest:
    if (t)
      my_microsecond_delay(t);
  }while(1);
done:
  buf_end = buf_index;
  buf_read = true;
}

void SI4432_Fill(int s, int start)
{
  SI4432_Sel = s;
  uint16_t sel = SI_nSEL[SI4432_Sel];
#if 0
  uint32_t t = calc_min_sweep_time_us(); // Time to delay in uS for all sweep
  if (t < setting.sweep_time_us){
    t = setting.sweep_time_us - t;
    t = t / (sweep_points - 1);          // Now in uS per point
  }
  else
    t = 0;
#endif
  uint32_t t = setting.additional_step_delay_us;
  systime_t measure = chVTGetSystemTimeX();
//  __disable_irq();
#if 1
  int stop = sweep_points - start;
  int i = 0;
  uint8_t *buf = &age[start];
  do {
    palClearPad(GPIOC, sel);
    shiftOut(SI4432_REG_RSSI);
    buf[i]=shiftIn();
    palSetPad(GPIOC, sel);
    if (++i >=stop) break;
    if (t)
      my_microsecond_delay(t);
  } while(1);
#else
  shiftInBuf(sel, SI4432_REG_RSSI, &age[start], sweep_points - start, t);
#endif
//  __enable_irq();
  setting.measure_sweep_time_us = sa_ST2US(chVTGetSystemTimeX() - measure);
  buf_index = start; // Is used to skip 1st entry during level triggering
  buf_end = sweep_points - 1;
  buf_read = true;
}
#endif

#ifdef __LISTEN__
const uint8_t dBm_to_volt [] =
{
 255,
 225,
 198,
 175,
 154,
 136,
 120,
 106,
 93,
 82,
 72,
 64,
 56,
 50,
 44,
 39,
 34,
 30,
 26,
 23,
 21,
 18,
 16,
 14,
 12,
 11,
 10,
 8,
 7,
 7,
 6,
 5,
 5,
};

void SI4432_Listen(int s)
{
  SI4432_Sel = s;
  uint16_t sel = SI_nSEL[SI4432_Sel];
  uint8_t max = 0;
  uint16_t count = 0;
  chSysLock();
  operation_requested &= ~(OP_LEVER|OP_TOUCH);  // keep a console request, its command is waiting
  chSysUnlock();
  do {
      uint8_t v;
      palClearPad(GPIOC, sel);
      shiftOut(SI4432_REG_RSSI);
      v = shiftIn();
      palSetPad(GPIOC, sel);
      if (max < v)                // Peak
        max = v;
      if (count > 1000) {         // Decay
        max -= 1;
        count = 0;
      } else
        count++;
      v = max - v;
      DAC->DHR12R1 = dBm_to_volt[v] << 4; // Use DAC: CH1 and put 12 bit right aligned value
    } while(operation_requested == OP_NONE);
  count = 0;
//   dacPutChannelX(&DACD1, 0, 0);
}
#endif

#define MINIMUM_WAIT_FOR_RSSI   280
uint32_t SI4432_offset_delay = 300;

pureRSSI_t getSI4432_RSSI_correction(void){
  return SI4432_RSSI_correction;
};

pureRSSI_t SI4432_RSSI(uint32_t i, int s, bool break_on_operation)
{
  (void) i;
  int32_t RSSI_RAW;
  // SEE DATASHEET PAGE 61
#ifdef USE_SI4463           // Not used!!!!!!!
  if (SI4432_Sel == 2) {
    RSSI_RAW = Si446x_getRSSI();
  } else
#endif
//START_PROFILE
#ifdef __FAST_SWEEP__
  if (buf_read) {
    pureRSSI_t val = DEVICE_TO_PURE_RSSI(age[buf_index++]);
    if (buf_index >= sweep_points)
      buf_index = 0;
    if (buf_index == buf_end)
      buf_read = false;
    return val;
  }
#endif
  SI4432_Sel = s;
  int stepdelay = SI4432_step_delay;
  if (SI4432_frequency_changed) {
    if (stepdelay < MINIMUM_WAIT_FOR_RSSI) {
      stepdelay = MINIMUM_WAIT_FOR_RSSI;
    }
    SI4432_frequency_changed = false;
  } else if (SI4432_offset_changed) {
//    stepdelay = MINIMUM_WAIT_FOR_RSSI + (stepdelay - MINIMUM_WAIT_FOR_RSSI)/8;
    stepdelay = SI4432_offset_delay;
    SI4432_offset_changed = false;
  }
  if (stepdelay)
    my_microsecond_delay(stepdelay);
    // chThdSleepMicroseconds(SI4432_step_delay);
  i = setting.repeat;
  RSSI_RAW  = 0;
  do{
    RSSI_RAW += DEVICE_TO_PURE_RSSI((deviceRSSI_t)SI4432_Read_Byte(SI4432_REG_RSSI));
    if (--i == 0) break;
    my_microsecond_delay(100);
    if (break_on_operation && operation_requested)
      break;
  }while(1);

  if (setting.repeat > 1){
    RSSI_RAW = RSSI_RAW / setting.repeat;
  }
 //   if (MODE_INPUT(setting.mode) && RSSI_RAW == 0)
 //     SI4432_Init();
#ifdef __SIMULATION__
#error "Fixme!!! add correct simulation in pureRSSI_t type"
  RSSI_RAW = Simulated_SI4432_RSSI(i,s);
#endif
//STOP_PROFILE
  // Serial.println(dBm,2);
  return RSSI_RAW;
}

static const uint8_t SI4432_init_script[] =
{
  SI4432_INT_ENABLE1, 0x0,
  SI4432_INT_ENABLE2, 0x0,
  SI4432_CLOCK_RECOVERY_GEARSHIFT, 0x00,
  SI4432_AGC_OVERRIDE, 0x60,
  SI4432_AFC_LOOP_GEARSHIFT_OVERRIDE, 0x00,
  SI4432_AFC_TIMING_CONTROL, 0x02,
  SI4432_CLOCK_RECOVERY_GEARSHIFT, 0x03,
  SI4432_CLOCK_RECOVERY_OFFSET2, 0x01,
  SI4432_CLOCK_RECOVERY_OFFSET1, 0x11,
  SI4432_CLOCK_RECOVERY_OFFSET0, 0x11,
  SI4432_CLOCK_RECOVERY_TIMING_GAIN1, 0x01,
  SI4432_CLOCK_RECOVERY_TIMING_GAIN0, 0x13,
  SI4432_AFC_LIMITER, 0xFF,
  SI4432_DATAACCESS_CONTROL, 0x61, // Disable all packet handling
  SI4432_AGC_OVERRIDE, 0x60, // AGC, no LNA, fast gain increment
  SI4432_GPIO0_CONF, 0x12, // Normal
  SI4432_GPIO1_CONF, 0x15,
  SI4432_GPIO2_CONF, 0x1F
};


void SI4432_Sub_Init(void)
{
  SI4432_Reset();
  const uint8_t *p = SI4432_init_script;
  while (*p) {
    uint8_t r = *p++;
    uint8_t v = *p++;
    SI4432_Write_Byte (r,v);
  }
  // IF Filter Bandwidth
//  set_rbw(100) ;

//  SI4432_Receive();

}

#define V0_XTAL_CAPACITANCE 0x64
#define V1_XTAL_CAPACITANCE 0x64



void SI4432_Init()
{
#if 1

  CS_SI0_LOW;                       // Drop CS so power can be removed
  CS_SI1_LOW;                       // Drop CS so power can be removed
  CS_PE_LOW;                        // low is the default safe state
  SPI2_CLK_LOW;                     // low is the default safe state
  SPI2_SDI_LOW;                     // will be set with any data out

  palClearPad(GPIOB, GPIO_RF_PWR);  // Drop power
  chThdSleepMilliseconds(10);      // Wait
  palSetPad(GPIOB, GPIO_RF_PWR);    // Restore power
  CS_SI0_HIGH;                      // And set chip select lines back to inactive
  CS_SI1_HIGH;
  chThdSleepMilliseconds(10);      // Wait
#endif
  SPI2_CLK_LOW;
  //DebugLine("IO set");
  SI4432_Sel = SI4432_RX;
  SI4432_Sub_Init();

  SI4432_Sel = SI4432_LO;
  SI4432_Sub_Init();
//DebugLine("1 init done");
}

void set_calibration_freq(int freq)
{
  SI4432_Sel = SI4432_LO;         //Select Lo module
  if (freq < 0 || freq > 7 ) {
    SI4432_Write_Byte(SI4432_GPIO2_CONF, 0x1F) ; // Set GPIO2 to GND
  } else {
    SI4432_Write_Byte(SI4432_GPIO2_CONF, 0xC0) ; // Set GPIO2 maximumdrive and clock output
    SI4432_Write_Byte(Si4432_UC_OUTPUT_CLOCK, freq & 0x07) ; // Set GPIO2 frequency
  }
}
#endif
#endif

//------------PE4302 -----------------------------------------------
#ifdef __PE4302__

// Comment out this define to use parallel mode PE4302

#define PE4302_en 10

void PE4302_init(void) {
  CS_PE_LOW;
}

#define PE4302_DELAY 100
#if 0
void PE4302_shiftOut(uint8_t val)
{
     uint8_t i;
     SI4432_log(SI4432_Sel);
     SI4432_log(val);
     for (i = 0; i < 8; i++)  {
           if (val & (1 << (7 - i)))
             SPI2_SDI_HIGH;
           else
             SPI2_SDI_LOW;
//           chThdSleepMicroseconds(PE4302_DELAY);
           SPI2_CLK_HIGH;
//           chThdSleepMicroseconds(PE4302_DELAY);
           SPI2_CLK_LOW;
//           chThdSleepMicroseconds(PE4302_DELAY);
     }
}
#endif

static unsigned char old_attenuation = 255;
bool PE4302_Write_Byte(unsigned char DATA )
{
  if (old_attenuation == DATA)
    return false;
//  chThdSleepMicroseconds(PE4302_DELAY);
//  SPI2_CLK_LOW;
//  chThdSleepMicroseconds(PE4302_DELAY);
//  PE4302_shiftOut(DATA);

  shiftOut(DATA);
//  chThdSleepMicroseconds(PE4302_DELAY);
  CS_PE_HIGH;
//  chThdSleepMicroseconds(PE4302_DELAY);
  CS_PE_LOW;
//  chThdSleepMicroseconds(PE4302_DELAY);
  return true;
}

#endif

#if 0
//-----------------SI4432 dummy------------------
void SI4432_Write_Byte(unsigned char ADR, unsigned char DATA ) {}
unsigned char SI4432_Read_Byte(unsigned char ADR) {return ADR;}
float set_rbw(float WISH) {return (WISH > 600.0?600: (WISH<3.0?3:WISH));}
void set_calibration_freq(int p) {}
void SI4432_Set_Frequency(long f) {}
void PE4302_Write_Byte(unsigned char DATA ) {}
void PE4302_init(void) {}
#endif

#ifdef __SIMULATION__
unsigned long seed = 123456789;
extern float actual_rbw;
float myfrand(void)
{
  seed = (unsigned int) (1103515245 * seed + 12345) ;
  return ((float) seed) / 1000000000.0;
}
#define NOISE  ((myfrand()-2) * 2)  // +/- 4 dBm noise
extern int settingAttenuate;

//#define LEVEL(i, f, v) (v * (1-(fabs(f - frequencies[i])/actual_rbw/1000)))

float LEVEL(uint32_t i, freq_t f, int v)
{
  float dv;
  float df = fabs((float)f - (float)i);
  if (df < actual_rbw*1000)
    dv = df/(actual_rbw*1000);
  else
    dv =  1 + 50*(df - actual_rbw*1000)/(actual_rbw*1000);
  return (v - dv - settingAttenuate);
}

float Simulated_SI4432_RSSI(uint32_t i, int s)
{
  SI4432_Sel = s;
  float v = -100 + log10(actual_rbw)*10 + NOISE;
  if(s == 0) {
    v = fmax(LEVEL(i,10000000,-20),v);
    v = fmax(LEVEL(i,20000000,-40),v);
    v = fmax(LEVEL(i,30000000,-30),v);
    v = fmax(LEVEL(i,40000000,-90),v);
  } else {
    v = fmax(LEVEL(i,320000000,-20),v);
    v = fmax(LEVEL(i,340000000,-40),v);
    v = fmax(LEVEL(i,360000000,-30),v);
    v = fmax(LEVEL(i,380000000,-90),v);
  }
  return(v);
}

#endif
//------------------------------- ADF4351 -------------------------------------

#ifdef __ADF4351__

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

#define CS_ADF0_HIGH     palSetPad(GPIOA, 9)
#define CS_ADF1_HIGH     palSetPad(GPIOA, 10)

#define CS_ADF0_LOW     palClearPad(GPIOA, 9)
#define CS_ADF1_LOW     palClearPad(GPIOA, 10)

#define SPI3_CLK_HIGH   palSetPad(GPIOA, 1)
#define SPI3_CLK_LOW    palClearPad(GPIOA, 1)

#define SPI3_SDI_HIGH   palSetPad(GPIOA, 2)
#define SPI3_SDI_LOW    palClearPad(GPIOA, 2)


void ADF_shiftOut(uint8_t val)
{
     uint8_t i;
     for (i = 0; i < 8; i++)  {
           if (val & (1 << (7 - i)))
             SPI3_SDI_HIGH;
           else
             SPI3_SDI_LOW;
//           chThdSleepMicroseconds(10);
           SPI3_CLK_HIGH;
//           chThdSleepMicroseconds(10);
           SPI3_CLK_LOW;
//           chThdSleepMicroseconds(10);
     }
}

//unsigned long registers[6] =  {0x4580A8, 0x80080C9, 0x4E42, 0x4B3, 0xBC803C, 0x580005} ;
//unsigned long registers[6] =  {0x4C82C8, 0x80083E9, 0x6E42, 0x8004B3, 0x8C81FC, 0x580005} ;

//uint32_t registers[6] =  {0x320000, 0x8008011, 0x4E42, 0x4B3,0x8C803C , 0x580005} ;         //25 MHz ref

uint32_t registers[6] =  {0xA00000, 0x8000011, 0x4E42, 0x4B3,0xDC003C , 0x580005} ;         //10 MHz ref

int debug = 0;
int ADF4351_LE[2] = { 9, 10};
int ADF4351_Mux = 7;


//#define DEBUG(X) // Serial.print( X )
//#define DEBUGLN(X) Serial.println( X )
//#define DEBUGFLN(X,Y) Serial.println( X,Y )
//#define DEBUGF(X,Y) Serial.print( X,Y )
#define DEBUG(X)
#define DEBUGLN(X)


double RFout, //Output freq in MHz
#if 0   //Black modules
  PFDRFout[6] = {25.0,25.0,25.0,10.0,10.0,10.0}, //Reference freq in MHz
  Chrystal[6] = {25.0,25.0,25.0,10.0,10.0,10.0},
#else // Green modules
  PFDRFout[6] = {10.0,10.0,10.0,10.0,10.0,10.0}, //Reference freq in MHz
  Chrystal[6] = {10.0,10.0,10.0,10.0,10.0,10.0},
#endif

  OutputChannelSpacing = 0.010, // = 0.01
  FRACF; // Temp

unsigned int long RFint,  // Output freq/10Hz
  INTA,         // Temp
  RFcalc, //UI
  MOD, //Temp
  FRAC; //Temp

uint8_t OutputDivider; // Temp
uint8_t lock=2; //Not used

// Lock = A4

void ADF4351_Setup()
{
//  palSetPadMode(GPIOA, 1, PAL_MODE_OUTPUT_PUSHPULL );
//  palSetPadMode(GPIOA, 2, PAL_MODE_OUTPUT_PUSHPULL );

  SPI3_CLK_HIGH;
  SPI3_SDI_HIGH;
  CS_ADF0_HIGH;
  CS_ADF1_HIGH;
//  bitSet (registers[2], 17); // R set to 8
//  bitClear (registers[2], 14); // R set to 8

//  while(1) {
//
  ADF4351_set_frequency(0,100000000,0);
  ADF4351_set_frequency(1,150000000,0);
//  ADF4351_Set(0);
//  ADF4351_Set(1);
//  chThdSleepMilliseconds(1000);
//  }
//  bitSet (registers[2], 17); // R set to 8
//  bitClear (registers[2], 14); // R set to 8
//  for (int i=0; i<6; i++) pinMode(ADF4351_LE[i], OUTPUT);          // Setup pins
//  for (int i=0; i<6; i++) digitalWrite(ADF4351_LE[i], HIGH);
//  pinMode(ADF4351_Mux, INPUT);
//  SPI.begin();                          // Init SPI bus
//  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  //SPI.setDataMode(SPI_MODE0);           // CPHA = 0  Clock positive
  //SPI.setBitOrder(MSBFIRST);
}

void ADF4351_WriteRegister32(int channel, const uint32_t value)
{
  palClearPad(GPIOA, ADF4351_LE[channel]);
//  chThdSleepMicroseconds(10);
  for (int i = 3; i >= 0; i--) ADF_shiftOut((value >> (8 * i)) & 0xFF);
//  chThdSleepMicroseconds(10);
  palSetPad(GPIOA, ADF4351_LE[channel]);
//  chThdSleepMicroseconds(10);
  palClearPad(GPIOA, ADF4351_LE[channel]);
//  chThdSleepMicroseconds(10);
}

void ADF4351_disable_output()
{
    bitClear (registers[4], 5); // digital lock
    ADF4351_Set(0);
}

void ADF4351_enable_output()
{
    bitSet (registers[4], 5); // digital lock
    ADF4351_Set(0);
}
void ADF4351_Set(int channel)
{ for (int i = 5; i >= 0; i--) {
    ADF4351_WriteRegister32(channel, registers[i]);
//    if (debug)  Serial.println(registers[i],HEX);
}
}

void ADF4351_set_frequency(int channel, unsigned long freq, int drive)  // freq / 10Hz
{
  ADF4351_prep_frequency(channel,freq, drive);
  ADF4351_Set(channel);
}

void ADF4351_spur_mode(int S)
{
    if (S & 1) {
      bitSet (registers[2], 29); // R set to 8
    } else {
      bitClear (registers[2], 29); // R set to 8
    }
    if (S & 2)
      bitSet (registers[2], 30); // R set to 8
    else
      bitClear (registers[2], 30); // R set to 8
}

void ADF4351_R_counter(int R)
{
      int dbl = false;
      if (R < 0) {
        dbl = true;
        R = -R;
      }
      if (R<1)
        return;
      if (dbl) {
        bitSet (registers[2], 25); // Reference doubler
      } else {
        bitClear (registers[2], 25); // Reference doubler
      }
      for (int channel=0; channel < 6; channel++) {
        PFDRFout[channel] = Chrystal[channel] * (dbl?2:1) / R;
      }
      registers[2] &= ~ (((unsigned long)0x3FF) << 14);
      registers[2] |= (((unsigned long)R) << 14);
}

void ADF4351_CP(int p)
{
      registers[2] &= ~ (((unsigned long)0xF) << 9);
      registers[2] |= (((unsigned long)p) << 9);
}

void ADF4351_level(int p)
{
      registers[4] &= ~ (((unsigned long)0x3) << 3);
      registers[4] |= (((unsigned long)p) << 3);
}

void ADF4351_channel_spacing(int spacing)
{
  OutputChannelSpacing = 0.001 * spacing;
}

static uint32_t gcd(uint32_t x, uint32_t y)
{
  uint32_t z;
  while (y != 0) {
    z = x % y;
    x = y;
    y = z;
  }
  return x;
}

void ADF4351_prep_frequency(int channel, unsigned long freq, int drive)  // freq / 10Hz
{
  (void)drive;
//  if (channel == 0)
    RFout=freq/1000000.0;  // To MHz
//  else
 //   RFout=freq/1000002.764;  // To MHz

    if (RFout >= 2200) {
      OutputDivider = 1;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 0);
    } else if (RFout >= 1100) {
      OutputDivider = 2;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 1);
    } else if (RFout >= 550) {
      OutputDivider = 4;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 1);
      bitWrite (registers[4], 20, 0);
    } else if (RFout >= 275)  {
      OutputDivider = 8;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 1);
      bitWrite (registers[4], 20, 1);
    } else if (RFout >= 137.5)  {
      OutputDivider = 16;
      bitWrite (registers[4], 22, 1);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 0);
    } else if (RFout >= 68.75) {
      OutputDivider = 32;
      bitWrite (registers[4], 22, 1);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 1);
    } else {
      OutputDivider = 64;
      bitWrite (registers[4], 22, 1);
      bitWrite (registers[4], 21, 1);
      bitWrite (registers[4], 20, 0);
    }

    INTA = (RFout * OutputDivider) / PFDRFout[channel];
    MOD = (PFDRFout[channel] / OutputChannelSpacing) + 0.01;
//    MOD = 3125;
    FRACF = (((RFout * OutputDivider) / PFDRFout[channel]) - INTA) * MOD;
    FRAC = roundf(FRACF);

  while (FRAC > 4095 || MOD > 4095) {
    FRAC = FRAC >> 1;
    MOD = MOD >> 1;
 //   Serial.println( "MOD/FRAC reduced");
  }

    int32_t k = gcd(FRAC, MOD);
    if (k > 1) {
      FRAC /= k;
      MOD /= k;
//      Serial.print( "MOD/FRAC gcd reduced");
    }
//    while (denom >= (1<<20)) {
//      num >>= 1;
//      denom >>= 1;
//    }


//  if (INTA <= 75) Serial.println( "INTA <= 75");
//  if (FRAC > 4095) Serial.println( "FRAC > 4095");
//  if (MOD > 4095) Serial.println( "MOD > 4095");


//  if (FRAC > 4095) Serial.println( "FRAC > 4095");
//  if (MOD > 4095) Serial.println( "MOD > 4095");
//  if (INTA > 4095) Serial.println( "INT > 4095");

  if (debug) {
    DEBUG("  ODIV=");
    DEBUG(OutputDivider);
    DEBUG("  INT=");
    DEBUG(INTA);
    DEBUG("  FRAC=");
    DEBUG(FRAC);
    DEBUG("  MOD=");
    DEBUG(MOD);
    DEBUG( " CalF=");
//    DEBUGFLN(PFDRFout[channel] *(INTA + ((double)FRAC)/MOD)/OutputDivider,6);

//  DEBUG("  FRACF=");
//  DEBUGF(FRACF,6);
  }
    registers[0] = 0;
    registers[0] = INTA << 15; // OK
    FRAC = FRAC << 3;
    registers[0] = registers[0] + FRAC;
    if (MOD == 1) MOD = 2;
    registers[1] = 0;
    registers[1] = MOD << 3;
    registers[1] = registers[1] + 1 ; // restore address "001"
    bitSet (registers[1], 27); // Prescaler at 8/9
/*
    drive = 1;
    if (drive == 0) {
      bitClear (registers[4], 3); // +5dBm + out
      bitClear (registers[4], 4); // +5dBm
      bitClear (registers[4], 6); // +5dBm - out
      bitClear (registers[4], 7); // +5dBm
    } else if (drive == 1) {
      bitSet (registers[4], 6); // +5dBm
      bitClear (registers[4], 7); // +5dBm - out
      bitSet (registers[4], 3); // +5dBm
      bitClear (registers[4], 4); // +5dBm + out
    } else if (drive == 2) {
      bitClear (registers[4], 6); // +5dBm - out
      bitSet (registers[4], 7); // +5dBm
      bitClear (registers[4], 3); // +5dBm + out
      bitSet (registers[4], 4); // +5dBm
    }
    else {
      bitSet (registers[4], 6); // +5dBm - out
      bitSet (registers[4], 7); // +5dBm
      bitSet (registers[4], 3); // +5dBm + out
      bitSet (registers[4], 4); // +5dBm
    }
*/
//    bitSet (registers[4], 5); // enable + output
//    bitClear (registers[4], 8); // enable B output

#if 0
    if (FRAC == 0)
      bitSet (registers[2], 8); // INT mode
    else
      bitClear (registers[2], 8); // INT mode
    bitSet (registers[2], 13); // Double buffered

    bitSet (registers[2], 28); // Digital lock == "110" sur b28 b27 b26
    bitSet (registers[2], 27); // digital lock
    bitClear (registers[2], 26); // digital lock

    //bitSet (registers[4], 10); // Mute till lock
//    bitSet (registers[3], 23); // Fast lock
 #endif
//   bitSet (registers[4], 10); // Mute till lock
//    ADF4351_Set(channel);
}



#endif

#pragma GCC pop_options
