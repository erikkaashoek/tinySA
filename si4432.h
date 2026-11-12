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


#ifndef __SI4432_H__

#define __SI4432_H__

extern uint32_t SI4432_step_delay;
extern uint32_t SI4432_offset_delay;
#ifdef __SI4432__

//
#define MAX_SI4432                         2
// #define SI4432_DUMMY                       2     // never used

#define SI4432_DEV_TYPE                    0x00
#define SI4432_DEV_VERSION                 0x01
#define SI4432_DEV_STATUS                  0x02
#define SI4432_INT_STATUS1                 0x03
#define SI4432_INT_STATUS2                 0x04
#define SI4432_INT_ENABLE1                 0x05
#define SI4432_INT_ENABLE2                 0x06
#define SI4432_STATE                       0x07
#define SI4432_OPERATION_CONTROL           0x08
#define Si4432_CRYSTAL_OSC_LOAD_CAP        0x09
#define Si4432_UC_OUTPUT_CLOCK             0x0A
#define SI4432_GPIO0_CONF                  0x0B
#define SI4432_GPIO1_CONF                  0x0C
#define SI4432_GPIO2_CONF                  0x0D
#define SI4432_IOPORT_CONF                 0x0E
#define SI4432_IF_FILTER_BW                0x1C
#define SI4432_AFC_LOOP_GEARSHIFT_OVERRIDE 0x1D
#define SI4432_AFC_TIMING_CONTROL          0x1E
#define SI4432_CLOCK_RECOVERY_GEARSHIFT    0x1F
#define SI4432_CLOCK_RECOVERY_OVERSAMPLING 0x20
#define SI4432_CLOCK_RECOVERY_OFFSET2      0x21
#define SI4432_CLOCK_RECOVERY_OFFSET1      0x22
#define SI4432_CLOCK_RECOVERY_OFFSET0      0x23
#define SI4432_CLOCK_RECOVERY_TIMING_GAIN1 0x24
#define SI4432_CLOCK_RECOVERY_TIMING_GAIN0 0x25
#define SI4432_REG_RSSI                    0x26
#define SI4432_RSSI_THRESHOLD              0x27
#define SI4432_AFC_LIMITER                 0x2A
#define SI4432_AFC_CORRECTION_READ         0x2B
#define Si4432_OOK_COUNTER_VALUE_1         0x2C
#define Si4432_OOK_COUNTER_VALUE_2         0x2D
#define Si4432_SLICER_PEAK_HOLD            0x2E
#define SI4432_DATAACCESS_CONTROL          0x30
#define SI4432_EZMAC_STATUS                0x31
#define SI4432_HEADER_CONTROL1             0x32
#define SI4432_HEADER_CONTROL2             0x33
#define SI4432_PREAMBLE_LENGTH             0x34
#define SI4432_PREAMBLE_DETECTION          0x35
#define SI4432_SYNC_WORD3                  0x36
#define SI4432_SYNC_WORD2                  0x37
#define SI4432_SYNC_WORD1                  0x38
#define SI4432_SYNC_WORD0                  0x39
#define SI4432_TRANSMIT_HEADER3            0x3A
#define SI4432_TRANSMIT_HEADER2            0x3B
#define SI4432_TRANSMIT_HEADER1            0x3C
#define SI4432_TRANSMIT_HEADER0            0x3D
#define SI4432_PKG_LEN                     0x3E
#define SI4432_CHECK_HEADER3               0x3F
#define SI4432_CHECK_HEADER2               0x40
#define SI4432_CHECK_HEADER1               0x41
#define SI4432_CHECK_HEADER0               0x42
#define SI4432_RECEIVED_HEADER3            0x47
#define SI4432_RECEIVED_HEADER2            0x48
#define SI4432_RECEIVED_HEADER1            0x49
#define SI4432_RECEIVED_HEADER0            0x4A
#define SI4432_RECEIVED_LENGTH             0x4B
#define SI4432_CHARGEPUMP_OVERRIDE         0x58
#define SI4432_DIVIDER_CURRENT_TRIM        0x59
#define SI4432_VCO_CURRENT_TRIM            0x5A
#define SI4432_AGC_OVERRIDE                0x69
#define SI4432_TX_POWER                    0x6D
#define SI4432_TX_DATARATE1                0x6E
#define SI4432_TX_DATARATE0                0x6F
#define SI4432_MODULATION_MODE1            0x70
#define SI4432_MODULATION_MODE2            0x71
#define SI4432_FREQ_DEVIATION              0x72
#define SI4432_FREQ_OFFSET1                0x73
#define SI4432_FREQ_OFFSET2                0x74
#define SI4432_FREQBAND                    0x75
#define SI4432_FREQCARRIER_H               0x76
#define SI4432_FREQCARRIER_L               0x77
#define SI4432_FREQCHANNEL                 0x79
#define SI4432_CHANNEL_STEPSIZE            0x7A
#define SI4432_FIFO                        0x7F


extern  uint16_t SI4432_Sel;         // currently selected SI4432

extern uint8_t SI4432_frequency_changed;
extern uint8_t SI4432_offset_changed;

void SI4432_shiftOutDword(uint32_t buf, uint16_t size);
#define SI4432_Write_Byte(ADR, DATA)                  {uint32_t temp = (((ADR|0x80)&0xFF)<<0)|(((DATA )&0xFF)<<8); SI4432_shiftOutDword(temp, 2);}
#define SI4432_Write_2_Byte(ADR, DATA1, DATA2)        {uint32_t temp = (((ADR|0x80)&0xFF)<<0)|(((DATA1)&0xFF)<<8)|(((DATA2)&0xFF)<<16);SI4432_shiftOutDword(temp, 3);}
#define SI4432_Write_3_Byte(ADR, DATA1, DATA2, DATA3) {uint32_t temp = (((ADR|0x80)&0xFF)<<0)|(((DATA1)&0xFF)<<8)|(((DATA2)&0xFF)<<16)|(((DATA3)&0xFF)<<24);SI4432_shiftOutDword(temp, 4);}

//void SI4432_Write_Byte(uint8_t ADR, uint8_t DATA);
//void SI4432_Write_2_Byte(uint8_t ADR, uint8_t DATA1, uint8_t DATA2);
//void SI4432_Write_3_Byte(uint8_t ADR, uint8_t DATA1, uint8_t DATA2, uint8_t DATA3 );

uint8_t SI4432_Read_Byte(uint8_t ADR);

void SI4432_Transmit(int d);
void SI4432_Receive(void);
void SI4432_Reset(void);

void SI4432_Init(void);
void SI4432_Drive(int);
pureRSSI_t getSI4432_RSSI_correction(void);
pureRSSI_t SI4432_RSSI(uint32_t i, int s, bool break_on_operation);
#ifdef __SIMULATION__
float Simulated_SI4432_RSSI(uint32_t i, int s);
#endif
void SI4432_Set_Frequency ( uint32_t Freq );

uint16_t force_rbw(int i);
uint16_t set_rbw(uint16_t WISH);
void set_calibration_freq(int freq);
#ifdef __FAST_SWEEP__
void SI4432_Fill(int s, int start);
void SI4432_Fill_clear(void);
void SI4432_trigger_fill(int s, uint8_t trigger_lvl, int up_direction, int trigger_mode);
#if 0
int SI4432_is_fast_mode(void);
#endif
#endif

#endif


bool PE4302_Write_Byte(unsigned char DATA );
void PE4302_init(void);

#ifdef __ADF4351__
extern ioline_t ADF4351_LE[];
extern int debug;
void   ADF4351_Setup(void);


void ADF4351_WriteRegister32(int channel, const uint32_t value);
void ADF4351_Latch(void);
uint64_t ADF4351_set_frequency(int channel, uint64_t freq);
uint64_t ADF4351_prepare_frequency(int channel, uint64_t freq);
//int ADF4351_set_frequency_with_offset(uint32_t freq, int offset, uint8_t drive_strength);
void ADF4351_Set(int channel);
void ADF4351_spur_mode(int S);
void ADF4351_R_counter(int R);
void ADF4351_channel_spacing(int spacing);
void ADF4351_CP(int p);
void ADF4351_drive(int p);
void ADF4351_aux_drive(int p);
void ADF4351_enable(int p);
void ADF4351_enable_aux_out(int p);
int ADF4351_locked(void);
void ADF4350_shift_ref(int f);

//void ADF4351_enable_out(int p);
//void ADF4351_enable(int s);
//void ADF4351_enable_aux_out(int s);


#endif

#ifdef __SI4463__
#include "si446x_defs.h"
#include "si446x.h"
int16_t Si446x_RSSI(bool break_on_operation);
uint8_t getFRR(uint8_t reg);
si446x_state_t getState(void);
void setState(si446x_state_t newState);

extern si446x_info_t SI4463_info;
pureRSSI_t getSI4463_RSSI_correction(void);
void Si446x_getInfo(si446x_info_t* info);
void SI446x_Fill(int s, int start);
void SI446x_Fill_clear(void);
void SI446x_trigger_fill(int s, uint8_t trigger_lvl, int up_direction, int trigger_mode);
void SI4463_init(void);
void set_calibration_freq(int freq);
#define ADF4351_LO 3
#define ADF4351_LO2 4
#define SI4463_RX  2

#endif

#ifdef TINYSA4
void enable_extra_lna(int s);
void enable_ultra(int s);
void enable_rx_output(int s);
void enable_high(int s);
void enable_direct(int s);
void enable_ADF_output(int f, int t);
#endif


#endif //__SI4432_H__
