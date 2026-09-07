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
#include "hal.h"
#include "nanovna.h"
#ifdef __SI5351__
#include "si5351.h"

//inline int palReadLine(uint32_t line) {
//  return ( palReadPort(PAL_PORT(line)) & (1<<PAL_PAD(line)) )
//}
char  pll_lock_failed = 0;
int si5351_available = false;

    /*
     * Software i2c bus
     */
#define I2C_DELAY     my_microsecond_delay(20);

    static inline void scl_low(void) {
      palClearLine(LINE_SCL);
        I2C_DELAY;
    }

    static inline void scl_high(void) {
      palSetLine(LINE_SCL);
        I2C_DELAY;
    }

    static inline void sda_low(void) {
      palClearLine(LINE_SDA);
        I2C_DELAY;
    }

    static inline void sda_high(void) {
      palSetLine(LINE_SDA);
        I2C_DELAY;
    }

    static void i2c_begin(void) {
        sda_low();
        scl_low();
    }

    static void i2c_end(void) {
        sda_low();
        scl_high();
        sda_high();
    }

    uint32_t i2c_recv(int bits) {
        uint32_t ret = 0;
//        soft_i2c_sda.set_mode(GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, HIGH); // Input pullup
        while(bits--) {
            scl_high();
            ret<<= 1;
            if (palReadLine(LINE_SDA)) ret|=1;
            scl_low();
        }
//        soft_i2c_sda.set_mode(GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, LOW); // Output low
        return ret;
    }

#define CLOCK_TICK      {scl_high(); scl_low();}
#define DATA_OUT(bit)   {if (bit) sda_high(); else sda_low();}
    static bool i2c_send(uint8_t data) {
        // put data on bus
        for (uint16_t mask = 0x80; mask; mask>>=1){
            DATA_OUT(data&mask);
            CLOCK_TICK; // clock tick
        }
        // Read answer bit
        bool ret = i2c_recv(1);
        // Stop transfer at error (no answer)
        if (ret)
            i2c_end();
        return ret;
    }

    void i2c_init(void) {
//        soft_i2c_clk.set_mode(GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL);
//        soft_i2c_sda.set_mode(GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL);
        i2c_begin();
        i2c_end();
    }
#define I2C_WRITE           0
#define I2C_READ            1

    bool i2c_probe(uint8_t devAddr) {
        i2c_begin();
        // device address
        if(i2c_send((devAddr<<1) | I2C_WRITE))
            return false;
        i2c_end();
        return true;
    }

    // return value: 0: success; -1: no device ack; -2: no register addr ack; -3: no data ack
    int i2c_write(uint8_t devAddr, uint8_t addr, uint8_t data) {
        i2c_begin();
        // device address
        if(i2c_send((devAddr<<1) | I2C_WRITE))
            return -1;
        // register address
        if(i2c_send(addr))
            return -2;
        // data
        if(i2c_send(data))
            return -3;
        i2c_end();
        return 0;
    }

    // return value: 0: success; -1: no device ack; -2: no register addr ack or data ack
    int i2c_write_buf(uint8_t devAddr, uint8_t* data, int len) {
        i2c_begin();
        // device address
        if(i2c_send((devAddr<<1) | I2C_WRITE))
            return -1;
        // data
        for(int i=0; i<len; i++)
            if(i2c_send(data[i]))
                return -2;
        i2c_end();
        return 0;
    }

    // return value: >= 0: the read data; -1: no device ack; -2: no register addr ack
    int i2c_read(uint8_t devAddr, uint8_t addr) {
        i2c_begin();
        // device address
        if(i2c_send((devAddr<<1) | I2C_READ))
            return -1;
        // register address
        if(i2c_send(addr))
            return -2;
        // data
        int res = i2c_recv(8);
        i2c_end();
        return res;
    }



#define SI5351_I2C_ADDR   	(0x60<<1)

static bool si5351_read(uint8_t reg, uint8_t* buf)
{
    int addr = SI5351_I2C_ADDR>>1;
    int v = i2c_read(addr, reg);
    if (v < 0)
      return false;
    *buf = (uint8_t) v;
    return true;
}

static bool si5351_write(uint8_t reg, uint8_t dat)
{
  int addr = SI5351_I2C_ADDR>>1;
  int s = i2c_write(addr, reg, dat);
  return s >= 0;
}

static bool si5351_bulk_write(const uint8_t *buf, int len)
{
  int addr = SI5351_I2C_ADDR>>1;
  int s = i2c_write_buf(addr, (uint8_t* )buf, len);
  return s >= 0;
}

// register addr, length, data, ...
static const uint8_t si5351_configs[] = {
  2, SI5351_REG_3_OUTPUT_ENABLE_CONTROL, 0xff,
  4, SI5351_REG_16_CLK0_CONTROL, SI5351_CLK_POWERDOWN, SI5351_CLK_POWERDOWN, SI5351_CLK_POWERDOWN,
//  2, SI5351_REG_183_CRYSTAL_LOAD, SI5351_CRYSTAL_LOAD_8PF,
  // setup PLL (30MHz * 30 = 900MHz, 30/2-2=13)
  9, SI5351_REG_26_PLL_A, /*P3*/0, 1, /*P1*/0, 13, 0, /*P3/P2*/0, 0, 0,
  // RESET PLL
  2, SI5351_REG_177_PLL_RESET, SI5351_PLL_RESET_A | SI5351_PLL_RESET_B,
  // setup multisynth (900MHz / 30 = 30MHz, 30/2-2=13)
  9, SI5351_REG_42_MULTISYNTH0, /*P3*/0, 1, /*P1*/0, 13, 0, /*P2|P3*/0, 0, 0,
#ifdef __ENABLE_CLK2__
  2, SI5351_REG_18_CLK2_CONTROL, SI5351_CLK_DRIVE_STRENGTH_2MA | SI5351_CLK_INPUT_MULTISYNTH_N | SI5351_CLK_INTEGER_MODE,
  2, SI5351_REG_3_OUTPUT_ENABLE_CONTROL, 0,
#else
  2, SI5351_REG_18_CLK2_CONTROL,SI5351_CLK_POWERDOWN,
  2, SI5351_REG_17_CLK1_CONTROL, SI5351_CLK_POWERDOWN,
  2, SI5351_REG_3_OUTPUT_ENABLE_CONTROL, 0x06,
#endif
  0 // sentinel
};

static bool si5351_wait_ready(void)
{
    uint8_t status = 0xff;
    systime_t start = chVTGetSystemTime();
    systime_t end = start + TIME_MS2I(1000); // 1000 ms timeout
    while (chVTIsSystemTimeWithin(start, end))
    {
        if(!si5351_read(0, &status))
            status = 0xff;  // comm timeout
        if ((status & 0x80) == 0) 
            return true;
    }
    return false;
}

#if 1
static void si5351_wait_pll_lock(void)
{
    systime_t start = chVTGetSystemTime();
    uint8_t status = 0xff;
    if(!si5351_read(0, &status))
        status = 0xff;  // comm timeout
    if ((status & 0x60) == 0)
        return;
    systime_t end = start + TIME_MS2I(100); // 100 ms timeout
    while (chVTIsSystemTimeWithin(start, end))
    {
        if(!si5351_read(0, &status))
            status = 0xff;  // comm timeout
        if ((status & 0x60) == 0)
            return;
    }
    pll_lock_failed = true;
}

#endif

bool si5351_init(void)
{
  if (!si5351_wait_ready())
      return false;
//  my_microsecond_delay(200);
  const uint8_t *p = si5351_configs;
  while (*p) {
    uint8_t len = *p++;
    if (!si5351_bulk_write(p, len))
        return false;
    p += len;
  }
  si5351_wait_pll_lock();
  if (pll_lock_failed)
    return false;
  return true;
}

static void si5351_disable_output(void)
{
  uint8_t reg[4];
  si5351_write(SI5351_REG_3_OUTPUT_ENABLE_CONTROL, 0xff);
  reg[0] = SI5351_REG_16_CLK0_CONTROL;
  reg[1] = SI5351_CLK_POWERDOWN;
  reg[2] = SI5351_CLK_POWERDOWN;
  reg[3] = SI5351_CLK_POWERDOWN;
  si5351_bulk_write(reg, 4);
}

static void si5351_enable_output(void)
{
#ifdef __ENABLE_CLK2__
  si5351_write(SI5351_REG_3_OUTPUT_ENABLE_CONTROL, 0x00);
#else
  si5351_write(SI5351_REG_3_OUTPUT_ENABLE_CONTROL, 0x04);
#endif
}

static void si5351_reset_pll(void)
{
  //si5351_write(SI5351_REG_177_PLL_RESET, SI5351_PLL_RESET_A | SI5351_PLL_RESET_B);
  si5351_write(SI5351_REG_177_PLL_RESET, 0xAC);
}

static void si5351_setupPLL(
    uint8_t     pll, /* SI5351_PLL_A or SI5351_PLL_B */
    uint8_t     mult,
    uint32_t    num,
    uint32_t    denom)
{
  /* Get the appropriate starting point for the PLL registers */
  const uint8_t pllreg_base[] = {
    SI5351_REG_26_PLL_A,
    SI5351_REG_34_PLL_B
  };
  uint32_t P1;
  uint32_t P2;
  uint32_t P3;

  /* Feedback Multisynth Divider Equation
   * where: a = mult, b = num and c = denom
   * P1 register is an 18-bit value using following formula:
   * 	P1[17:0] = 128 * mult + floor(128*(num/denom)) - 512
   * P2 register is a 20-bit value using the following formula:
   * 	P2[19:0] = 128 * num - denom * floor(128*(num/denom))
   * P3 register is a 20-bit value using the following formula:
   * 	P3[19:0] = denom
   */

  /* Set the main PLL config registers */
  if (num == 0)
  {
    /* Integer mode */
    P1 = 128 * mult - 512;
    P2 = 0;
    P3 = 1;
  }
  else
  {
    /* Fractional mode */
    //P1 = (uint32_t)(128 * mult + floor(128 * ((float)num/(float)denom)) - 512);
    P1 = 128 * mult + ((128 * num) / denom) - 512;
    //P2 = (uint32_t)(128 * num - denom * floor(128 * ((float)num/(float)denom)));
    P2 = 128 * num - denom * ((128 * num) / denom);
    P3 = denom;
  }

  /* The datasheet is a nightmare of typos and inconsistencies here! */
  uint8_t reg[9];
  reg[0] = pllreg_base[pll];
  reg[1] = (P3 & 0x0000FF00) >> 8;
  reg[2] = (P3 & 0x000000FF);
  reg[3] = (P1 & 0x00030000) >> 16;
  reg[4] = (P1 & 0x0000FF00) >> 8;
  reg[5] = (P1 & 0x000000FF);
  reg[6] = ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16);
  reg[7] = (P2 & 0x0000FF00) >> 8;
  reg[8] = (P2 & 0x000000FF);
  si5351_bulk_write(reg, 9);
}

static void si5351_setupMultisynth(
    uint8_t     output,
    uint8_t	    pllSource,
    uint32_t    div, // 4,6,8, 8+ ~ 900
    uint32_t    num,
    uint32_t    denom,
    uint32_t    rdiv, // SI5351_R_DIV_1~128
    uint8_t     drive_strength)
{
  /* Get the appropriate starting point for the PLL registers */
  const uint8_t msreg_base[] = {
    SI5351_REG_42_MULTISYNTH0,
    SI5351_REG_50_MULTISYNTH1,
    SI5351_REG_58_MULTISYNTH2,
  };
  const uint8_t clkctrl[] = {
    SI5351_REG_16_CLK0_CONTROL,
    SI5351_REG_17_CLK1_CONTROL,
    SI5351_REG_18_CLK2_CONTROL
  };
  uint8_t dat;

  uint32_t P1;
  uint32_t P2;
  uint32_t P3;
  uint32_t div4 = 0;

  /* Output Multisynth Divider Equations
   * where: a = div, b = num and c = denom
   * P1 register is an 18-bit value using following formula:
   * 	P1[17:0] = 128 * a + floor(128*(b/c)) - 512
   * P2 register is a 20-bit value using the following formula:
   * 	P2[19:0] = 128 * b - c * floor(128*(b/c))
   * P3 register is a 20-bit value using the following formula:
   * 	P3[19:0] = c
   */
  /* Set the main PLL config registers */
  if (div == 4) {
    div4 = SI5351_DIVBY4;
    P1 = P2 = 0;
    P3 = 1;
  } else if (num == 0) {
    /* Integer mode */
    P1 = 128 * div - 512;
    P2 = 0;
    P3 = 1;
  } else {
    /* Fractional mode */
    P1 = 128 * div + ((128 * num) / denom) - 512;
    P2 = 128 * num - denom * ((128 * num) / denom);
    P3 = denom;
  }

  /* Set the MSx config registers */
  uint8_t reg[9];
  reg[0] = msreg_base[output];
  reg[1] = (P3 & 0x0000FF00) >> 8;
  reg[2] = (P3 & 0x000000FF);
  reg[3] = ((P1 & 0x00030000) >> 16) | div4 | rdiv;
  reg[4] = (P1 & 0x0000FF00) >> 8;
  reg[5] = (P1 & 0x000000FF);
  reg[6] = ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16);
  reg[7] = (P2 & 0x0000FF00) >> 8;
  reg[8] = (P2 & 0x000000FF);
  si5351_bulk_write(reg, 9);

  /* Configure the clk control and enable the output */
  dat = drive_strength | SI5351_CLK_INPUT_MULTISYNTH_N;
  if (pllSource == SI5351_PLL_B)
    dat |= SI5351_CLK_PLL_SELECT_B;
  if (num == 0)
    dat |= SI5351_CLK_INTEGER_MODE;
  si5351_write(clkctrl[output], dat);
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

#define XTALFREQ 30000000L
#define PLL_N 30
#define PLLFREQ (XTALFREQ * PLL_N)

static void si5351_set_frequency_fixedpll(
    int channel, int pll, int pllfreq, int freq,
    uint32_t rdiv, uint8_t drive_strength)
{
    int32_t div = pllfreq / freq; // range: 8 ~ 1800
    int32_t num = pllfreq - freq * div;
    int32_t denom = freq;
    //int32_t k = freq / (1<<20) + 1;
    int32_t k = gcd(num, denom);
    num /= k;
    denom /= k;
    while (denom >= (1<<20)) {
      num >>= 1;
      denom >>= 1;
    }
    si5351_setupMultisynth(channel, pll, div, num, denom, rdiv, drive_strength);
}

static void si5351_set_frequency_fixeddiv(
    int channel, int pll, int freq, int div,
    uint8_t     drive_strength)
{
    int32_t pllfreq = freq * div;
    int32_t multi = pllfreq / XTALFREQ;
    int32_t num = pllfreq - multi * XTALFREQ;
    int32_t denom = XTALFREQ;
    int32_t k = gcd(num, denom);
    num /= k;
    denom /= k;
    while (denom >= (1<<20)) {
      num >>= 1;
      denom >>= 1;
    }
    si5351_setupPLL(pll, multi, num, denom);
    si5351_setupMultisynth(channel, pll, div, 0, 1, SI5351_R_DIV_1, drive_strength);
}

/* 
 * 1~100MHz fixed PLL 900MHz, fractional divider
 * 100~150MHz fractional PLL 600-900MHz, fixed divider 6
 * 150~200MHz fractional PLL 600-900MHz, fixed divider 4
 */
void si5351_set_frequency(int channel, int freq, uint8_t drive_strength)
{
  if (freq <= 100000000) {
    si5351_setupPLL(SI5351_PLL_A, 30, 0, 1);
    si5351_set_frequency_fixedpll(channel, SI5351_PLL_A, PLLFREQ, freq, SI5351_R_DIV_1, drive_strength);
  } else if (freq < 150000000) {
    si5351_set_frequency_fixeddiv(channel, SI5351_PLL_A, freq, 6, drive_strength);
  } else {
    si5351_set_frequency_fixeddiv(channel, SI5351_PLL_A, freq, 4, drive_strength);
  }
}

void si5351_set_int_mul_div(int channel, int multi, int div, uint8_t drive_strength)
{
  si5351_setupPLL(SI5351_PLL_A, multi, 0, 1);
  si5351_setupMultisynth(channel, SI5351_PLL_A, div, 0, 1, SI5351_R_DIV_1, drive_strength);
}

static int current_band = -1;

/*
 * configure output as follows:
 * CLK0: frequency + offset
 * CLK1: frequency
 * CLK2: fixed 8MHz
 */
#define CLK2_FREQUENCY 8000000L
int si5351_set_frequency_with_offset(uint32_t freq, int offset, uint8_t drive_strength)
{
  int band;
  int delay = 3;
  uint32_t ofreq = freq + offset;
  uint32_t rdiv = SI5351_R_DIV_1;
#ifdef __VNA__
  /* if (freq > config.harmonic_freq_threshold * 5 ) {
	    freq /= 7;
	    ofreq /= 9;
  }else */
  if (freq > config.harmonic_freq_threshold * 3) {
    freq /= 5;
    ofreq /= 7;
  } else if (freq > config.harmonic_freq_threshold) {
    freq /= 3;
    ofreq /= 5;
  }
#endif
  if (freq <= 100000000) {
    band = 0;
  } else if (freq < 160000000) {
    band = 1;
  } else {
    band = 2;
  }
  if (freq <= 500000) {
    rdiv = SI5351_R_DIV_64;
  } else if (freq <= 4000000) {
    rdiv = SI5351_R_DIV_8;
  }

#if 1
  if (current_band != band)
    si5351_disable_output();
#endif

  switch (band) {
  case 0:
    // fractional divider mode. only PLL A is used.
    if (current_band == 1 || current_band == 2){
    	si5351_reset_pll();
    	si5351_setupPLL(SI5351_PLL_A, PLL_N, 0, 1);
    }

    if (rdiv == SI5351_R_DIV_8) {
      freq *= 8;
      ofreq *= 8;
    } else if (rdiv == SI5351_R_DIV_64) {
      freq *= 64;
      ofreq *= 64;
    }

    si5351_set_frequency_fixedpll(0, SI5351_PLL_A, PLLFREQ, ofreq,
                                  rdiv, drive_strength);
    si5351_set_frequency_fixedpll(1, SI5351_PLL_A, PLLFREQ, freq,
                                  rdiv, drive_strength);
    //if (current_band != 0)
#ifdef __ENABLE_CLK2__
      si5351_set_frequency_fixedpll(2, SI5351_PLL_A, PLLFREQ, CLK2_FREQUENCY,
                                    SI5351_R_DIV_1, SI5351_CLK_DRIVE_STRENGTH_2MA);
#endif
    break;

  case 1:
    // Set PLL twice on changing from band 2
    if (current_band == 2) {
      si5351_set_frequency_fixeddiv(0, SI5351_PLL_A, ofreq, 6, drive_strength);
      si5351_set_frequency_fixeddiv(1, SI5351_PLL_B, freq, 6, drive_strength);
    }

    // div by 6 mode. both PLL A and B are dedicated for CLK0, CLK1
    si5351_set_frequency_fixeddiv(0, SI5351_PLL_A, ofreq, 6, drive_strength);
    si5351_set_frequency_fixeddiv(1, SI5351_PLL_B, freq, 6, drive_strength);
#ifdef __ENABLE_CLK2__
    si5351_set_frequency_fixedpll(2, SI5351_PLL_B, freq * 6, CLK2_FREQUENCY,
                                  SI5351_R_DIV_1, SI5351_CLK_DRIVE_STRENGTH_2MA);
#endif
    break;

  case 2:
    // div by 4 mode. both PLL A and B are dedicated for CLK0, CLK1
    si5351_set_frequency_fixeddiv(0, SI5351_PLL_A, ofreq, 4, drive_strength);
    si5351_set_frequency_fixeddiv(1, SI5351_PLL_B, freq, 4, drive_strength);
#ifdef __ENABLE_CLK2__
    si5351_set_frequency_fixedpll(2, SI5351_PLL_B, freq * 4, CLK2_FREQUENCY,
                                  SI5351_R_DIV_1, SI5351_CLK_DRIVE_STRENGTH_2MA);
#endif
    break;
  }

  if (current_band != band) {
    si5351_reset_pll();
    si5351_wait_pll_lock();
#if 1
    si5351_enable_output();
#endif
    delay += 10;
  }

  current_band = band;
  return delay;
}
#endif
