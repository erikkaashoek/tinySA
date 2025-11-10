/*
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
#include "spi.h"

#pragma GCC push_options
#pragma GCC optimize ("Os")

//#define __USE_FRR_FOR_RSSI__


// Define for use hardware SPI mode
#define USE_HARDWARE_SPI_MODE

// 10MHz clock
#define SI4432_10MHZ 10000000U
// !!!! FROM ili9341.c for disable it !!!!
//#define LCD_CS_HIGH    palSetPad(GPIOB, GPIOB_LCD_CS)

// Not use delays for CS
#if 1
 #define SI_CS_DELAY
 #define PE_CS_DELAY
 #define ADF_CS_DELAY
#else
 #define SI_CS_DELAY    {__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");}
 #define PE_CS_DELAY    {__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");}
 #define ADF_CS_DELAY   {__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");}
#endif

#define SI_CS_LOW      {palClearLine(LINE_RX_SEL);SI_CS_DELAY;}
#define SI_CS_HIGH     {SI_CS_DELAY;palSetLine(LINE_RX_SEL);}

#define SI_SDN_LOW      palClearLine(LINE_RX_SDN);
#define SI_SDN_HIGH     palSetLine(LINE_RX_SDN);

// Hardware or software SPI use
#ifdef USE_HARDWARE_SPI_MODE
#define SI4432_SPI         SPI1

// Check device SPI clock speed
#if STM32_PCLK2 > 48000000   // 48 or 72M MCU
// On 72M MCU STM32_PCLK2 = 72M, SPI = 72M/4 = 18M
//#define SI4432_SPI_SPEED       SPI_BR_DIV4
#define SI4432_SPI_SPEED       SPI_BR_DIV8
#else
// On 48M MCU STM32_PCLK2 = 48M, SPI = 48M/2 = 24M
//#define SI4432_SPI_SPEED       SPI_BR_DIV2
#define SI4432_SPI_SPEED       SPI_BR_DIV4
#endif


//#define ADF_SPI_SPEED   SPI_BR_DIV64
//#define ADF_SPI_SPEED   SPI_BR_DIV32
#define ADF_SPI_SPEED   SPI_BR_DIV2

#define PE4302_HW_SHIFT     true
#define PE_SPI_SPEED   SPI_BR_DIV8
#define PE_SW_DELAY     2

static uint32_t old_spi_settings;
#else
static uint32_t new_port_moder;
#endif
static uint32_t old_port_moder;

#define SPI1_CLK_HIGH   palSetPad(GPIOB, GPIOB_SPI_SCLK)
#define SPI1_CLK_LOW    palClearPad(GPIOB, GPIOB_SPI_SCLK)

#define SPI1_SDI_HIGH   palSetPad(GPIOB, GPIOB_SPI_MOSI)
#define SPI1_SDI_LOW    palClearPad(GPIOB, GPIOB_SPI_MOSI)
#define SPI1_RESET      palClearPort(GPIOB, (1<<GPIOB_SPI_SCLK)|(1<<GPIOB_SPI_MOSI))

#define SPI1_SDO       ((palReadPort(GPIOB)>>GPIOB_SPI_MISO)&1)
#define SPI1_portSDO   (palReadPort(GPIOB)&(1<<GPIOB_SPI_MISO))

#ifdef __PE4302__
#define CS_PE_HIGH      {PE_CS_DELAY;palSetLine(LINE_PE_SEL);}
#define CS_PE_LOW       {PE_CS_DELAY;palClearLine(LINE_PE_SEL);}
#endif

//#define MAXLOG 1024
//unsigned char SI4432_logging[MAXLOG];
//volatile int log_index = 0;

//#define SI4432_log(X)   { if (log_index < MAXLOG)  SI4432_logging[log_index++] = X; }
#define SI4432_log(X)

void start_SI4432_SPI_mode(void){
#ifdef USE_HARDWARE_SPI_MODE
  old_spi_settings = SI4432_SPI->CR1;
  SPI_BR_SET(SI4432_SPI, SI4432_SPI_SPEED);
#else
  // Init legs mode for software bitbang
  old_port_moder = GPIOB->MODER;
  new_port_moder = old_port_moder & ~(PIN_MODE_ANALOG(GPIOB_SPI_SCLK)|PIN_MODE_ANALOG(GPIOB_SPI_MISO)|PIN_MODE_ANALOG(GPIOB_SPI_MOSI));
  new_port_moder|= PIN_MODE_OUTPUT(GPIOB_SPI_SCLK)|PIN_MODE_INPUT(GPIOB_SPI_MISO)|PIN_MODE_OUTPUT(GPIOB_SPI_MOSI);
  GPIOB->MODER = new_port_moder;
  // Pull down SPI
  SPI1_SDI_LOW;
  SPI1_CLK_LOW;
#endif
}

void stop_SI4432_SPI_mode(void){
#ifdef USE_HARDWARE_SPI_MODE
  SI4432_SPI->CR1 = old_spi_settings;
#else
  // Restore hardware SPI
  GPIOB->MODER = old_port_moder;
#endif
}

void start_PE4312_SPI_mode(void){
  // Init legs mode for software bitbang
  old_spi_settings = SI4432_SPI->CR1;
  old_port_moder = GPIOB->MODER;
  uint32_t new_port_moder = old_port_moder & ~(PIN_MODE_ANALOG(GPIOB_SPI_SCLK)|PIN_MODE_ANALOG(GPIOB_SPI_MISO)|PIN_MODE_ANALOG(GPIOB_SPI_MOSI));
  new_port_moder|= PIN_MODE_OUTPUT(GPIOB_SPI_SCLK)|PIN_MODE_INPUT(GPIOB_SPI_MISO)|PIN_MODE_OUTPUT(GPIOB_SPI_MOSI);
  GPIOB->MODER = new_port_moder;
  // Pull down SPI
  SPI1_SDI_LOW;
  SPI1_CLK_LOW;
}

void stop_PE4312_SPI_mode(void){
  // Restore hardware SPI
  GPIOB->MODER = old_port_moder;
  SI4432_SPI->CR1 = old_spi_settings;
}

#if 0
static void software_shiftOut(uint8_t val)
{
  SI4432_log(SI4432_Sel);
  SI4432_log(val);
  uint8_t i = 0;
  do {
    if (val & 0x80)
      SPI1_SDI_HIGH;
    my_microsecond_delay(PE_SW_DELAY);
    SPI1_CLK_HIGH;
    my_microsecond_delay(PE_SW_DELAY);
    SPI1_RESET;
    val<<=1;
  }while((++i) & 0x07);
}
#endif

static void shiftOut(uint8_t val)
{
#ifdef USE_HARDWARE_SPI_MODE
  while (SPI_TX_IS_NOT_EMPTY(SI4432_SPI));
  SPI_WRITE_8BIT(SI4432_SPI, val);
  while (SPI_IS_BUSY(SI4432_SPI)) // drop rx and wait tx
    (void)SPI_READ_8BIT(SI4432_SPI);
#else
  SI4432_log(SI4432_Sel);
  SI4432_log(val);
  uint8_t i = 0;
  do {
    SPI1_SDI_HIGH;
    SPI1_CLK_HIGH;
    SPI1_RESET;
    val<<=1;
  }while((++i) & 0x07);
#endif
}

static uint8_t shiftIn(void)
{
#ifdef USE_HARDWARE_SPI_MODE
//  while (SPI_TX_IS_NOT_EMPTY(SI4432_SPI));
  SPI_WRITE_8BIT(SI4432_SPI, 0xFF);
  while (SPI_RX_IS_EMPTY(SI4432_SPI)) ; // drop rx and wait tx
  return SPI_READ_8BIT(SI4432_SPI);
#else
  uint32_t value = 0;
  uint8_t i = 0;
  do {
    value<<=1;
    SPI1_CLK_HIGH;
    value|=SPI1_portSDO;
    SPI1_CLK_LOW;
  }while((++i) & 0x07);
  return value>>GPIOB_SPI_MISO;
#endif
}

uint32_t SI4432_step_delay = 1500;
uint32_t SI4432_offset_delay = 1500;
#define MINIMUM_WAIT_FOR_RSSI   280


//------------PE4302 -----------------------------------------------
#ifdef __PE4302__
void PE4302_init(void) {
  CS_PE_LOW;
}

static unsigned char old_attenuation = 255;
bool PE4302_Write_Byte(unsigned char DATA )
{
//  if (old_attenuation == DATA)                /// Must always have same execution time
//    return false;
  old_attenuation = DATA;
#ifdef __ARW621__
  DATA = DATA << 1;
#endif
#if PE4302_HW_SHIFT
  set_SPI_mode(SPI_MODE_SI);
  if (SI4432_SPI_SPEED != PE_SPI_SPEED)
    SPI_BR_SET(SI4432_SPI, PE_SPI_SPEED);
  SPI_WRITE_8BIT(SI4432_SPI, DATA);
  while (SPI_IS_BUSY(SI4432_SPI)) {};
#else                               // Run PE4312 in SW mode to avoid disturbances
  set_SPI_mode(SPI_MODE_PE);
  software_shiftOut(DATA);
#endif
  CS_PE_HIGH;
  my_microsecond_delay(PE_SW_DELAY);
  CS_PE_LOW;
  my_microsecond_delay(PE_SW_DELAY);
#if PE4302_HW_SHIFT
  if (SI4432_SPI_SPEED != PE_SPI_SPEED)
    SPI_BR_SET(SI4432_SPI, SI4432_SPI_SPEED);
#endif
  return true;
}
#endif

//------------------------------- ADF4351 -------------------------------------

//#define SI5351_INITIAL_FREQ 3206896551
//#define SI5351_INITIAL_FREQ 15000000000ULL

#define NO_SHIFT_MUL 30
#define NO_SHIFT_DIV 30

#ifdef __SI5351__
#include "si5351.h"
static int shifted = 0;
//static int old_shifted = -1;
#define SHIFT_MUL   31
#define SHIFT_DIV   29

//#define SI5351_INITIAL_FREQ 3000000000ULL


void ADF4350_shift_ref(int f) {
  if (f == shifted)
    return;
  shifted = f;
  if (si5351_available && shifted)
    si5351_set_int_mul_div(0, SHIFT_MUL, SHIFT_DIV, 0);
  else
    si5351_set_int_mul_div(0, NO_SHIFT_MUL, NO_SHIFT_DIV, 0);
  ADF4351_recalculate_PFDRFout();
}
#else
int si5351_available = false;
#endif

#define __NEW_ADF4351__
#ifdef __NEW_ADF4351__

bool ADF4351_frequency_changed = false;

uint16_t R = 1;
int old_R = 0;
uint16_t N = 1;
uint16_t frac = 0;
uint16_t modulus = 200;
uint16_t out_div = 0;
uint16_t mux = 0;
uint16_t csr = 1;  // cycle slip reduction, if enabled cp must be lowest value
uint16_t bscm = 1; // Band select clock mode

//  uint32_t reg_0 = 0;
uint32_t reg_1 = 0;
uint32_t reg_2 = 0;
uint32_t reg_3 = 0;
uint32_t reg_4 = 0;
uint32_t reg_5 = 0;

uint16_t id = 0;
uint8_t rfPower = 0b00;
static freq_t prev_actual_freq = 0;
bool pdwn = false;
uint16_t powerDown = 0;


static const uint8_t auxPower = 0b11;

// band select divider. 1 to 255.
volatile uint16_t  bsDivider = 100; // For set internal logic clock (24M / 192 = 125k) max 125k

// charge pump current, 0 to 15.
static uint8_t cpCurrent = 0;       // Must be zero when using either CSR or Fast Lock

// CLKDIV divider (for fastlock and phase resync). 0 to 4095.
volatile uint16_t fastlockDivider = 6;      // Not used when in CSR mode

static const uint16_t phase = 1;

enum {
    CLKDIVMODE_OFF = 0b00,
    CLKDIVMODE_FASTLOCK = 0b01,
    CLKDIVMODE_RESYNC = 0b10
} clkDivMode = CLKDIVMODE_FASTLOCK;

static const enum {
    LD_LOW = 0b00,
    LD_LOCK_DETECT = 0b01,
    LD_HIGH = 0b11
} ld_pin = LD_LOCK_DETECT; // Used for led output

bool refDouble = false;
static const bool refDiv2 = false;
static const bool rfEnable = true;
static bool auxEnable = false;
static const bool feedbackFromDivided = true;

static const bool LDF = false;      // for fractional mode
static const bool LDP = true;       // 6 ns
static const bool LDS = true;

static enum {
    LD_LOW_NOISE = 0b00,
    LD_LOW_SPUR1 = 0b10,
    LD_LOW_SPUR2 = 0b11
} noiseMode = LD_LOW_SPUR1;

/*
static const enum {
    p_4_div_5 = 0,   // min integer 23, max freq = 3.0GHz
    p_8_div_9 = 1    // min integer 75, max freq = 4.4GHz
} prescaler = p_8_div_9;
*/
static const enum {
    CPt_NORMAL       = 0b00, // Work, use default
    CPt_LONG_RESET   = 0b01, // Work
    CPt_FORCE_SOURCE = 0b10,
    CPt_FORCE_SINK   = 0b11,
} CP_Test = CPt_NORMAL;

const enum {
    CPm_DISABLE = 0b00, // Default
    CPm_10pct   = 0b01,
    CPm_20pct   = 0b10,
    CPm_30pct   = 0b11, // ! Show best linearity result
} CP_Mode = CPm_DISABLE;

#define CS_ADF0_HIGH     {palSetLine(LINE_LO_SEL);ADF_CS_DELAY;}
#define CS_ADF0_LOW      {palClearLine(LINE_LO_SEL);ADF_CS_DELAY;}

bool ADF4351_dirty = false;

void ADF4351_WriteRegister32(int channel, const uint32_t value)
{
  (void) channel;
    // Select chip
    CS_ADF0_LOW;
    // Send 32 bit register
  #if 1
    SPI_WRITE_8BIT(SI4432_SPI, (value >> 24));
    SPI_WRITE_8BIT(SI4432_SPI, (value >> 16));
    SPI_WRITE_8BIT(SI4432_SPI, (value >>  8));
    SPI_WRITE_8BIT(SI4432_SPI, (value >>  0));
    ADF4351_dirty = true;
//    while (SPI_IS_BUSY(SI4432_SPI)); // drop rx and wait tx
  #else
    shiftOut((value >> 24) & 0xFF);
    shiftOut((value >> 16) & 0xFF);
    shiftOut((value >>  8) & 0xFF);
    shiftOut((value >>  0) & 0xFF);
  #endif
    // unselect
//    CS_ADF0_HIGH;
}

void ADF4351_Latch(void)
{
  if (ADF4351_dirty == false)
    return;
  while (SPI_IS_BUSY(SI4432_SPI)) {}; // drop rx and wait tx
  CS_ADF0_HIGH;
}

void sendConfig(void) {
  if (SI4432_SPI_SPEED != ADF_SPI_SPEED)
    SPI_BR_SET(SI4432_SPI, ADF_SPI_SPEED);
  if (max2871) {
//    pdwn = false; //Power down is no longer active.
    uint32_t reg;
    const bool fractional = false;
    const uint32_t phase = 1; // Recommended

//    const bool LDS = (SI5351_INITIAL_FREQ > 3200000000ULL ? true: false);

    // reg 5
    //        LD pin      register 5
    reg = (ld_pin<<22) | 0b101;
    if (reg!=reg_5) {ADF4351_WriteRegister32(id, reg); reg_5 = reg;}

    // reg 4
    //     bs devider             fb                          rf divider          bs divider               VCO down         mtld      aux sel    aux en           aux pwr         rf en           rf pwr        register 4
    reg = (bsDivider>>7) << 24 | (feedbackFromDivided<<23) | (out_div<<20)     | ((bsDivider&0x7f)<<12) | (powerDown<<11) | (0<<10) | (0<<9)  | (auxEnable<<8) | (auxPower<<6) | (rfEnable<<5) | (rfPower<<3) | 0b100;
    if (reg!=reg_4) {ADF4351_Latch(); ADF4351_WriteRegister32(id, reg); reg_4 = reg;}

    // reg 3
    //     bscm      |  csr        mutedel   clkdiv mode        clkdiv           register 3
    reg = (bscm<<23) | (csr<<18) | (0<<17) | (clkDivMode<<15) | (fastlockDivider<<3) | 0b011;
    if (reg!=reg_3) {ADF4351_Latch(); ADF4351_WriteRegister32(id, reg); reg_3 = reg;}

    // reg 2                                                                                                                                                 cp three     reset
    //     LD speed   noise mode         muxout         ref dbr            ref div2        R         DB        CP current        LDF       LDP      PD pol   powerdown    state    counter   register 2
    reg = (LDS<<31) | (LD_LOW_SPUR1<<29) | (mux<<26) | (refDouble<<25) | (refDiv2 << 24) | (R<<14) | (0<<13) | (cpCurrent<<9) | (LDF<<8) | (LDP<<7) | (1<<6) | (pdwn<<5)  | (0<<4) |  (0<<3) |   0b010;
    if (reg!=reg_2) {ADF4351_Latch(); ADF4351_WriteRegister32(id, reg); reg_2 = reg;}

    // reg 1
    //      CP mode        CP test          phase       frac modulus
    reg = (CP_Mode<<29) | (CP_Test<<27) | (phase<<15) | (modulus<<3) | 0b001;
    if (reg!=reg_1) {ADF4351_Latch(); ADF4351_WriteRegister32(id, reg); reg_1 = reg;}

    // reg 0 (need always send for apply some reg 1 - 5 settings
    reg = (fractional<<31) | (N<<15) | (frac<<3) | 0b000;
    /*if (reg!=reg_0)*/ {ADF4351_Latch(); ADF4351_WriteRegister32(id, reg);/* reg_0 = reg;*/}
    ADF4351_Latch();
  } else {
//    pdwn = false; //Power down is no longer active.
    uint32_t reg;
#ifdef BOARD_DOUBLE_REF_MODE
    uint32_t prescaler = (N > 75) ? 1 : 0;
#else
    uint32_t prescaler = 1;
#endif
    // reg 5
    //        LD pin      register 5
    reg = (ld_pin<<22) | (0b11<<19)| 0b101;
    if (reg!=reg_5) {ADF4351_WriteRegister32(id, reg); reg_5 = reg;}

    // reg 4
    //     fb                          rf divider          bs divider       VCO down  mtld      aux sel    aux en           aux pwr         rf en           rf pwr        register 4
    reg = (feedbackFromDivided<<23) | (out_div<<20)     | (bsDivider<<12) | (0<<11) | (0<<10) | (0<<9)  | (auxEnable<<8) | (auxPower<<6) | (rfEnable<<5) | (rfPower<<3) | 0b100;
    if (reg!=reg_4) { ADF4351_Latch(); ADF4351_WriteRegister32(id, reg); reg_4 = reg;}

    // reg 3
    //     csr       clkdiv mode             clkdiv         register 3
    reg = (csr<<18) | (clkDivMode<<15) | (fastlockDivider<<3) | 0b011;
    if (reg!=reg_3) {ADF4351_Latch(); ADF4351_WriteRegister32(id, reg); reg_3 = reg;}

    // reg 2                                                                                                                                                 cp three    reset
    //     noise mode        muxout         ref dbr            ref div2        R         DB        CP current        LDF       LDP       PD pol   powerdown    state    counter   register 2
    reg = (LD_LOW_NOISE<<29) | (mux<<26) | (refDouble<<25) | (refDiv2 << 24) | (R<<14) | (0<<13) | (cpCurrent<<9) | (LDF<<8) | (LDP<<7) | (1<<6) | (pdwn<<5)  | (0<<4) |  (0<<3) |   0b010;
    if (reg!=reg_2) {ADF4351_Latch(); ADF4351_WriteRegister32(id, reg); reg_2 = reg;}

    // reg 1
    //     prescaler         phase        frac modulus
    reg = (prescaler<<27) | (phase<<15) | (modulus<<3) | 0b001;
    if (reg!=reg_1) {ADF4351_Latch(); ADF4351_WriteRegister32(id, reg); reg_1 = reg;}

    // reg 0 (need always send for apply some reg 1 - 5 settings
    reg = (N<<15) | (frac<<3) | 0b000;
    /*if (reg!=reg_0)*/ {ADF4351_Latch(); ADF4351_WriteRegister32(id, reg);/* reg_0 = reg;*/}
    ADF4351_Latch();
  }
  if (SI4432_SPI_SPEED != ADF_SPI_SPEED)
    SPI_BR_SET(SI4432_SPI, SI4432_SPI_SPEED);
}


void sendPowerdown(bool p) {
    if(pdwn == p)
        return;
    pdwn = p;
    uint32_t reg = (noiseMode<<29) | (0b001<<26) | (refDouble<<25) | (refDiv2 << 24) | (R<<14) | (cpCurrent<<9) | (0<<8) | (0<<7) | (1<<6) | (pdwn<<5) | 0b010;
    if (reg!=reg_2) {ADF4351_WriteRegister32(id, reg); reg_2 = reg;ADF4351_Latch(); }
}


static uint32_t adf4350_get_O(uint64_t freqHz) {
  if(max2871) {
    if(freqHz > 3000000000)  return 0; //  1
    else  if(freqHz > 1500000000)  return 1; //  2
    else  if(freqHz >  750000000)  return 2; //  4
    else  if(freqHz >  375000000)  return 3; //  8
    else  if(freqHz >  187500000)  return 4; // 16
    else  if(freqHz >  137500000)  return 5; // 32
    else  if(freqHz >   68750000)  return 6; // 64
    else/*if(freqHz >   34375000)*/return 7; //128
  }else{
    if(freqHz > 2200000000)  return 0; //  1
    else  if(freqHz > 1100000000)  return 1; //  2
    else  if(freqHz >  550000000)  return 2; //  4
    else  if(freqHz >  275000000)  return 3; //  8
    else/*if(freqHz >  137500000)*/return 4; // 16
  }
}

freq_t xtal;

uint64_t ADF4351_set_frequency(int channel, uint64_t freqHz) {
  (void) channel;
  // RFout = xtalFreqHz � (N + FRAC/MOD) = xtalFreqHz � (N * MOD + FRAC) / MOD
  // step = xtalFreqHz / MOD; !!!! should get integer result, also this result should divided by 16
  // for 24M step = 24M / 4000 = 6k and 6k/16 = 375
  // Nx = RFout / step
  // N * 4000 + frac = Nx
  // N    = Nx / 4000
  // frac = Nx % 4000
#if 1
  out_div = adf4350_get_O(freqHz);
#ifdef __SI5351__
//  if (shifted != old_shifted || xtal == 0)
  {
//    old_shifted = shifted;
    if (shifted)
      xtal = (config.setting_frequency_30mhz * SHIFT_MUL) / SHIFT_DIV;
    else
    {
#endif
     xtal = config.setting_frequency_30mhz; // * NO_SHIFT_MUL)/ NO_SHIFT_DIV;
#ifdef __SI5351__
    }
  }
#endif
  if (refDouble) {
    xtal<<=1;
  }
  if (R > 1)
    xtal /= R;

#if 1
  uint32_t modulus_x2 = modulus<<1;
  uint32_t INTA_F = (((freqHz << out_div) * (uint64_t)(modulus_x2*FREQ_MULTIPLIER))/ xtal) + 1;
  N = INTA_F / modulus_x2;
  frac = (INTA_F - N * modulus_x2)>>1;
  if (frac >= modulus) {
    frac -= modulus;
    N++;
  }

  freq_t actual_freq = (((uint64_t)xtal *(uint64_t)(N * modulus +frac)) >> out_div) / (modulus*FREQ_MULTIPLIER);
#else

  uint32_t _N = ((freqHz<<out_div) * (uint64_t)modulus) / xtal;

  N    = _N / modulus;
  frac = _N % modulus;
#endif

#else
  uint32_t MOD = ADF4350_modulo;
  if (MOD == 0)
    MOD = 60;
  uint32_t MOD_X2 = MOD<<1;
  uint32_t INTA_F = ((freq * (uint64_t)output_divider) * (uint64_t)MOD_X2/ PFDR) + 1;
  uint32_t INTA = INTA_F / MOD_X2;
  uint32_t FRAC = (INTA_F - INTA * MOD_X2)>>1;
  if (FRAC >= MOD) {
    FRAC -= MOD;
    INTA++;
  }

  freq_t actual_freq = ((uint64_t)xtal *(N * modulus +frac))/ (1<<out_div) / modulus;
#endif


  if (prev_actual_freq == actual_freq)
    return prev_actual_freq;
  prev_actual_freq = actual_freq;

  ADF4351_frequency_changed = true;
  sendConfig();
  return actual_freq;
}

void ADF4351_force_refresh(void) {
  prev_actual_freq = 0;
//  reg_0 = 0;
  reg_1 = 0;
  reg_2 = 0;
  reg_3 = 0;
  reg_4 = 0;
}

void ADF4351_modulo(int m)
{
  modulus = m;
}

uint16_t ADF4351_get_modulo(void)
{
  return modulus;
}

void ADF4351_spur_mode(int S)
{
  (void) S;
//  noiseMode = S;
}

void ADF4351_R_counter(int new_R)
{
  if (new_R == old_R)
    return;
  old_R = new_R;
  refDouble = false;
  if (new_R < 0) {
    refDouble = true;
    new_R = -new_R;
  }
  if (new_R<1)
    return;
  R = new_R;
  if (R>1)
    setting.increased_R = true;
  else
    setting.increased_R = false;
  clear_frequency_cache();                              // When R changes the possible frequencies will change
}


void ADF4351_mux(int m)
{
  mux = m;
  sendConfig();
}

void ADF4351_csr(int c)
{
  csr = (c & 0x1);
  sendConfig();
}

void ADF4351_fastlock(int c)
{
  clkDivMode = (c & 0x3);
  sendConfig();
}

void ADF4351_CP(int p)
{
  if (cpCurrent == p)
    return;
  cpCurrent = p;
  sendConfig();
}

uint16_t ADF4351_get_CP(void)
{
  return cpCurrent;
}

void ADF4351_drive(int p)
{
  if (rfPower == p)
    return;
  rfPower = p;
  sendConfig();
  my_microsecond_delay(1000);
}
#if 0
void ADF4351_aux_drive(int p)
{
  if ( auxPower == p)
    return;
  auxPower = p;
  sendConfig();
}
#endif

void ADF4351_enable_aux_out(int s)
{
  if ( auxEnable == s)
    return;
  auxEnable = s;
  sendConfig();
}


void ADF4351_enable(int s)
{
  if ( powerDown == !s)
    return;
  powerDown = !s;
  sendConfig();
  osalThreadSleepMilliseconds(10);

}

void ADF4351_enable_out(int s)
{
  if ( pdwn == !s)
     return;
  powerDown = !s;
  pdwn = !s;
  sendConfig();
  osalThreadSleepMilliseconds(10);
}

void ADF4351_recalculate_PFDRFout(void) {
  int local_r = old_R;
  old_R = -1;
  ADF4351_R_counter(local_r);
  sendConfig();
}

void ADF4351_Setup(void)
{
  CS_ADF0_HIGH;

#ifdef __SI5351__
  si5351_available = si5351_init();
  if (si5351_available) {
    si5351_set_frequency(0, (config.setting_frequency_30mhz * NO_SHIFT_MUL)/ NO_SHIFT_DIV /100, 0);
    si5351_set_int_mul_div(0, NO_SHIFT_MUL, NO_SHIFT_DIV, 0);
  }
  si5351_available = false; // Don't use shifting
#endif

  cpCurrent = 0;
 if (max2871) {
//    refDouble = true;
 } else {
    ADF4351_csr(1);           //Cycle slip enabled
    ADF4351_fastlock(1);      // Fastlock enabled
    cpCurrent = 0;
  }
//  R = 1;
  ADF4351_set_frequency(0,3000000000);
  ADF4351_mux(0);   // Tristate
}

#else

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))

#define maskedWrite(reg, bit, mask, value)   (reg) &= ~(((uint32_t)mask) << (bit)); (reg) |=  ((((uint32_t) (value)) & ((uint32_t)mask)) << (bit));

#define CS_ADF0_HIGH     {palSetLine(LINE_LO_SEL);ADF_CS_DELAY;}
#define CS_ADF1_HIGH     {ADF_CS_DELAY;palSetLine(LINE_LO_SEL);}

#define CS_ADF0_LOW      {palClearLine(LINE_LO_SEL);ADF_CS_DELAY;}
#define CS_ADF1_LOW      {ADF_CS_DELAY;palClearLine(LINE_LO_SEL);}

#define CS_ADF_LOW(ch)   {palClearLine(ch);ADF_CS_DELAY;}
#define CS_ADF_HIGH(ch)  {ADF_CS_DELAY;palSetLine(ch);}

uint32_t registers[6] =  {0xC88000, 0x8008011, 0x1800C642, 0x48963,0xA5003C , 0x580005} ;         //10 MHz ref
uint32_t old_registers[6];

int debug = 0;
ioline_t ADF4351_LE[2] = { LINE_LO_SEL, LINE_LO_SEL};
//int ADF4351_Mux = 7;

bool ADF4351_frequency_changed = false;

//#define DEBUG(X) // Serial.print( X )
//#define DEBUGLN(X) Serial.println( X )
//#define DEBUGFLN(X,Y) Serial.println( X,Y )
//#define DEBUGF(X,Y) Serial.print( X,Y )
#define DEBUG(X)
#define DEBUGLN(X)

#define XTAL    300000000
uint64_t  PFDRFout[6] = {XTAL,XTAL,XTAL,10000000,10000000,10000000}; //Reference freq in MHz

int64_t
  ADF4350_modulo = 0,          // Linked to spur table!!!!!
  target_freq;

int old_R = 0;


void ADF4351_Setup(void)
{
//  palSetPadMode(GPIOA, 1, PAL_MODE_OUTPUT_PUSHPULL );
//  palSetPadMode(GPIOA, 2, PAL_MODE_OUTPUT_PUSHPULL );


  local_setting_frequency_30mhz_x100 = config.setting_frequency_30mhz;
#ifdef __SI5351__
  si5351_available = si5351_init();
  if (si5351_available)
    si5351_set_frequency(0, 30000000, 0);
  si5351_available = false; // Don't use shifting
#endif

//  SPI3_CLK_HIGH;
//  SPI3_SDI_HIGH;
  CS_ADF0_HIGH;
//  CS_ADF1_HIGH;

  //  bitSet (registers[2], 17); // R set to 8
//  bitClear (registers[2], 14); // R set to 8

//  while(1) {
//

  ADF4351_R_counter(1);

  ADF4351_CP(0);

  ADF4351_fastlock(1);      // Fastlock enabled
  ADF4351_csr(1);           //Cycle slip enabled

  ADF4351_set_frequency(0,200000000);

  ADF4351_mux(0);   // Tristate
//    ADF4351_mux(6);   // Show lock on led

}

void ADF4351_WriteRegister32(int channel, const uint32_t value)
{
  // Select chip
  CS_ADF_LOW(ADF4351_LE[channel]);
  // Send 32 bit register
#if 1
  SPI_WRITE_8BIT(SI4432_SPI, (value >> 24));
  SPI_WRITE_8BIT(SI4432_SPI, (value >> 16));
  SPI_WRITE_8BIT(SI4432_SPI, (value >>  8));
  SPI_WRITE_8BIT(SI4432_SPI, (value >>  0));
  while (SPI_IS_BUSY(SI4432_SPI)); // drop rx and wait tx
#else
  shiftOut((value >> 24) & 0xFF);
  shiftOut((value >> 16) & 0xFF);
  shiftOut((value >>  8) & 0xFF);
  shiftOut((value >>  0) & 0xFF);
#endif
  // unselect
  CS_ADF_HIGH(ADF4351_LE[channel]);
}

void ADF4351_Set(int channel)
{
#if 0
  for (int i = 5; i >= 0; i--) {
    if (registers[i] != old_registers[i])
      goto update;
  }
  return;
update:
#endif
  set_SPI_mode(SPI_MODE_SI);
  if (SI4432_SPI_SPEED != ADF_SPI_SPEED)
    SPI_BR_SET(SI4432_SPI, ADF_SPI_SPEED);
  for (int i = 5; i >= 0; i--) {
#if 0
    if (i == 0 || registers[i] != old_registers[i])
#endif
      ADF4351_WriteRegister32(channel, registers[i]);
    old_registers[i] = registers[i];

  }
  if (SI4432_SPI_SPEED != ADF_SPI_SPEED)
    SPI_BR_SET(SI4432_SPI, SI4432_SPI_SPEED);
}

static freq_t prev_actual_freq = 0;

void ADF4351_force_refresh(void) {
  prev_actual_freq = 0;
  for (int i = 5; i >= 0; i--)
    old_registers[i] = 0;
}

void ADF4351_modulo(int m)
{
  ADF4350_modulo = m;
//  ADF4351_set_frequency(0, (uint64_t)prev_actual_freq);
}

uint64_t ADF4351_set_frequency(int channel, uint64_t freq)  // freq / 10Hz
{
  uint64_t actual_freq = ADF4351_prepare_frequency(channel,freq);
  if (actual_freq != prev_actual_freq) {
    ADF4351_frequency_changed = true;
    ADF4351_Set(channel);
    prev_actual_freq = actual_freq;
  }
  return actual_freq;
}

void ADF4351_spur_mode(int S)
{
  bitWrite(registers[2], 29, S & 1);
  bitWrite(registers[2], 30, S & 2);
  ADF4351_Set(0);
}

void ADF4351_R_counter(int R)
{
  if (R == old_R)
    return;
  old_R = R;
  int dbl = false;
  if (R < 0) {
    dbl = true;
    R = -R;
  }
  if (R<1)
    return;
  bitWrite(registers[2], 25, dbl); // Reference doubler
  for (int channel=0; channel < 6; channel++) {
    PFDRFout[channel] = (local_setting_frequency_30mhz_x100 * (dbl?2:1)) / R;
  }
  maskedWrite(registers[2],14, 0x3FF, R);
//   ADF4351_Set(0);            // Let next frequency set do the writing
//  ADF4351_force_refresh();
  clear_frequency_cache();                              // When R changes the possible frequencies will change
}

void ADF4351_recalculate_PFDRFout(void){
  int local_r = old_R;
  old_R = -1;
  local_setting_frequency_30mhz_x100 = config.setting_frequency_30mhz;
  ADF4351_R_counter(local_r);
}

void ADF4351_mux(int R)
{
  maskedWrite(registers[2],26, 0x7, R);
//  registers[2] &= ~(((uint32_t)     0x7) << 26);
//  registers[2] |=  (((uint32_t)R & 0x07) << 26);
  ADF4351_Set(0);
}

void ADF4351_csr(int c)
{
  maskedWrite(registers[3],18, 0x1, c);
//  registers[3] &= ~(((uint32_t)     0x1) << 18);
//  registers[3] |=  (((uint32_t)c & 0x01) << 18);
  ADF4351_Set(0);
}

void ADF4351_fastlock(int c)
{
  maskedWrite(registers[3],15, 0x3, c);
//  registers[3] &= ~(((uint32_t)     0x3) << 15);
//  registers[3] |=  (((uint32_t)c & 0x03) << 15);
  ADF4351_Set(0);
}

void ADF4351_CP(int p)
{
  maskedWrite(registers[2],9, 0xF, p);
//  registers[2] &= ~(((uint32_t)0xF) << 9);
//  registers[2] |=  (((uint32_t)  p) << 9);
  ADF4351_Set(0);
}

void ADF4351_drive(int p)
{
  if (((registers[4] >> 3) & 0x03 ) == (p & 0x03))
    return;
  maskedWrite(registers[4],3, 0x3, p);
//  p &= 0x03;
//  registers[4] &= ~(((uint32_t)0x3) << 3);
//  registers[4] |=  (((uint32_t)  p) << 3);
  ADF4351_Set(0);
  my_microsecond_delay(1000);

}

void ADF4351_aux_drive(int p)
{
  if (((registers[4] >> 6) & 0x03 ) == (p & 0x03))
    return;
  maskedWrite(registers[4],6, 0x3, p);
//  p &= 0x03;
//  registers[4] &= ~(((uint32_t)0x3) << 6);
//  registers[4] |=  (((uint32_t)  p) << 6);
  ADF4351_Set(0);
}
#if 0
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
#endif




uint64_t ADF4351_prepare_frequency(int channel, uint64_t freq)  // freq / 10Hz
{
  uint32_t output_divider;
  target_freq = freq;
  if (max2871) {
    if (freq >= 3000000000) {
      output_divider = 1 * FREQ_MULTIPLIER;
      bitWrite (registers[4], 22, 0);   // 0
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 0);
    } else if (freq >= 1500000000) {
      output_divider = 2 * FREQ_MULTIPLIER;
      bitWrite (registers[4], 22, 0);   // 1
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 1);
    } else if (freq >= 750000000) {
      output_divider = 4 * FREQ_MULTIPLIER;
      bitWrite (registers[4], 22, 0);   // 2
      bitWrite (registers[4], 21, 1);
      bitWrite (registers[4], 20, 0);
    } else if (freq >= 375000000)  {
      output_divider = 8 * FREQ_MULTIPLIER;
      bitWrite (registers[4], 22, 0);   // 3
      bitWrite (registers[4], 21, 1);
      bitWrite (registers[4], 20, 1);
    } else if (freq >= 187500000)  {
      output_divider = 16 * FREQ_MULTIPLIER;
      bitWrite (registers[4], 22, 1);   // 4
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 0);
    } else if (freq >= 137500000)  {
      output_divider = 32 * FREQ_MULTIPLIER;
      bitWrite (registers[4], 22, 1);   // 5
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 1);
    } else if (freq >= 68750000)  {
      output_divider = 64 * FREQ_MULTIPLIER;
      bitWrite (registers[4], 22, 1);   // 6
      bitWrite (registers[4], 21, 1);
      bitWrite (registers[4], 20, 0);
    } else {                        // > 34375000
      output_divider = 128 * FREQ_MULTIPLIER;
      bitWrite (registers[4], 22, 1);   // 7
      bitWrite (registers[4], 21, 1);
      bitWrite (registers[4], 20, 1);
    }
  } else {
    if (freq >= 2200000000) {
      output_divider = 1 * FREQ_MULTIPLIER;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 0);
    } else if (freq >= 1100000000) {
      output_divider = 2 * FREQ_MULTIPLIER;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 1);
    } else if (freq >= 550000000) {
      output_divider = 4 * FREQ_MULTIPLIER;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 1);
      bitWrite (registers[4], 20, 0);
    } else if (freq >= 275000000)  {
      output_divider = 8 * FREQ_MULTIPLIER;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 1);
      bitWrite (registers[4], 20, 1);
    } else {                        // > 137500000
      output_divider = 16 * FREQ_MULTIPLIER;
      bitWrite (registers[4], 22, 1);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 0);
    }
  }
     uint32_t PFDR = (uint32_t)PFDRFout[channel];
    uint32_t MOD = ADF4350_modulo;
    if (MOD == 0)
      MOD = 60;
    uint32_t MOD_X2 = MOD<<1;
    uint32_t INTA_F = ((freq * (uint64_t)output_divider) * (uint64_t)MOD_X2/ PFDR) + 1;
    uint32_t INTA = INTA_F / MOD_X2;
    uint32_t FRAC = (INTA_F - INTA * MOD_X2)>>1;
    if (FRAC >= MOD) {
      FRAC -= MOD;
      INTA++;
    }
#if 0   // No visible performance improvement
    uint32_t reduce = gcd(MOD, FRAC);
    if (reduce>1) {
      FRAC /= reduce;
      MOD /= reduce;
      if (MOD == 1)
        MOD=2;
    }
#endif
    uint64_t actual_freq = ((uint64_t)PFDR *(INTA * MOD +FRAC))/output_divider / MOD;

#if 0       // Only for debugging
     int max_delta =  PFDRFout[channel]/output_divider/MOD/100;
    if (actual_freq < freq - max_delta || actual_freq > freq + max_delta ){
       while(1)
         my_microsecond_delay(10);
    }
    max_delta = freq - actual_freq;
    if (max_delta > 200000 || max_delta < -200000 || freq == 0) {
      while(1)
        my_microsecond_delay(10);
    }
    if (FRAC >= MOD ){
      while(1)
        my_microsecond_delay(10);
    }
#endif

    bitWrite (registers[4], 10, 1);     // Mute till lock detect

    registers[0] = 0;
    registers[0] = INTA << 15; // OK
    registers[0] = registers[0] + (FRAC << 3);
    if (MOD == 1) MOD = 2;
    registers[1] = 0;
    registers[1] = MOD << 3;
    registers[1] |= 1 ; // restore register address "001"
    registers[1] |= 1 << 15;  // Set PHASE to 1
    bitSet (registers[1], 27); // Prescaler at 8/9
    return actual_freq;
}

void ADF4351_enable(int s)
{
  if (bitRead(registers[4],11) != (s & 0x01))
    return;
  if (s)
    bitClear(registers[4], 11);     // Inverse logic!!!!!
  else
    bitSet(registers[4], 11);
  ADF4351_Set(0);
  osalThreadSleepMilliseconds(10);

}

void ADF4351_enable_aux_out(int s)
{
  if (bitRead(registers[4],8) == (s & 0x01))
    return;
  if (s) {
    bitSet(registers[4], 8);
    maskedWrite(registers[4],6, 0x3, 3); // Max drive aux out
  } else
    bitClear(registers[4], 8);
  ADF4351_Set(0);
  osalThreadSleepMilliseconds(10);
}

void ADF4351_enable_out(int s)
{
  if (bitRead(registers[4],5) == (s & 0x01))
    return;
  if (s) {
    bitClear(registers[4], 11);     // Disable VCO power down
    bitClear(registers[2], 5);      // Disable power down
    bitSet(registers[4], 5);        // Enable output
  } else {
    bitClear(registers[4], 5);      // Disable output
    bitSet(registers[2], 5);        // Enable power down
    bitSet(registers[4], 11);        // Enable VCO power down
  }
  ADF4351_Set(0);
  osalThreadSleepMilliseconds(1);
}

#endif

// ------------------------------ SI4468 -------------------------------------


bool SI4463_frequency_changed = false;
bool SI4463_offset_changed = false;
int SI4463_offset_value = 0;
uint8_t SI4463_rbw_selected = 0;

static int SI4463_band = -1;
//static freq_t SI4463_prev_freq = 0;
//static float SI4463_step_size = 100;        // Will be recalculated once used
static uint8_t SI4463_channel = 0;
static uint8_t SI4463_in_tx_mode = false;
//int SI4463_R = 5;
static int SI4463_output_level = 0x20;

static si446x_state_t SI4463_get_state(void);
static int SI4463_set_state(si446x_state_t);

#define SI4463_READ_CTS       (palReadLine(LINE_RX_CTS))
#define SI4463_CTS_TIMEOUT  1000000
#ifdef __WAIT_CTS_WHILE_SLEEPING__
extern volatile int sleep;
#if 0
#define SI4463_WAIT_CTS       while (!SI4463_READ_CTS) {\
  if (sleep) {\
    CS_PE_HIGH;\
    __WFI();\
    CS_PE_LOW;\
  } \
  };
#else

int SI4463_wait_CTS(void) {
  int t=0;
  while (!SI4463_READ_CTS) {
    t++;
    if (t >=SI4463_CTS_TIMEOUT)
      return -1;
  }
  return 0;
}

#define SI4463_WAIT_CTS  SI4463_wait_CTS()

//#define SI4463_WAIT_CTS      {int t=0; while (!SI4463_READ_CTS) { t++; if (t >=SI4463_CTS_TIMEOUT) ili9341_drawstring("SI4486 deadlock", 0,20);} }
#endif
#else
#define SI4463_WAIT_CTS       while (!SI4463_READ_CTS) ;
#endif

#if 0   // not used
static void SI4463_write_byte(uint8_t ADR, uint8_t DATA)
{
  set_SPI_mode(SPI_MODE_SI);
  SI_CS_LOW;
  ADR |= 0x80 ; // RW = 1
  shiftOut( ADR );
  shiftOut( DATA );
  SI_CS_HIGH;
}

static void SI4463_write_buffer(uint8_t ADR, uint8_t *DATA, int len)
{
  set_SPI_mode(SPI_MODE_SI);
  SI_CS_LOW;
  ADR |= 0x80 ; // RW = 1
  shiftOut( ADR );
  while (len-- > 0)
    shiftOut( *(DATA++) );
  SI_CS_HIGH;
}
#endif

static uint8_t SI4463_read_byte( uint8_t ADR )
{
  uint8_t DATA ;
  set_SPI_mode(SPI_MODE_SI);
  SI_CS_LOW;
  shiftOut( ADR );
  DATA = shiftIn();
  SI_CS_HIGH;
  return DATA ;
}

#ifdef NOTUSED
static uint8_t SI4463_get_response(void* buff, uint8_t len)
{
    uint8_t cts = 0;
//    set_SPI_mode(SPI_MODE_SI);
    cts = SI4463_READ_CTS;
    if (!cts) {
      return false;
    }
//    __disable_irq();
    SI_CS_LOW;
    shiftOut( SI446X_CMD_READ_CMD_BUFF );
    cts = (shiftIn() == 0xFF);
    if (cts)
    {
        // Get response data
        for(uint8_t i=0;i<len;i++) {
            ((uint8_t*)buff)[i] = shiftIn();
        }
    }
    SI_CS_HIGH;
//    __enable_irq();
    return cts;
}
#endif


void SI4463_do_first_api(void* data, uint8_t len, void* out, uint8_t outLen)
{
  uint8_t *ptr = (uint8_t *)data;
  set_SPI_mode(SPI_MODE_SI);
//  SI4463_WAIT_CTS;         // Wait for CTS
  SI_CS_LOW;
#if 1                                               // Inline transfer
  while (len--){
    while (SPI_TX_IS_NOT_EMPTY(SI4432_SPI));
    SPI_WRITE_8BIT(SI4432_SPI, *ptr++);
  }
  while (SPI_IS_BUSY(SI4432_SPI)) {};
#else
  while (len--)
    shiftOut(*ptr++); // (pgm_read_byte(&((uint8_t*)data)[i]));
#endif
  SI_CS_HIGH;

  if(out == NULL) return; // If we have an output buffer then read command response into it

  while (SPI_RX_IS_NOT_EMPTY(SI4432_SPI))
    (void)SPI_READ_8BIT(SI4432_SPI);      // Remove lingering bytes from SPI RX buffer
  SI4463_WAIT_CTS;                        // Wait for CTS

  SI_CS_LOW;
#if 1
  SPI_WRITE_8BIT(SI4432_SPI, SI446X_CMD_READ_CMD_BUFF);
  SPI_WRITE_8BIT(SI4432_SPI, 0xFF);
  while (SPI_IS_BUSY(SI4432_SPI));
  SPI_READ_16BIT(SI4432_SPI);        // drop SI446X_CMD_READ_CMD_BUFF and CTS 0xFF
#else
  shiftOut( SI446X_CMD_READ_CMD_BUFF );
  shiftIn();                         // Should always be 0xFF
#endif
  // Get response data
  ptr = (uint8_t *)out;
  while (outLen--){
#if 1                                               // Inline transfer
    SPI_WRITE_8BIT(SI4432_SPI, 0xFF);
    while (SPI_RX_IS_EMPTY(SI4432_SPI)); //wait rx data in buffer
    *ptr++ = SPI_READ_8BIT(SI4432_SPI);
#else
    *ptr++ = shiftIn();
#endif
  }
  SI_CS_HIGH;
}


int SI4463_do_api(void* data, uint8_t len, void* out, uint8_t outLen)
{
  uint8_t *ptr = (uint8_t *)data;
  set_SPI_mode(SPI_MODE_SI);
  if (SI4463_WAIT_CTS)  return -1;         // Wait for CTS
  SI_CS_LOW;
#if 1                                               // Inline transfer
  while (len--){
    while (SPI_TX_IS_NOT_EMPTY(SI4432_SPI));
    SPI_WRITE_8BIT(SI4432_SPI, *ptr++);
  }
  while (SPI_IS_BUSY(SI4432_SPI)) {};
#else
  while (len--)
    shiftOut(*ptr++); // (pgm_read_byte(&((uint8_t*)data)[i]));
#endif
  SI_CS_HIGH;

  if(out == NULL) return 0; // If we have an output buffer then read command response into it

  while (SPI_RX_IS_NOT_EMPTY(SI4432_SPI))
    (void)SPI_READ_8BIT(SI4432_SPI);      // Remove lingering bytes from SPI RX buffer
  if (SI4463_WAIT_CTS)  return -1;         // Wait for CTS

  SI_CS_LOW;
#if 1
  SPI_WRITE_8BIT(SI4432_SPI, SI446X_CMD_READ_CMD_BUFF);
  SPI_WRITE_8BIT(SI4432_SPI, 0xFF);
  while (SPI_IS_BUSY(SI4432_SPI));
  SPI_READ_16BIT(SI4432_SPI);        // drop SI446X_CMD_READ_CMD_BUFF and CTS 0xFF
#else
  shiftOut( SI446X_CMD_READ_CMD_BUFF );
  shiftIn();                         // Should always be 0xFF
#endif
  // Get response data
  ptr = (uint8_t *)out;
  while (outLen--){
#if 1                                               // Inline transfer
    SPI_WRITE_8BIT(SI4432_SPI, 0xFF);
    while (SPI_RX_IS_EMPTY(SI4432_SPI)); //wait rx data in buffer
    *ptr++ = SPI_READ_8BIT(SI4432_SPI);
#else
    *ptr++ = shiftIn();
#endif
  }
  SI_CS_HIGH;
  return 0;
}

#ifdef notused
static void SI4463_set_properties(uint16_t prop, void* values, uint8_t len)
{
    // len must not be greater than 12

    uint8_t data[16] = {
        SI446X_CMD_SET_PROPERTY,
        (uint8_t)(prop>>8),
        len,
        (uint8_t)prop
    };

    // Copy values into data, starting at index 4
    memcpy(data + 4, values, len);

    SI4463_do_api(data, len + 4, NULL, 0);
}
#endif

#include "si446x_cmd.h"

#include "radio_config_Si4468_undef.h"
#undef RADIO_CONFIGURATION_DATA_ARRAY
#include "radio_config_Si4468_default.h"


// Used in RBW setting
#define GLOBAL_GPIO_PIN_CFG 0x13, 0x07, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00
#define GLOBAL_CLK_CFG 0x11, 0x00, 0x01, 0x01, 0x00
// ---------------------------------------------------------------------------------------------------- v ------------  RSSI control byte
#define GLOBAL_RF_MODEM_RAW_CONTROL 0x11, 0x20, 0x0A, 0x45, 0x03, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x03, 0x10, 0x40
//0x11 SI446X_CMD_SET_PROPERTY
//0x20  SI446X_PROP_GROUP_MODEM
//0x0A  10 Count
//0x45  Start register
//0x03  [0x45] MODEM_RAW_CONTROL
//0x00  [0x46] RAWEYE[10:8]
//0x00  [0x47] RAWEYE[7:0]
//0x01  [0x48] MODEM_ANT_DIV_MODE
//0x00  [0x49] MODEM_ANT_DIV_CONTROL
//0xFF  [0x4A] MODEM_RSSI_THRESH
//0x06  [0x4B] MODEM_RSSI_JUMP_THRESH
//0x18  [0x4C] MODEM_RSSI_CONTROL
//0x10  [0x4D] MODEM_RSSI_CONTROL2
//0x40  [0x4E] MODEM_RSSI_COMP
// -----------------------------------------------------------------------------------------------------^ --------------
#define GLOBAL_RF_MODEM_AGC_CONTROL 0x11, 0x20, 0x01, 0x35, 0xF1             // Override AGC gain increase


#undef RF_MODEM_AGC_CONTROL_1
#define RF_MODEM_AGC_CONTROL_1 GLOBAL_RF_MODEM_AGC_CONTROL

#undef RF_MODEM_AGC_WINDOW_SIZE_12_1
//#define RF_MODEM_AGC_WINDOW_SIZE_12_1 0x11, 0x20, 0x0C, 0x38, 0x11, 0x07, 0x07, 0x80, 0x02, 0x4C, 0xCD, 0x00, 0x27, 0x0C, 0x84, 0x23
#define RF_MODEM_AGC_WINDOW_SIZE_12_1 0x11, 0x20, 0x0C, 0x38, 0x11, 0x07, 0x07, 0x80, 0x1C, 0x4C, 0xCD, 0x00, 0x27, 0x0C, 0x84, 0x23


#undef RF_GPIO_PIN_CFG
#define RF_GPIO_PIN_CFG GLOBAL_GPIO_PIN_CFG
#undef RF_GLOBAL_CLK_CFG_1
#define RF_GLOBAL_CLK_CFG_1 GLOBAL_CLK_CFG

// Remember to change RF_MODEM_AFC_LIMITER_1_3_1 !!!!!!!!!

static const uint8_t SI4468_config[] = RADIO_CONFIGURATION_DATA_ARRAY;

// Set new state
static int SI4463_set_state(si446x_state_t newState)
{
  uint8_t data[] = {
        SI446X_CMD_CHANGE_STATE,
        newState
  };
  return SI4463_do_api(data, sizeof(data), NULL, 0);
}

static uint8_t gpio_state[5] = {
  SI446X_GPIO_MODE_DIV_CLK,
  SI446X_GPIO_MODE_CTS,
  SI446X_GPIO_MODE_DONOTHING,
  SI446X_GPIO_MODE_DONOTHING,
  SI446X_GPIO_MODE_DRIVE1
};

int SI4463_refresh_gpio(void)
{
  uint8_t data2[] = {
    SI446X_CMD_GPIO_PIN_CFG,
    gpio_state[0], // GPIO[0]
    gpio_state[1], // GPIO[1]
    gpio_state[2], // GPIO[2]       // High
    gpio_state[3], // GPIO[3]       // TX/RX
    gpio_state[4], // NIRQ          // Direct
    0,             // SDO
    0              // GEN_CONFIG
  };
  return SI4463_do_api(data2, sizeof(data2), NULL, 0);
}

void SI4463_set_gpio(int i, int s)
{
  if (gpio_state[i] == s)
    return;
  gpio_state[i] = s;
#if 0   // debug gpio
  gpio_state[2] = 3;
  gpio_state[3] = 2;
#endif
  SI4463_refresh_gpio();
}

#if 0
static void SI4463_clear_FIFO(void)
{
    // 'static const' saves 20 bytes of flash here, but uses 2 bytes of RAM
  static const uint8_t clearFifo[] = {
        SI446X_CMD_FIFO_INFO,
        SI446X_FIFO_CLEAR_RX | SI446X_FIFO_CLEAR_TX
  };
  SI4463_do_api((uint8_t*)clearFifo, sizeof(clearFifo), NULL, 0);
}
#endif

void SI4463_set_output_level(int t)
{
  SI4463_output_level = t;
  if (SI4463_in_tx_mode) {
#if 1
    {
      uint8_t data[] = {
       SI446X_CMD_SET_PROPERTY, 0x22, 0x04, 0x00,        // PA_MODE
       0x08,  // Coarse PA mode and class E PA        Fine PA mode = 0x04
       (uint8_t)SI4463_output_level,  // Level
       0x00,  // Duty
       0x00   // Ramp
      };
      SI4463_do_api(data, sizeof(data), NULL, 0);
    }
#else
    SI4463_start_tx(0);         // Refresh output level
#endif
  }
}
void SI4463_start_tx(uint8_t CHANNEL)
{
//   si446x_state_t s;
#if 0
  s = SI4463_get_state();
  if (s == SI446X_STATE_RX){
    SI4463_set_state(SI446X_STATE_READY);
    my_microsecond_delay(200);
    s = SI4463_get_state();
    if (s != SI446X_STATE_READY){
      my_microsecond_delay(1000);
    }
  }
#endif
#if 1
  {
    uint8_t data[] = {
     SI446X_CMD_SET_PROPERTY, 0x20, 0x01, 0x00,
     0x00,  // CW mode
    };
    SI4463_do_api(data, sizeof(data), NULL, 0);
  }
#endif
#if 1
  {
    uint8_t data[] = {
     SI446X_CMD_SET_PROPERTY, 0x22, 0x04, 0x00,        // PA_MODE
     0x08,  // Coarse PA mode and class E PA        Fine PA mode = 0x04
     (uint8_t)SI4463_output_level,  // Level
     0x00,  // Duty
     0x00   // Ramp
    };
    SI4463_do_api(data, sizeof(data), NULL, 0);
  }
#endif
//  retry:
  {
    uint8_t data[] =
    {
     SI446X_CMD_ID_START_TX,
     CHANNEL,
     0, // Stay in TX state
     0,  // TX len
     0,  // TX len
     0,// TX delay
     0// Num repeat
    };
    SI4463_do_api(data, sizeof(data), NULL, 0);
  }
  SI4463_in_tx_mode = true;
  my_microsecond_delay(1000);
#if 0       // Check state for debugging
  s = SI4463_get_state();
  if (s != SI446X_STATE_TX){
    my_microsecond_delay(1000);
    goto retry;
  }
#endif

}


int SI4463_start_rx(uint8_t CHANNEL)
{
  si446x_state_t s = SI4463_get_state();
  if (s == SI446X_STATE_TX){
    if (SI4463_set_state(SI446X_STATE_READY))
      return -1;
  }
  if (SI4463_refresh_gpio()) return -1;


#if 0
  {
    uint8_t data[] =
    {
       0x11, 0x10, 0x01, 0x03, 0xf0
    };
    SI4463_do_api(data, sizeof(data), NULL, 0); // Send PREAMBLE_CONFIG_STD_2 for long timeout
  }
  {
    uint8_t data[] =
    {
     0x11, 0x20, 0x01, 0x00, 0x09,  // Restore OOK mode
    };
    SI4463_do_api(data, sizeof(data), NULL, 0);
  }
#endif
  uint8_t data[] = {
    SI446X_CMD_ID_START_RX,
    CHANNEL,
    0,
    0,
    0,
#ifdef __USE_FRR_FOR_RSSI__
    SI446X_CMD_START_RX_ARG_NEXT_STATE1_RXTIMEOUT_STATE_ENUM_RX,
#else
    SI446X_CMD_START_RX_ARG_NEXT_STATE1_RXTIMEOUT_STATE_ENUM_NOCHANGE,
#endif
    SI446X_CMD_START_RX_ARG_NEXT_STATE2_RXVALID_STATE_ENUM_RX,
    SI446X_CMD_START_RX_ARG_NEXT_STATE3_RXINVALID_STATE_ENUM_RX
  };
//retry:
  if (SI4463_do_api(data, sizeof(data), NULL, 0))
    return -1;

#if 0       // Get state for debugging
  si446x_state_t s = SI4463_get_state();
  if (s != SI446X_STATE_RX) {
    my_microsecond_delay(1000);
    goto retry;
  }
#endif
#if 0
  {
    uint8_t data2[] = { 0x11, 0x20, 0x01, 0x58, 0x10 };  // set FAST_DELAY to 0x10,
    SI4463_do_api(data2, sizeof(data2), NULL, 0);
  }
#endif
  SI4463_in_tx_mode = false;
  return 0;
}



void SI4463_short_start_rx(void)
{
  uint8_t data[] = {
    SI446X_CMD_ID_START_RX,
  };
  SI4463_do_api(data, sizeof(data), NULL, 0);
  SI4463_in_tx_mode = false;
}

void SI4463_clear_int_status(void)
{
  uint8_t data[9] = {
    SI446X_CMD_ID_GET_INT_STATUS
  };
  SI4463_do_api(data, 1, data, SI446X_CMD_REPLY_COUNT_GET_INT_STATUS);

}

void set_calibration_freq(int ref)
{
  if (ref >= 0) {
    SI4463_set_gpio(0, SI446X_GPIO_MODE_DIV_CLK);     // GPIO 0 is clock out
    uint8_t data2[5] = {  // GLOBAL_CLK_CFG Clock config
      SI446X_CMD_SET_PROPERTY, SI446X_PROP_GROUP_GLOBAL, 0x01, 0x01,
      0x40|(ref<<3)// DIVIDED_CLK_EN = 1, DIVIDED_CLK_SEL = ref, CLK_32K_SEL = 0
    };
    SI4463_do_api(data2, 5, NULL, 0);
  } else {
    SI4463_set_gpio(0, SI446X_GPIO_MODE_TRISTATE);    // stop clock out
  }
}

si446x_info_t SI4463_info;

void Si446x_getInfo(si446x_info_t* info)
{
  uint8_t data[8] = {SI446X_CMD_PART_INFO};
  SI4463_do_api(data, 1, data, 8);

  info->chipRev   = data[0];
  info->part      = (data[1]<<8) | data[2];
  info->partBuild = data[3];
  info->id        = (data[4]<<8) | data[5];
  info->customer  = data[6];
  info->romId     = data[7];


  data[0] = SI446X_CMD_FUNC_INFO;
  SI4463_do_api(data, 1, data, 6);

  info->revExternal   = data[0];
  info->revBranch     = data[1];
  info->revInternal   = data[2];
  info->patch         = (data[3]<<8) | data[4];
  info->func          = data[5];
}

float old_temp = -100;
#define TEMP_HISTERESE 0.5

float Si446x_get_temp(void)
{
  uint8_t data[8] = { SI446X_CMD_GET_ADC_READING, 0x10, 0 };
  SI4463_do_api(data, 3, data, 8);
  float t = (data[4] << 8) + data[5];
  t = (899.0 * t /4096.0) - 293.0;
  if (t > old_temp - TEMP_HISTERESE && t < old_temp + TEMP_HISTERESE) {
    return(old_temp);
  }
  old_temp = t;
  return t;
}

#ifdef notused
static uint8_t SI4463_get_device_status(void)
{
  uint8_t data[2] =
  {
    SI446X_CMD_ID_REQUEST_DEVICE_STATE, 0
  };
  SI4463_do_api(data, 1, data, SI446X_CMD_REPLY_COUNT_REQUEST_DEVICE_STATE);
  return(data[0]);
}
#endif


// Read a fast response register
uint8_t getFRR(uint8_t reg)
{
  set_SPI_mode(SPI_MODE_SI);
//  SPI_BR_SET(SI4432_SPI, SI4432_SPI_SPEED);

  return SI4463_read_byte(reg);
}

// Get current radio state
static si446x_state_t SI4463_get_state(void)
{
#if 0
#if 0
   uint8_t data[2] = {
       SI446X_CMD_REQUEST_DEVICE_STATE
   };
   SI4463_do_api(data, 1, data, 2);
   uint8_t state = data[0] & 0x0F;
#endif
   uint8_t state = SI4463_get_device_status();
#else

//again:
//   SI4463_wait_for_cts();
   uint8_t state = getFRR(SI446X_CMD_READ_FRR_B);
#endif
#if 0 // Only for debugging
   if (state == 255) {
     my_microsecond_delay(100);
     goto again;
   }
#endif
   if(state == SI446X_STATE_TX_TUNE)
        state = SI446X_STATE_TX;
    else if(state == SI446X_STATE_RX_TUNE)
        state = SI446X_STATE_RX;
    else if(state == SI446X_STATE_READY2)
        state = SI446X_STATE_READY;
    else
      state = state;
    return (si446x_state_t)state;
}


void set_RSSI_comp(void)
{
  // Set properties:           RF_MODEM_RSSI_COMP_1
  // Number of properties:     1
  // Group ID:                 0x20
  // Start ID:                 0x4E
  // Default values:           0x40,
  // Descriptions:
  //   MODEM_RSSI_COMP - RSSI compensation value.
  //
  // #define RF_MODEM_RSSI_COMP_1 0x11, 0x20, 0x01, 0x4E, 0x40
  uint8_t data[5] = {
    SI446X_CMD_SET_PROPERTY,
    SI446X_PROP_GROUP_MODEM,
    0x01,
    0x4E,  // MODEM_RSSI_COMP set as
    0x40   // RSSI_COMP
  };
  SI4463_do_api(data, sizeof(data), NULL, 0);
}

static bool SI4463_offset_active = false;
static void SI4463_set_offset(int16_t offset){
  // Set properties:           MODEM_FREQ_OFFSET
  // Number of properties:     2
  // Group ID:                 0x20
  // Start ID:                 0x0d
  // Default values:           0x00, 0x00
  // Descriptions:
  //   MODEM_FREQ_OFFSET1 - High byte of the offset
  //   MODEM_FREQ_OFFSET2 - Low byte of the offset
  //
  uint8_t data[] = {
    SI446X_CMD_SET_PROPERTY,
    SI446X_PROP_GROUP_MODEM,
    0x02,
    0x0d,      // MODEM_FREQ_OFFSET
    (uint8_t) ((offset>>8) & 0xff),
    (uint8_t) ((offset) & 0xff)
  };
  SI4463_do_api(data, sizeof(data), NULL, 0);
  SI4463_offset_changed = true;
  SI4463_offset_active = (offset != 0);
}

// Set offset for frequency
void si_set_offset(int16_t offset) {
  SI4463_offset_value = offset;
  SI4463_set_offset(offset);
}

// Set additional offset for fm modulation output
void si_fm_offset(int16_t offset) {
  SI4463_set_offset(offset + SI4463_offset_value);
}

#ifdef __FAST_SWEEP__
extern deviceRSSI_t age[POINTS_COUNT];
static int buf_index = 0;
static uint16_t buf_end = 0;
static bool  buf_read = false;

static char Si446x_readRSSI(void){
  uint8_t rssi;
#ifdef __USE_FRR_FOR_RSSI__
#if 0               // Restart RX, not needed as modem stays in RX mode
  SI4463_WAIT_CTS;                       // Wait for CTS
  SPI_WRITE_8BIT(SI4432_SPI, SI446X_CMD_ID_START_RX);
  while (SPI_IS_BUSY(SI4432_SPI)) ;      // wait tx
  SPI_READ_8BIT(SI4432_SPI);             // Skip command byte response
#endif
  while (SPI_RX_IS_NOT_EMPTY(SI4432_SPI))
    (void)SPI_READ_8BIT(SI4432_SPI);     // Remove lingering bytes
  do {
    SI_CS_LOW;
    SPI_WRITE_8BIT(SI4432_SPI, SI446X_CMD_READ_FRR_A);
    SPI_WRITE_8BIT(SI4432_SPI, 0xFF);       // read FRR_A
    while (SPI_IS_BUSY(SI4432_SPI));        // wait
    SPI_READ_8BIT(SI4432_SPI);
    rssi = SPI_READ_8BIT(SI4432_SPI);       // get last byte as FRR_A (rssi)
    SI_CS_HIGH;
  } while (rssi == 0);                      // Wait for latch to happen
#elif 0
  SI_CS_LOW;
  SI4463_WAIT_CTS;                       // Wait for CTS
  SPI_WRITE_8BIT(SI4432_SPI, SI446X_CMD_GET_MODEM_STATUS);
  while (SPI_IS_BUSY(SI4432_SPI)) ; // wait tx
  SI_CS_HIGH;

  while (SPI_RX_IS_NOT_EMPTY(SI4432_SPI))
    (void)SPI_READ_8BIT(SI4432_SPI);     // Remove lingering bytes
  SI4463_WAIT_CTS;                       // Wait for CTS

  SI_CS_LOW;
  SPI_WRITE_8BIT(SI4432_SPI, SI446X_CMD_READ_CMD_BUFF); // read answer
  while (SPI_IS_BUSY(SI4432_SPI)) ;      // wait tx
  SPI_READ_8BIT(SI4432_SPI);             // Drop SI446X_CMD_READ_CMD_BUFF read
  SPI_WRITE_16BIT(SI4432_SPI, 0xFFFF);   // begin read 2 bytes
  SPI_WRITE_16BIT(SI4432_SPI, 0xFFFF);   // next  read 2 bytes
  while (SPI_IS_BUSY(SI4432_SPI));       // wait tx
  SPI_READ_8BIT(SI4432_SPI);             // read CMD_ COMPLETE
  SPI_READ_8BIT(SI4432_SPI);             // MODEM_PEND
  SPI_READ_8BIT(SI4432_SPI);             // MODEM_STATUS
  rssi = SPI_READ_8BIT(SI4432_SPI);      // CURR_RSSI
//  SPI_WRITE_8BIT(SI4432_SPI, 0xFF);
//  while (SPI_IS_BUSY(SI4432_SPI)) ;    // wait tx
//  rssi = SPI_READ_8BIT(SI4432_SPI);    // LATCH_RSSI
  SI_CS_HIGH;
#else
  uint8_t data[4];
  data[0] = SI446X_CMD_GET_MODEM_STATUS;
  SI4463_do_api(data, 1, data, 3);
  rssi = data[2];
  if (rssi > 197)
    rssi -= 1;
#endif
  return rssi;
}

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

void SI446x_trigger_fill(int s, uint8_t trigger_lvl, int up_direction, int trigger_mode)
{
  (void)s;
  uint8_t rssi;
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

    age[i++] = rssi = Si446x_readRSSI();
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


void SI446x_Fill(int s, int start)
{
  (void)s;
  set_SPI_mode(SPI_MODE_SI);
#if 0       // Only for testing
  uint8_t data2[] = {
     0x11, 0x20, 0x01, 0x4C, 0x03   // set RSSI control
  };
  SI4463_do_api(data2, sizeof(data2), NULL, 0);
  uint8_t data[] = {
     0x12, 0x20, 0x01, 0x4C   // get RSSI control
  };
  SI4463_do_api(data, sizeof(data), data, 1);
#endif

//  SPI_BR_SET(SI4432_SPI, SI4432_SPI_FASTSPEED);
  uint32_t t = setting.additional_step_delay_us;

  static uint32_t old_t = 0;
  if (t < old_t +100 && t + 100 > old_t) {          // avoid oscillation
    t = (t + old_t) >> 1;
  }
  old_t = t;

  systime_t measure = chVTGetSystemTimeX();
  int i = start;
// For SI446X_CMD_READ_FRR_A need drop Rx buffer
#ifdef __USE_FRR_FOR_RSSI__
  while(SPI_RX_IS_NOT_EMPTY(SI4432_SPI)) (void)SPI_READ_8BIT(SI4432_SPI);      // Remove lingering bytes
  // Get first point data
  pureRSSI_t last;
  do{
    SI_CS_LOW;
    SPI_WRITE_8BIT(SI4432_SPI, SI446X_CMD_READ_FRR_A);
    SPI_WRITE_8BIT(SI4432_SPI, 0xFF);      // begin read 1 bytes
    while (SPI_IS_BUSY(SI4432_SPI)) ;      // wait tx
    SPI_READ_8BIT(SI4432_SPI);             // Skip command byte response
    last = SPI_READ_8BIT(SI4432_SPI);      // Get FRR A
    SI_CS_HIGH;
  } while (last == 0);

#endif
  __disable_irq();
  do {
#ifndef  __USE_FRR_FOR_RSSI__
    if (t)
      my_microsecond_delay(t);
    age[i] = Si446x_readRSSI();
    if (++i >= sweep_points) break;
#else
 // DEBUG!! restart
  SI_CS_LOW;
  SPI_WRITE_8BIT(SI4432_SPI, SI446X_CMD_READ_FRR_A);
  SPI_WRITE_8BIT(SI4432_SPI, 0xFF);      // begin read 1 bytes
  while (SPI_IS_BUSY(SI4432_SPI)) ;      // wait tx
  SPI_READ_8BIT(SI4432_SPI);             // Skip command byte response
  age[i] = SPI_READ_8BIT(SI4432_SPI);    // Get FRR A
  SI_CS_HIGH;
  // latched RSSI reset for next measure - use last known data, for stable measure time
  if (age[i] == 0) age[i] = last;
  last = age[i];
  if (t)
    my_microsecond_delay(t);
  if (++i >= sweep_points) break;
#endif

  } while(1);
  __enable_irq();
  setting.measure_sweep_time_us = sa_ST2US(chVTGetSystemTimeX() - measure);
  buf_index = (start<=0 ? 0 : start); // Is used to skip 1st entry during level triggering
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
static int32_t RSSI_RAW  = 0;

void SI4432_Listen(int s)
{
  (void) s;
  uint8_t max = 0;
  uint16_t count = 0;
  operation_requested = OP_NONE;
//  SI4463_WAIT_CTS;                // Wait for CTS
  do {
      uint8_t v = Si446x_readRSSI();
      if (config.no_audio_agc) {
        DAC->DHR12R1 = v << 4; // Use DAC: CH1 and put 12 bit right aligned value
      } else {
        if (max < v)                // Peak
          max = v;
        if (count > 1000) {         // Decay
          max -= 1;
          count = 0;
        } else
          count++;
        v = max - v;
        DAC->DHR12R1 = dBm_to_volt[v] << 4; // Use DAC: CH1 and put 12 bit right aligned value
      }
    } while(operation_requested == OP_NONE);
  count = 0;
//  dacPutChannelX(&DACD2, 0, 0);
}
#endif


int16_t Si446x_RSSI(bool break_on_operation)
{
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

  int i = setting.repeat;
  if (setting.exp_aver == 1)
    RSSI_RAW  = 0;

  //  SI4463_WAIT_CTS;         // Wait for CTS
  do{
    //   if (MODE_INPUT(setting.mode) && RSSI_R
#define SAMPLE_COUNT 1
    int j = SAMPLE_COUNT; //setting.repeat;
    int RSSI_RAW_ARRAY[3];
    do{
//      if (setting.step_delay)
//        my_microsecond_delay(setting.step_delay);             // moved to sweep
      RSSI_RAW_ARRAY[--j] = Si446x_readRSSI();
      if (j == 0) break;
//      my_microsecond_delay(20);
    }while(1);
#if SAMPLE_COUNT == 3
    int t;
    if (RSSI_RAW_ARRAY[0] > RSSI_RAW_ARRAY[1]) {
      t = RSSI_RAW_ARRAY[1];
      RSSI_RAW_ARRAY[1] = RSSI_RAW_ARRAY[0];
      RSSI_RAW_ARRAY[0] = t;
    }
    if (RSSI_RAW_ARRAY[1] > RSSI_RAW_ARRAY[2]) {
      t = RSSI_RAW_ARRAY[2];
      RSSI_RAW_ARRAY[2] = RSSI_RAW_ARRAY[1];
      RSSI_RAW_ARRAY[1] = t;
    }
    if (RSSI_RAW_ARRAY[0] > RSSI_RAW_ARRAY[1]) {
      t = RSSI_RAW_ARRAY[1];
      RSSI_RAW_ARRAY[1] = RSSI_RAW_ARRAY[0];
      RSSI_RAW_ARRAY[0] = t;
    }

    RSSI_RAW += DEVICE_TO_PURE_RSSI(RSSI_RAW_ARRAY[1]);
#else
#ifdef TINYSA4
    if (setting.exp_aver == 1)
      RSSI_RAW += DEVICE_TO_PURE_RSSI(RSSI_RAW_ARRAY[0]);
    else
      RSSI_RAW = ((setting.exp_aver-1) * RSSI_RAW  + DEVICE_TO_PURE_RSSI(RSSI_RAW_ARRAY[0]))/setting.exp_aver;

#else
    RSSI_RAW += DEVICE_TO_PURE_RSSI(RSSI_RAW_ARRAY[0]);
#endif
#endif
    if (--i <= 0) break;
    if (setting.repeat > 1)
      my_microsecond_delay(SI4432_step_delay);
    if (break_on_operation && operation_requested)
      break;
  }while(1);

  if (setting.repeat > 1 && setting.exp_aver == 1)
    RSSI_RAW = RSSI_RAW / setting.repeat;

  return RSSI_RAW;
}



void SI446x_set_AGC_LNA(uint8_t v)
{
  uint8_t data[2] = {
      0xd0,                 // AGC_OVERRIDE
      v
  };
  SI4463_do_api(data, sizeof(data), NULL, 0);
#if 0
  if (v == 0) {
    data[0] = 0xd1;     // Read AGC?????? NO
    SI4463_do_api(data, 1, data, 1);
  }
#endif
}


#ifdef notused
// Do an ADC conversion
static uint16_t getADC(uint8_t adc_en, uint8_t adc_cfg, uint8_t part)
{
    uint8_t data[6] = {
        SI446X_CMD_GET_ADC_READING,
        adc_en,
        adc_cfg
    };
    SI4463_do_api(data, 3, data, 6);
    return (data[part]<<8 | data[part + 1]);
}
#endif

// -------------- 0.2 kHz ----------------------------

#include "radio_config_Si4468_undef.h"
#include "radio_config_Si4468_200Hz.h"
#include "radio_config_Si4468_short.h"

static const uint8_t SI4463_RBW_02kHz[] =
    RADIO_CONFIGURATION_DATA_ARRAY;

// -------------- 1kHz ----------------------------

#include "radio_config_Si4468_undef.h"
#include "radio_config_Si4468_1kHz.h"
#include "radio_config_Si4468_short.h"

static const uint8_t SI4463_RBW_1kHz[] =
    RADIO_CONFIGURATION_DATA_ARRAY;

// -------------- 3 kHz ----------------------------
#include "radio_config_Si4468_undef.h"
#include "radio_config_Si4468_3kHz.h"
#include "radio_config_Si4468_short.h"

static const uint8_t SI4463_RBW_3kHz[] =
    RADIO_CONFIGURATION_DATA_ARRAY;

// -------------- 10 kHz ----------------------------
#include "radio_config_Si4468_undef.h"
#include "radio_config_Si4468_10kHz.h"
#include "radio_config_Si4468_short.h"

static const uint8_t SI4463_RBW_10kHz[] =
    RADIO_CONFIGURATION_DATA_ARRAY;

// -------------- 30 kHz ----------------------------
#include "radio_config_Si4468_undef.h"
#include "radio_config_Si4468_30kHz.h"
#include "radio_config_Si4468_short.h"

static const uint8_t SI4463_RBW_30kHz[] =
    RADIO_CONFIGURATION_DATA_ARRAY;

// -------------- 100kHz ----------------------------
#include "radio_config_Si4468_undef.h"
#include "radio_config_Si4468_100kHz.h"
#include "radio_config_Si4468_short.h"

static const uint8_t SI4463_RBW_100kHz[] =
    RADIO_CONFIGURATION_DATA_ARRAY;

// -------------- 300kHz ----------------------------

#include "radio_config_Si4468_undef.h"
#include "radio_config_Si4468_300kHz.h"
#include "radio_config_Si4468_short.h"

static const uint8_t SI4463_RBW_300kHz[] =
    RADIO_CONFIGURATION_DATA_ARRAY;

// -------------- 600kHz ----------------------------

#include "radio_config_Si4468_undef.h"
#include "radio_config_Si4468_600kHz.h"
#include "radio_config_Si4468_short.h"

static const uint8_t SI4463_RBW_600kHz[] =
    RADIO_CONFIGURATION_DATA_ARRAY;

// -------------- 850kHz ----------------------------

#include "radio_config_Si4468_undef.h"
#include "radio_config_Si4468_850kHz.h"
#include "radio_config_Si4468_short.h"

static const uint8_t SI4463_RBW_850kHz[] =
    RADIO_CONFIGURATION_DATA_ARRAY;

// User asks for an RBW of WISH, go through table finding the last triple
// for which WISH is greater than the first entry, use those values,
// Return the first entry of the following triple for the RBW actually achieved
#define IF_BW(dwn3, ndec, filset) (((dwn3)<<7)|((ndec)<<4)|(filset))
typedef struct {
  const uint8_t  *reg;                   // IF_BW(dwn3, ndec, filset)
  int16_t   RSSI_correction_x_10;  // Correction * 10
  int16_t RBWx10;                // RBW in kHz
  int16_t noise_correction_x10; // Noise marker correction * 10
}RBW_t; // sizeof(RBW_t) = 8 bytes

static const RBW_t RBW_choices[] =
{
#if 0
 // BW register    corr  freq
 {SI4463_RBW_02kHz,  20,3,    26},
 {SI4463_RBW_1kHz,   23,10,    8},
 {SI4463_RBW_3kHz,   17,30,    9},
 {SI4463_RBW_10kHz,  12,100,   7},
 {SI4463_RBW_30kHz,  12,300,  10},
 {SI4463_RBW_100kHz,  2,1000, 17},
 {SI4463_RBW_300kHz,  4,3000, 10},
 {SI4463_RBW_600kHz, -10,6000,  8},
 {SI4463_RBW_850kHz, -9,8500,  8},
#else

#define NOISE_BASE_CORRECTION   7   // 7
 {SI4463_RBW_02kHz,  10,2,  NOISE_BASE_CORRECTION +  0}, //
 {SI4463_RBW_1kHz,   15,10, NOISE_BASE_CORRECTION +  -5},//
 {SI4463_RBW_3kHz,   10,30, NOISE_BASE_CORRECTION +  -5},//
 {SI4463_RBW_10kHz,  14,100,NOISE_BASE_CORRECTION +  -2}, //
 {SI4463_RBW_30kHz,  0,300, NOISE_BASE_CORRECTION +  -2},//
 {SI4463_RBW_100kHz, 0,1000,NOISE_BASE_CORRECTION +  -1},//
 {SI4463_RBW_300kHz, 0,3000,NOISE_BASE_CORRECTION},      // 300k must have RSSI correction = 0
 {SI4463_RBW_600kHz, 5,6000,NOISE_BASE_CORRECTION +  -2},//
 {SI4463_RBW_850kHz, 8,8500,NOISE_BASE_CORRECTION +  2},//
#endif
};

const uint8_t SI4432_RBW_count = ((int)(sizeof(RBW_choices)/sizeof(RBW_t)));

static pureRSSI_t SI4463_RSSI_correction = float_TO_PURE_RSSI(-120);
bool SI4463_RSSI_correction_enabled = true;
int16_t SI4463_noise_correction_x10;
static int prev_band = -1;

pureRSSI_t getSI4463_RSSI_correction(void){
  return SI4463_RSSI_correction;
};

void switch_SI4463_RSSI_correction(bool enabled){
  SI4463_RSSI_correction_enabled = enabled;
};

void SI4463_set_IF(int band) {
// Set properties:           MODEM_IF_FREQ
// Number of properties:     3
// Group ID:                 0x20
// Start ID:                 0x1b
// Descriptions:
//   MODEM_IF_FREQ - Set the IF frequency as a function of frequency band.
  // #define RF_MODEM_CLKGEN_BAND_1 0x11, 0x20, 0x01, 0x51, 0x0A
//#define RF_MODEM_TX_RAMP_DELAY_12 0x11, 0x20, 0x03, 0x18, 0x03, 0xC0, 0x00

static const uint8_t if_freq_high[6] = { 0x03, 0x03, 0x03, 0x03, 0, 0x02};
static const uint8_t if_freq_low[6] =  { 0xc0, 0x60, 0x80, 0x40, 0, 0x80};

  uint8_t data3[] = { 0x11, 0x20, 0x03, 0x1b, if_freq_high[band], if_freq_low[band] };
  SI4463_do_api(data3, sizeof(data3), NULL, 0);
}

void SI4463_set_modem_DSM(void) {
// Set properties:           MODEM_DSM_CTRL
// Number of properties:     1
// Group ID:                 0x20
// Start ID:                 0x02
// Descriptions:
//   MODEM_DSM_CTRL - Miscellaneous control bits for the Delta-Sigma Modulator (DSM) in the PLL Synthesizer.
//#define MODEM_DSM_CTRL 0x11, 0x20, 0x01, 0x02, 0x07

  static const uint8_t data3[] = { 0x11, 0x20, 0x01, 0x02, 0x03};
  SI4463_do_api((uint8_t *)data3, sizeof(data3), NULL, 0);
}


uint16_t force_rbw(int f)
{
  if (/*SI4463_in_tx_mode || */ f >= (int)(sizeof(RBW_choices)/sizeof(RBW_t)))
    return(0);          // RBW can be selected before switch to input mode is made
  SI4463_set_state(SI446X_STATE_READY);
  const uint8_t *config = RBW_choices[f].reg;
  uint16_t i=0;
  while(config[i] != 0)
  {
    SI4463_do_api((void *)&config[i+1], config[i], NULL, 0);
    i += config[i]+1;
  }
  if (prev_band != 0) SI4463_set_IF(prev_band);         // restore IF frequency if not in band zero
  if (setting.frequency_step < 100) SI4463_set_modem_DSM();   // Set small frequency steps if needed
  SI4463_clear_int_status();
  SI4463_short_start_rx();           // This can cause recalibration
//  SI4463_wait_for_cts();
  set_RSSI_comp();
//  prev_band = -1;
  SI4463_RSSI_correction = ( SI4463_RSSI_correction_enabled ?  float_TO_PURE_RSSI(RBW_choices[f].RSSI_correction_x_10 - 1200)/10 : float_TO_PURE_RSSI(-120) ) ;  // Set RSSI correction
  SI4463_noise_correction_x10 = RBW_choices[f].noise_correction_x10;
  SI4463_rbw_selected = f;
  return RBW_choices[f].RBWx10;                                                   // RBW achieved by SI4463 in kHz * 10
}

uint16_t set_rbw(uint16_t WISH)  {
  int i;
  for (i=0; i < (int)(sizeof(RBW_choices)/sizeof(RBW_t)) - 1; i++)
    if (WISH <= RBW_choices[i].RBWx10)
      break;
  return force_rbw(i);
}


#define Npresc 1    // 0=low / 1=High performance mode

freq_t SI4463_set_freq(freq_t freq)
{
  uint32_t output_divider;
//  SI4463_set_gpio(3,SI446X_GPIO_MODE_DRIVE1);       // For measuring duration of set_freq
  int S = 4 ;               // Approx 100 Hz channels
  SI4463_channel = 0;
  if (freq >= 822000000 && freq <= 1150000000)         {       // 822 to 1130MHz
    SI4463_band = 0;
    output_divider = 4 * FREQ_MULTIPLIER;
  } else if (freq >= 411000000 && freq <= 566000000) {    // 411 to  568MHz
    SI4463_band = 2;
    output_divider = 8 * FREQ_MULTIPLIER ;
  } else if (freq >= 329000000 && freq <= 454000000) {    // 329 to 454MHz
    SI4463_band = 1;
    output_divider = 10 * FREQ_MULTIPLIER;
  } else if (freq >= 274000000 && freq <= 378000000) {    // 274 to 378
    SI4463_band = 3;
    output_divider = 12 * FREQ_MULTIPLIER;
  } else if (freq >= 137000000 && freq <= 189000000){ // 137 to 189
    SI4463_band = 5;
    output_divider = 24 * FREQ_MULTIPLIER;
#if 0                           // Band 4, 6 and 7 do not function
  } else if (freq >= 137000000 && freq <= 189000000){ // 220 to 266
    SI4463_band = 4;
    output_divider = 12;
#endif
  } else
    return 0;
  if (SI4463_offset_active) {
    si_set_offset(0);
    SI4463_offset_active = false;
  }
  uint32_t R = (freq * output_divider) / (Npresc ? 2*config.setting_frequency_30mhz : 4*config.setting_frequency_30mhz) - 1;        // R between 0x00 and 0x7f (127)
  uint64_t MOD = 524288; // = 2^19
  uint32_t  F = ((freq * output_divider*MOD) / (Npresc ? 2*config.setting_frequency_30mhz : 4*config.setting_frequency_30mhz)) - R*MOD;
  freq_t actual_freq = (R*MOD + F) * (Npresc ? 2*config.setting_frequency_30mhz : 4*config.setting_frequency_30mhz)/ output_divider/MOD;
#if 0       // Only for debugging
  int delta = freq - actual_freq;
  if (delta < -100 || delta > 100 ){
    while(1)
      my_microsecond_delay(10);
  }
  if (F < MOD || F >= MOD*2){
    while(1)
      my_microsecond_delay(10);
  }
#endif

#if 0               // Hopping is fast but frequency setting is not yet reliable !!!!!
  if (SI4463_band == prev_band) {
    int vco = 2091 + ((((freq / 4 ) * output_divider - 850000000)/1000) * 492) / 200000;

    if (SI4463_in_tx_mode) {
      uint8_t data[] = {
                       SI446X_CMD_ID_TX_HOP,
                      (uint8_t) R,                   //  R data[4]
                      (uint8_t) ((F>>16) & 255),     //  F2,F1,F0 data[5] .. data[7]
                      (uint8_t) ((F>> 8) & 255),     //  F2,F1,F0 data[5] .. data[7]
                      (uint8_t) ((F    ) & 255),     //  F2,F1,F0 data[5] .. data[7]
                      (vco>>8) & 0xff,
                      vco & 0xff,
                      0x00,
                      0x32
      };
      SI4463_do_api(data, sizeof(data), NULL, 0);
    } else {

      uint8_t data[] = {
                       SI446X_CMD_ID_RX_HOP,
                      (uint8_t) R,                   //  R data[4]
                      (uint8_t) ((F>>16) & 255),     //  F2,F1,F0 data[5] .. data[7]
                      (uint8_t) ((F>> 8) & 255),     //  F2,F1,F0 data[5] .. data[7]
                      (uint8_t) ((F    ) & 255),     //  F2,F1,F0 data[5] .. data[7]
                      (vco>>8) & 0xff,
                      vco & 0xff
      };
      SI4463_do_api(data, sizeof(data), NULL, 0);
    }
    SI4463_frequency_changed = true;
//    SI4463_set_gpio(3,SI446X_GPIO_MODE_DRIVE0);   // For measuring duration of set_freq
    return actual_freq;
  }
#endif

  SI4463_set_state(SI446X_STATE_READY);

  /*
  // Set properties:           RF_FREQ_CONTROL_INTE_8
  // Number of properties:     8
  // Group ID:                 0x40
  // Start ID:                 0x00
  // Default values:           0x3C, 0x08, 0x00, 0x00, 0x00, 0x00, 0x20, 0xFF,
  // Descriptions:
  //   FREQ_CONTROL_INTE - Frac-N PLL Synthesizer integer divide number.
  //   FREQ_CONTROL_FRAC_2 - Frac-N PLL fraction number.
  //   FREQ_CONTROL_FRAC_1 - Frac-N PLL fraction number.
  //   FREQ_CONTROL_FRAC_0 - Frac-N PLL fraction number.
  //   FREQ_CONTROL_CHANNEL_STEP_SIZE_1 - EZ Frequency Programming channel step size.
  //   FREQ_CONTROL_CHANNEL_STEP_SIZE_0 - EZ Frequency Programming channel step size.
  //   FREQ_CONTROL_W_SIZE - Set window gating period (in number of crystal reference clock cycles) for counting VCO frequency during calibration.
  //   FREQ_CONTROL_VCOCNT_RX_ADJ - Adjust target count for VCO calibration in RX mode.
   */
  // #define RF_FREQ_CONTROL_INTE_8_1 0x11, 0x40, 0x08, 0x00, 0x41, 0x0D, 0xA9, 0x5A, 0x4E, 0xC5, 0x20, 0xFE
  uint8_t data[] = {
                    0x11, 0x40, 0x06, 0x00,
                    (uint8_t) R,                   //  R data[4]
                    (uint8_t) ((F>>16) & 255),     //  F2,F1,F0 data[5] .. data[7]
                    (uint8_t) ((F>> 8) & 255),     //  F2,F1,F0 data[5] .. data[7]
                    (uint8_t) ((F    ) & 255),     //  F2,F1,F0 data[5] .. data[7]
                    (uint8_t) ((S>> 8) & 255),     //  Step size data[8] .. data[9]
                    (uint8_t) ((S    ) & 255),     //  Step size data[8] .. data[9]
#if 1
                    0x20,                   // Window gate
                    0xFF,                    // Adj count
#endif
  };
  SI4463_do_api(data, sizeof(data), NULL, 0);

  if (SI4463_band != prev_band) {
    /*
  // Set properties:           RF_MODEM_CLKGEN_BAND_1
  // Number of properties:     1
  // Group ID:                 0x20
  // Start ID:                 0x51
  // Default values:           0x08,
  // Descriptions:
  //   MODEM_CLKGEN_BAND - Select PLL Synthesizer output divider ratio as a function of frequency band.
     */
    // #define RF_MODEM_CLKGEN_BAND_1 0x11, 0x20, 0x01, 0x51, 0x0A
    uint8_t data2[] = {
                       0x11, 0x20, 0x01, 0x51,
                      /* 0x10 + */ (uint8_t)(SI4463_band + (Npresc ? 0x08 : 0))           // 0x08 for high performance mode, 0x10 to skip recal
    };
    SI4463_do_api(data2, sizeof(data2), NULL, 0);


    SI4463_set_IF(SI4463_band);
    prev_band = SI4463_band;
  }
  if (SI4463_in_tx_mode)
    SI4463_start_tx(0);
  else {
    SI4463_start_rx(0);
  }
//  SI4463_set_gpio(3,SI446X_GPIO_MODE_DRIVE0);        // For measuring duration of set_freq
  SI4463_frequency_changed = true;
  return actual_freq;
}

void SI4463_init_rx(void)
{
 reset:
  SI_SDN_LOW;
  my_microsecond_delay(10000);
  SI_SDN_HIGH;
  my_microsecond_delay(10000);
  SI_SDN_LOW;
  my_microsecond_delay(100000);
  if (SI4463_WAIT_CTS) {
    ili9341_drawstring("SI4468 Reset fail", 0,10);
    goto reset;
  }
  for(uint16_t i=0;i<sizeof(SI4468_config);i++)
  {
    if (SI4463_do_api((void *)&SI4468_config[i+1], SI4468_config[i], NULL, 0)){
      ili9341_drawstring("SI4468 Configure fail", 0,10);
      goto reset;
    }
    i += SI4468_config[i];
  }
  clear_frequency_cache();
  if (SI4463_start_rx(SI4463_channel)) {
    ili9341_drawstring("SI4468 Start fail", 0,10);
    goto reset;
  }
  // Si446x_getInfo(&SI4463_info);
  prev_band = -1; // 433MHz
}

void SI4463_init_tx(void)
{

  SI4463_start_tx(0);
  prev_band = -1; // 433MHz
}

int SI4463_is_in_tx_mode(void)
{
  return SI4463_in_tx_mode;
}

void enable_extra_lna(int s)
{
  static int old_extra_lna = -1;
  if (s != old_extra_lna) {
    if (s)
      palSetLine(LINE_LNA);
    else
      palClearLine(LINE_LNA);
    old_extra_lna = s;
    osalThreadSleepMilliseconds(500);
  }
}

#ifdef __ULTRA__
void enable_ultra(int s)
{
static int old_ultra = 2;
  if (s != old_ultra) {
    old_ultra = s;
#ifdef TINYSA4
    if (s)
      palClearLine(LINE_ULTRA);
    else
      palSetLine(LINE_ULTRA);
#endif
  }
}
#endif

void enable_rx_output(int s)
{
static int old_rx_mode = 2;
  if (s == old_rx_mode)
    return;
  old_rx_mode = s;
  if (s)
    SI4463_set_gpio(3,SI446X_GPIO_MODE_DRIVE1);
  else
    SI4463_set_gpio(3,SI446X_GPIO_MODE_DRIVE0);
}

void enable_high(int s)
{
static int old_high = 2;
  if (old_high == s)
    return;
  old_high = s;
#ifdef __NEW_SWITCHES__
  if (s)
    SI4463_set_gpio(2,SI446X_GPIO_MODE_DRIVE0);
  else
    SI4463_set_gpio(2,SI446X_GPIO_MODE_DRIVE1);
#else
  if (s)
    SI4463_set_gpio(2,SI446X_GPIO_MODE_DRIVE1);
  else
    SI4463_set_gpio(2,SI446X_GPIO_MODE_DRIVE0);
#endif
}

void enable_ADF_output(int f, int t)
{
//  (void) t;
  ADF4351_enable(f);
  ADF4351_enable_out(f);
  ADF4351_enable_aux_out(t);
}

#ifdef __NEW_SWITCHES__
void enable_direct(int s)
{
  static int old_direct = 2;
  if (s == old_direct)
    return;
  old_direct = s;
  if (s)
    SI4463_set_gpio(4,SI446X_GPIO_MODE_DRIVE0);
  else
    SI4463_set_gpio(4,SI446X_GPIO_MODE_DRIVE1);
}
#endif


#pragma GCC pop_options
