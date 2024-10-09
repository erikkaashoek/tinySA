/*
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

//#ifdef __SI4432__
#include "si4432.h"		// comment out for simulation
//#endif
#include "stdlib.h"
//#define TINYSA4

#pragma GCC push_options
#ifdef TINYSA4
#pragma GCC optimize ("Os")
#else
#pragma GCC optimize ("Os")
#endif

#if 0
#define TRACE(X)  {     DAC->DHR12R1 =(uint32_t) ((X)*400); }       // Enable for realtime tracing
#else
#define TRACE(X)  // {     DAC->DHR12R1 = (X*400); }       // Enable for realtime tracing
#endif

#ifdef __FFT_DECONV__
void FFT(float *real, float *imag, int length, bool inverse);
float *real = (float *) &spi_buffer[0];
float *imag = (float *) &spi_buffer[512];
float *real2 = (float *) &spi_buffer[1024];
float *imag2 = (float *) &spi_buffer[1536];
#endif

#ifdef __FFT_VBW__
void FFT(float *real, float *imag, int length, bool inverse);
float *real = (float *) &spi_buffer[0];
float *imag = (float *) &spi_buffer[512];
#endif

//#define __DEBUG_AGC__         If set the AGC value will be shown in the stored trace and FAST_SWEEP rmmode will be disabled
#ifdef __DEBUG_AGC__
#ifdef __FAST_SWEEP__
#undef __FAST_SWEEP__
#endif
#endif
// uint8_t dirty = true;
uint8_t scandirty = true;
bool debug_avoid = false;
bool debug_avoid_second = false;
bool progress_bar = true;
#ifdef __ULTRA__
bool debug_spur = false;
bool debug_level = false;
#endif
int current_index = -1;

setting_t setting;

uint16_t actual_rbw_x10 = 0;
freq_t frequency_step_x10 = 0;
uint16_t vbwSteps = 1;
freq_t minFreq = 0;
freq_t maxFreq = 520000000;
static float old_a = -150;          // cached value to reduce writes to level registers
int spur_gate = 100;

#ifdef __BANDS__
uint16_t current_band = 0;
#endif

#ifdef __ULTRA__
freq_t ultra_start;
//bool ultra;
int LO_harmonic;
#endif
#ifdef TINYSA4
bool direct_test;
int noise_level;
float log_averaging_correction;
//uint32_t old_CFGR; // Not used??
//uint32_t orig_CFGR; // Not used??

int debug_frequencies = false;
int linear_averaging = true;

static freq_t old_freq[5] = { 0, 0, 0, 0,0};
static freq_t real_old_freq[5] = { 0, 0, 0, 0,0};
static long real_offset = 0;

static float old_temp = 0.0;
static int LO_spur_shifted;
static int LO_mirrored;
static int LO_shifting;

void clear_frequency_cache(void)
{
  for (unsigned int i = 0; i < sizeof(old_freq)/sizeof(freq_t) ; i++) {
    old_freq[i] = 0;
    real_old_freq[i] = 0;
  }
  ADF4351_force_refresh();
}
#else
static freq_t old_freq[4] = { 0, 0, 0, 0};
static freq_t real_old_freq[4] = { 0, 0, 0, 0};
#endif

#ifdef __ULTRA__
int old_drive = -1;
int actual_drive = -1;
#endif


#define BELOW_MAX_DRIVE(X) (drive_dBm[X] - drive_dBm[MAX_DRIVE])

#ifdef TINYSA4
const float si_drive_dBm []     = {-44.1, -30, -21.6, -17, -14, -11.7, -9.9, -8.4, -7.1, -6, -5, -4.2, -3.4, -2.7 , -2.1,  -1.5,  -1, -0.47, 0};
//const float adf_drive_dBm[]     = {-13,-7.5,-4.2, 0};
//const float adf_drive_dBm[]     = {-9, -4, 0, 0};
const float adf_drive_dBm[]     = {0, 0, 0, 0};     // Only use drive 0
const uint8_t drive_register[]  = {0,   1,   2,   3,   4,   5,  6,   6,    8,    9,    10,   11,   12,   13,   14,  15,  16,  17,   18};
float *drive_dBm = (float *) si_drive_dBm;
const int min_drive = 0;
int max_drive = 18;

#ifdef __NEW_SWITCHES__
#define SWITCH_ATTENUATION  ((setting.mode == M_GENHIGH) ? 40 : 25.0 + config.out_switch_offset)
#define MAX_DRIVE   max_drive // ((setting.mode == M_GENHIGH) ? 3 : 18)
#define MIN_DRIVE   min_drive // ((setting.mode == M_GENHIGH) ? 0: 0)
#define SL_GENHIGH_LEVEL_MIN    (drive_dBm[MIN_DRIVE])
#else
#define SWITCH_ATTENUATION  ((setting.mode == M_GENHIGH && config.high_out_adf4350) ? 40 : 25.0 + config.out_switch_offset)
#define MAX_DRIVE   ((setting.mode == M_GENHIGH && config.high_out_adf4350 ) ? 3 : 18)
#define MIN_DRIVE   ((setting.mode == M_GENHIGH && config.high_out_adf4350 ) ? 0: 0)
#define SL_GENHIGH_LEVEL_MIN    (drive_dBm[MIN_DRIVE] - (config.high_out_adf4350 ? 0: 37 - config.switch_offset))
#endif
#define RECEIVE_SWITCH_ATTENUATION  (29 + config.receive_switch_offset)
//#define POWER_OFFSET    -18             // Max level with all enabled
//#define POWER_RANGE     70
//#define SL_GENHIGH_LEVEL_MIN    -15
//#define SL_GENHIGH_LEVEL_RANGE    9

#define SL_GENHIGH_LEVEL_MAX    drive_dBm[MAX_DRIVE]

#define SL_GENLOW_LEVEL_MIN    -115
#define SL_GENLOW_LEVEL_MAX   -18.5
#ifdef TINYSA4_4
#define MAX_ATTENUATE (setting.extra_lna ? 0 : 31.5)
#else
#define MAX_ATTENUATE 31.5
#endif

//    0         1         2            3             4           5
// enum {PATH_OFF, PATH_LOW, PATH_DIRECT, PATH_LEAKAGE, PATH_ULTRA, PATH_HIGH};
int signal_path = PATH_OFF;

#ifdef TINYSA4

const char *const path_text[]=PATH_TEXT;

void set_output_drive(int d)
{
  if (signal_path == PATH_LEAKAGE) {
    if (max2871 && d == 0)
      d = 3;
    ADF4351_drive(d);
  } else {
#ifdef __SI4432__
    SI4432_Sel = SI4432_RX ;
    SI4432_Drive(d);
#endif
#ifdef __SI4463__
    SI4463_set_output_level(d);
#endif
    setting.rx_drive = d;
  }
}

void set_output_step_atten(int s)
{
  setting.atten_step = s;
  if (signal_path == PATH_LEAKAGE)
//    enable_ultra(!s);
    enable_direct(s);
  else {
#ifdef TINYSA3
    SI4432_Sel = SI4432_RX ;
    if (s)
      set_switch_receive();
    else
      set_switch_transmit();
#else
    enable_rx_output(!s);
#endif
  }
}

void set_output_path(freq_t f, float level)
{
  LO_harmonic = false;
  if (depth_error) { depth_error = false; redraw_request |= REDRAW_CAL_STATUS; draw_all(true);}
  if (force_signal_path) {
    signal_path = test_path;
    setting.mixer_output = (signal_path == PATH_ULTRA);
    if (test_output_drive >=0) {
      setting.atten_step = test_output_switch;
      set_output_drive(test_output_drive);
      PE4302_Write_Byte(test_output_attenuate);
      if (debug_level && SDU1.config->usbp->state == USB_ACTIVE) {
        shell_printf ("d=%d, a=%d, s=%d\r\n",test_output_drive, test_output_attenuate,  (setting.atten_step ? 1 : 0));
        osalThreadSleepMilliseconds(100);
      }

      goto set_path;
    }
  } else if (MODE_HIGH(setting.mode))
      signal_path = PATH_HIGH;
  else if (setting.mixer_output && (f >= MAX_LOW_OUTPUT_FREQ || (config.ultra_start != ULTRA_AUTO && f > config.ultra_start))) {
    signal_path = PATH_ULTRA;
    LO_harmonic = (f > ULTRA_MAX_FREQ);
  } else if (!setting.mixer_output && (f >= MAX_LOW_OUTPUT_FREQ || (config.ultra_start != ULTRA_AUTO && f > config.ultra_start))) {
    signal_path = PATH_LEAKAGE;
    LO_harmonic = (f > MAX_LO_FREQ);
  } else if (f > MINIMUM_DIRECT_FREQ)
    signal_path = PATH_DIRECT;
  else
    signal_path = PATH_LOW;

  level += PURE_TO_float(get_frequency_correction(f));

  switch (signal_path) {
  case PATH_LEAKAGE:
    drive_dBm = (float *)adf_drive_dBm;
    max_drive = 0;
    break;
  default:
    drive_dBm = (float *)si_drive_dBm;
    max_drive = 18;
  }

  if (signal_path != PATH_LEAKAGE) {
    float dt = Si446x_get_temp() - CENTER_TEMPERATURE;
    if (dt > 0)
      level += dt * DB_PER_DEGREE_ABOVE;  // Temperature correction
    else
      level += dt * DB_PER_DEGREE_BELOW;  // Temperature correction
  }

  if (signal_path == PATH_HIGH) {
    return;                 //TODO setup high path
  }

  float ATTENUATION_RESERVE = 0.0;
  if (signal_path == PATH_LEAKAGE) {
    ATTENUATION_RESERVE = 0;
  }
  else {
    if (setting.modulation == MO_AM)
      ATTENUATION_RESERVE = 6.0;
  }
  level += ATTENUATION_RESERVE;
  float switch_atten = SWITCH_ATTENUATION;
  if (signal_path == PATH_LEAKAGE)
#ifdef TINYSA4
    switch_atten = 32.0;
#else
    switch_atten = 31.0;
#endif
  float a = level - level_max();                 // convert to all settings maximum power output equals a = zero
  if (a < -switch_atten + BELOW_MAX_DRIVE(0)) {           // Switch needed
    a = a + switch_atten;
    setting.atten_step = true;
  } else
    setting.atten_step = false;

//  set_output_step_atten(setting.atten_step);
#ifdef TINYSA4
#define LOWEST_LEVEL (setting.atten_step ? 0 : MIN_DRIVE)
#else
#define LOWEST_LEVEL MIN_DRIVE
#endif

  int d;
#ifdef TINYSA4
  if (signal_path == PATH_LEAKAGE)
    d = 0;
  else
    d = MAX_DRIVE-8;        // Start in the middle
#else
  d = MAX_DRIVE-3;        // Start in the middle
#endif

  float blw =  BELOW_MAX_DRIVE(d);
  while (a > blw && d < MAX_DRIVE) { // Increase if needed
    d++;
    blw =  BELOW_MAX_DRIVE(d);
  }
  int ar = 31 - ATTENUATION_RESERVE;
  if (setting.modulation == MO_AM) // reserve attenuator range for AM modulation
    ar = 6;
  while (a + ar < blw && d > LOWEST_LEVEL) { // reduce till it fits attenuator ((ar+ATTENUATION_RESERVE) ..  ATTENUATION_RESERVE)
    d--;
    blw =  BELOW_MAX_DRIVE(d);
  }
  a -= blw;
  set_output_drive(d);
//  if (signal_path != PATH_LEAKAGE)
  a -= ATTENUATION_RESERVE;
  if (a > 0) {
    if (!level_error) { level_error = true; redraw_request |= REDRAW_CAL_STATUS; draw_all(true);}
    a += 0.49;
    a = (float)((int)(a*2))/2.0;
    setting.level -= a;
    a = 0;
//  } else if (setting.modulation == MO_AM && a < -10) {          // Insufficient headroom for modulation
//    if (!level_error) { level_error = true; redraw_request |= REDRAW_CAL_STATUS; draw_all(true); }
  } else {
    if (level_error)  { level_error = false; redraw_request |= REDRAW_CAL_STATUS; draw_all(true); }
  }
  if (a < -31.5)
    a = -31.5;
  a = -a - 0.25;        // Rounding
  setting.attenuate_x2 = (int)(a * 2);
  PE4302_Write_Byte(setting.attenuate_x2);
  if (debug_level && SDU1.config->usbp->state == USB_ACTIVE)
    shell_printf ("level=%.2f, d=%d, a=%d, s=%d, f=%.2f\r\n", level, d, setting.attenuate_x2, (setting.atten_step ? 1 : 0), PURE_TO_float(get_frequency_correction(f)));

  enable_extra_lna(false);
  if (setting.mute)
    signal_path = PATH_OFF;

  set_path:
  switch(signal_path) {
  case PATH_OFF:
    if (SI4463_is_in_tx_mode())
      SI4463_init_rx();
    enable_ADF_output(false,false);

    break;
  case PATH_LEAKAGE:
    if (SI4463_is_in_tx_mode())
      SI4463_init_rx();

    enable_ADF_output(true, setting.tracking_output);

#if 0
    enable_direct(false);
    enable_ultra(!setting.atten_step);
#else
    enable_direct(setting.atten_step);
    enable_ultra(true);
#endif
    enable_high(false);

    break;
  case PATH_LOW:
    enable_ultra(false);
    enable_high(false);
    enable_direct(false);
    goto common;

  case PATH_DIRECT:
    enable_ultra(true);
    enable_direct(true);
    enable_high(true);
    enable_ADF_output(false, false);
    goto common2;
  case PATH_ULTRA:
    enable_ultra(true);
    enable_direct(false);
    enable_high(false);
  common:
    enable_ADF_output(true, setting.tracking_output);
  common2:
    enable_rx_output(!setting.atten_step);
    if (!SI4463_is_in_tx_mode())
      SI4463_init_tx();
    break;
  }

}

static void calculate_static_correction(void);
void set_input_path(freq_t f)
{
  LO_harmonic = false;
  if (force_signal_path) {
    setting.extra_lna = test_path & 0x01;
    switch ((test_path & 0xFE)>>1) {
    case 0: signal_path = PATH_LOW; break;
    case 1: signal_path = PATH_ULTRA; break;
    case 2: signal_path = PATH_DIRECT; break;
    case 3: signal_path = PATH_ULTRA; LO_harmonic = true; break;
    }
  }
  else if (MODE_HIGH(setting.mode))
      signal_path = PATH_HIGH;
  else if (direct_test && f >= 900000000 && f < 1100000000)
    signal_path = PATH_DIRECT;
  else if (config.direct && f >= config.direct_start && f < config.direct_stop)
    signal_path = PATH_DIRECT;
  else if(config.ultra && ((config.ultra_start == ULTRA_AUTO && f > ultra_start) || (config.ultra_start != ULTRA_AUTO && f >config.ultra_start))) {
    LO_harmonic = (f > (config.harmonic_start?config.harmonic_start:ULTRA_MAX_FREQ));
    signal_path = PATH_ULTRA;
  } else
    signal_path = PATH_LOW;

  if (signal_path == PATH_HIGH) {
    return;                 // TODO setup high input path
  }
  enable_rx_output(setting.atten_step);

  switch(signal_path) {
  case PATH_LOW:
    enable_ultra(false);
    enable_direct(false);
    enable_high(false);
    enable_extra_lna(setting.extra_lna);
    goto common;

  case PATH_DIRECT:
    enable_ultra(true);
    enable_direct(true);
    enable_high(true);
    enable_extra_lna(setting.extra_lna);
    if (setting.tracking_output)
      enable_ADF_output(true, true);
    else
      enable_ADF_output(false, false);
    TRACE(1);

    goto common2;
  case PATH_ULTRA:
    enable_ultra(true);
    enable_high(false);
    enable_direct(false);
    enable_extra_lna(setting.extra_lna);
    common:
    enable_ADF_output(true, setting.tracking_output);
    common2:
    if (SI4463_is_in_tx_mode())
      SI4463_init_rx();
    break;
  }
  if (force_signal_path)
    calculate_static_correction();
}
#endif

#else
const int8_t drive_dBm [16] = {-38, -32, -30, -27, -24, -19, -15, -12, -5, -2, 0, 3, 6, 9, 12, 16};
#define SWITCH_ATTENUATION  (29 - config.switch_offset)
#define RECEIVE_SWITCH_ATTENUATION  (24 + config.receive_switch_offset)
//#define POWER_OFFSET    15
#define MAX_DRIVE   (setting.mode == M_GENHIGH ? 13 : 11)  // The value of 13 is linked to the SL_GENHIGH_LEVEL_MAX of 9
#define MIN_DRIVE   8
#define SL_GENHIGH_LEVEL_MIN    -38
#define SL_GENHIGH_LEVEL_MAX    9
#define SL_GENLOW_LEVEL_MIN    -76
#define SL_GENLOW_LEVEL_MAX   -7
#define MAX_ATTENUATE   31.5
#endif



//float level_min;
//float level_max;
//float level_range;

float channel_power[3];
float channel_power_watt[3];
volatile float flatness;

//int setting.refer = -1;  // Off by default
const uint32_t reffer_freq[] = {30000000, 15000000, 10000000, 4000000, 3000000, 2000000, 1000000};

#ifdef TINYSA3
const freq_t fh_low[] = {  240000000, 480000000, 720000000, 960000000, 1200000000 };
const freq_t fh_high[] = { 480000000, 960000000, 1920000000, 2880000000, 3840000000 };
#endif

uint8_t in_selftest = false;
uint8_t ignore_stored = false;
uint8_t in_step_test = false;
uint8_t in_calibration = false;
uint8_t calibration_stage;

void update_min_max_freq(void)
{
  switch(setting.mode) {
  case M_LOW:
    minFreq = 0;
#ifdef __ULTRA__
    if (config.ultra)
#ifdef TINYSA4
      if (setting.harmonic)
        maxFreq = setting.harmonic * MAX_LO_FREQ - DEFAULT_IF; // ULTRA_MAX_FREQ;  // make use of harmonic mode above ULTRA_MAX_FREQ
      else
        maxFreq = ULTRA_MAX_FREQ;
#else
      maxFreq = 3000000000; // ULTRA_MAX_FREQ;  // make use of harmonic mode above ULTRA_MAX_FREQ
#endif
    else
#endif
    maxFreq = NORMAL_MAX_FREQ;
#ifdef TINYSA4
    plot_printf(range_text, sizeof range_text, "%QHz to %QHz", minFreq, maxFreq);
#endif
    break;
  case M_GENLOW:
    minFreq = 0;
#ifdef TINYSA4
#ifdef __ULTRA_OUT__
    if (setting.mixer_output) {
      if (setting.harmonic)
        maxFreq = setting.harmonic * MAX_LO_FREQ - DEFAULT_IF; // ULTRA_MAX_FREQ;  // make use of harmonic mode above ULTRA_MAX_FREQ
      else
        maxFreq = ULTRA_MAX_FREQ+60000000; // Add 60MHz to go to 5.40GHz
    } else {
      if (setting.harmonic)
        maxFreq = setting.harmonic * (MAX_LO_FREQ+config.overclock); // ULTRA_MAX_FREQ;  // make use of harmonic mode above ULTRA_MAX_FREQ
      else
        maxFreq = MAX_LO_FREQ+config.overclock; // 4.4GHz
    }
#else
    maxFreq = MAX_LOW_OUTPUT_FREQ;
#endif
#else
   maxFreq = NORMAL_MAX_FREQ;
#endif
    break;
  case M_HIGH:
    minFreq = HIGH_MIN_FREQ_MHZ * 1000000;
    maxFreq = HIGH_MAX_FREQ_MHZ * 1000000;
#ifdef __HARMONIC__
#ifdef TINYSA3                  // different haemonics processing
    if (setting.harmonic) {
      minFreq = setting.harmonic * HIGH_MIN_FREQ_MHZ * 1000000;
      if (setting.harmonic < 4)
        maxFreq = setting.harmonic * HIGH_MAX_FREQ_MHZ * 1000000;
      else
        maxFreq = 2880000000;
    }
    if (get_sweep_frequency(ST_START) < minFreq)
      set_sweep_frequency(ST_START, minFreq);
    if (get_sweep_frequency(ST_STOP) > maxFreq)
      set_sweep_frequency(ST_STOP, maxFreq);
#endif
#endif
    break;
  case M_GENHIGH:
#ifdef TINYSA4 
#ifndef __NEW_SWITCHES__
    if (!config.high_out_adf4350) {
      minFreq =  136000000;
      maxFreq = 1150000000U;
    } else
#endif
    {
      minFreq =  136000000;
      maxFreq = MAX_LO_FREQ;
    }
#else
    minFreq = 240000000;
    maxFreq = 960000000;
#endif
    break;
  }
#ifdef TINYSA4
    plot_printf(range_text, sizeof range_text, "%.3QHz to %.3QHz", minFreq, maxFreq);
#endif
}

void reset_settings(int m)
{
//  strcpy((char *)spi_buffer, dummy);
  setting.mode = m;
  setting.sweep = false;
  disable_waterfall();
  setting.level_meter = false;
  setting.pulse = false;
#ifdef __BANDS__
  setting.multi_band = false;
  setting.multi_trace = false;
  for (int i = 0; i<BANDS_MAX;i++) {
    setting.bands[i].start = 0;
    setting.bands[i].end = 0;
    setting.bands[i].enabled = false;
    setting.bands[i].level = 0;
  }
#endif
#ifdef __ULTRA__
  ultra_start = (config.ultra_start == ULTRA_AUTO ? ULTRA_THRESHOLD : config.ultra_start);
#endif
#ifdef TINYSA4
  drive_dBm = (float *) (setting.mode == M_GENHIGH ? adf_drive_dBm : si_drive_dBm);
  setting.exp_aver = 1;
  setting.increased_R = false;
  setting.mixer_output = true;
#endif
  update_min_max_freq();
  force_signal_path = false;
  setting.frequency_var = 0;
  sweep_mode |= SWEEP_ENABLE;
  setting.unit_scale_index = 0;
  setting.unit_scale = 1;
  setting.unit = U_DBM;
  set_scale(10);
  set_reflevel(-10);
  setting.level_sweep = 0.0;
  setting.attenuate_x2 = 0;         // These should be initialized consistently
  setting.rx_drive=MAX_DRIVE;              // And this
  setting.atten_step = 0;           // And this, only used in low output mode
  setting.rbw_x10 = 0;
  for (int t=0;t<TRACES_MAX;t++) {
    setting.average[t] = 0;
    setting.stored[t] = false;
    setting.subtract[t] = 0;        // Disabled
    setting.normalized[t] = false;        // Disabled
  }
  for (int r=0;r<REFERENCE_MAX;r++)
    for (int l=0;l<LIMITS_MAX;l++)
      setting.limits[r][l].enabled = false;
  if (in_selftest) {
    setting.stored[TRACE_STORED] = true;
    TRACE_ENABLE(TRACE_STORED_FLAG);
  } else
#ifdef TINYSA4
    TRACE_DISABLE(TRACE_STORED_FLAG|TRACE_TEMP_FLAG|TRACE_STORED2_FLAG);
#else
  TRACE_DISABLE(TRACE_STORED_FLAG|TRACE_TEMP_FLAG);
#endif
#ifdef TINYSA4  
  setting.harmonic = 3;         // Automatically used when above ULTRA_MAX_FREQ
#else
#ifdef __ULTRA__
  setting.harmonic = 3;
#else
  setting.harmonic = 0;
#endif
#endif
#ifdef __DRAW_LINE__
  setting.draw_line = false;
#endif
//  setting.show_stored = 0;
  setting.auto_attenuation = false;
  setting.normalize_level = 0.0;
  setting.normalized_trace = -1;

#ifdef TINYSA4
  setting.lo_drive=5;
#else
  setting.lo_drive=13;
//  setting.rx_drive=8;		moved to top
//  setting.atten_step = 0;     moved to top
#endif
  setting.agc = S_AUTO_ON;
  setting.lna = S_AUTO_OFF;
  setting.tracking = false;
  setting.modulation = MO_NONE;
  setting.modulation_frequency = 1000;
#ifdef TINYSA4
  setting.modulation_depth_x100 = 80; // %80
  setting.modulation_deviation_div100 = 30; // 3kHz
#endif
  setting.step_delay = 0;
  setting.offset_delay = 0;
  setting.step_delay_mode = SD_NORMAL;
  setting.vbw_x100 = 0;      // Auto mode
  setting.repeat = 1;
  setting.auto_reflevel = true;     // Must be after SetReflevel
  setting.decay=20;
  setting.attack=1;
  setting.noise=5;
  setting.below_IF = S_AUTO_OFF;
  setting.tracking_output = false;
  setting.measurement = M_OFF;
#ifdef __ULTRA__
//  setting.ultra = S_AUTO_OFF;
#endif
#ifdef TINYSA4
  setting.frequency_IF = config.frequency_IF1; ;
#else
  setting.frequency_IF = DEFAULT_IF;
#endif
  setting.frequency_offset = FREQUENCY_SHIFT;
  setting.auto_IF = true;
  //set_external_gain(0.0);  // This also updates the help text!!!!!
  //setting.external_gain = 0.0;
  setting.trigger = T_AUTO;
  setting.trigger_direction = T_UP;
  setting.trigger_mode = T_MID;
  setting.fast_speedup = 0;
  setting.faster_speedup = 0;
  setting.trigger_level = -150.0;
#ifdef __TRIGGER_TRACE__
  setting.trigger_trace = 255;
#endif
  setting.linearity_step = 0;
//  setting.refer = -1;             // do not reset reffer when switching modes
  setting.mute = true;
#ifdef __SPUR__
#ifdef __ULTRA__
  if (m == M_LOW)
    setting.spur_removal = S_AUTO_OFF;
  else
    setting.spur_removal = S_OFF;
#else
  setting.spur_removal = S_OFF;
#endif
  setting.mirror_masking = false;
  setting.slider_position = 0;
  setting.slider_span = 100000;
#endif		// __SPUR__
  switch(m) {
  case M_LOW:
    set_sweep_frequency(ST_START, minFreq);
    set_sweep_frequency(ST_STOP, maxFreq);
#ifdef TINYSA4
    set_sweep_frequency(ST_STOP,  NORMAL_MAX_FREQ);    // TODO <----------------- temp ----------------------
    setting.attenuate_x2 = 0;
#else
    setting.attenuate_x2 = 60;
#endif
    setting.auto_attenuation = true;
    setting.sweep_time_us = 0;
#ifdef TINYSA4
    setting.lo_drive=5;
    setting.extra_lna = false;
#endif
//    setting.correction_frequency = config.correction_frequency[CORRECTION_LOW_IN];
//    setting.correction_value = config.correction_value[CORRECTION_LOW_IN];
    break;
  case M_GENLOW:
#ifdef TINYSA4
    setting.rx_drive= MAX_DRIVE;
    if (max2871)
      setting.lo_drive= 2;
    else
      setting.lo_drive= 1;
#else
//    setting.rx_drive=8;
	setting.lo_drive=13;
#endif
    set_sweep_frequency(ST_CENTER, 10000000);
    set_sweep_frequency(ST_SPAN, 0);
    setting.sweep_time_us = 2*ONE_SECOND_TIME;
    setting.step_delay_mode = SD_FAST;
#ifdef TINYSA4
    setting.extra_lna = false;
//    setting.correction_frequency = config.correction_frequency[CORRECTION_LOW_OUT];
//    setting.correction_value = config.correction_value[CORRECTION_LOW_OUT];
#else
//    setting.correction_frequency = config.correction_frequency[CORRECTION_LOW_IN];
//    setting.correction_value = config.correction_value[CORRECTION_LOW_IN];
#endif
//    level_min = SL_GENLOW_LEVEL_MIN + LOW_OUT_OFFSET;
//    level_max = SL_GENLOW_LEVEL_MAX + LOW_OUT_OFFSET;
//    level_range = level_max - level_min;
    break;
  case M_HIGH:
    set_sweep_frequency(ST_START, minFreq);
    set_sweep_frequency(ST_STOP,  maxFreq);
    setting.sweep_time_us = 0;
#ifdef TINYSA4
    setting.extra_lna = false;
#endif
//    setting.correction_frequency = config.correction_frequency[CORRECTION_HIGH_IN];
//    setting.correction_value = config.correction_value[CORRECTION_HIGH_IN];
    break;
  case M_GENHIGH:
#ifdef TINYSA4
	setting.lo_drive = MIN_DRIVE;
	setting.level = drive_dBm[setting.lo_drive]+ config.high_level_output_offset;
    set_sweep_frequency(ST_CENTER, (minFreq + maxFreq)/2 );
    setting.extra_lna = false;
#else
    setting.lo_drive=8;
    set_sweep_frequency(ST_CENTER, 300000000);
#endif
    set_sweep_frequency(ST_SPAN, 0);
    setting.sweep_time_us = 2*ONE_SECOND_TIME;
    setting.step_delay_mode = SD_FAST;
//    setting.correction_frequency = config.correction_frequency[CORRECTION_HIGH_IN];
//    setting.correction_value = config.correction_value[CORRECTION_HIGH_IN];
//    level_min = SL_GENHIGH_LEVEL_MIN + config.high_level_output_offset;
//    level_max = SL_GENHIGH_LEVEL_MAX + config.high_level_output_offset;
//    level_range = level_max - level_min;
    break;
  }
  setting.level =  level_max();     // This is the level with above settings.
  markers_reset();
  setting._active_marker = 0;
  //set_external_gain(0.0);  // This also updates the help text!!!!! Must be below level_min and level_max being set
  set_sweep_points(POINTS_COUNT);
  dirty = true;
}

uint32_t calc_min_sweep_time_us(void)         // Estimate minimum sweep time in uS,  needed to calculate the initial delays for the RSSI before first sweep
{
  uint32_t t;
  if (MODE_OUTPUT(setting.mode))
    t = 200*sweep_points;                   // 200 microseconds is the delay set in perform when sweeping in output mode
  else {
    uint32_t bare_sweep_time=0;
    bare_sweep_time = (SI4432_step_delay + MEASURE_TIME) * (sweep_points); // Single RSSI delay and measurement time in uS while scanning
    if (FREQ_IS_CW()) {
      bare_sweep_time = MINIMUM_SWEEP_TIME;       // minimum sweep time in fast CW mode
      if (setting.repeat != 1 || setting.sweep_time_us >= 100*ONE_MS_TIME || S_STATE(setting.spur_removal)) // if no fast CW sweep possible
        bare_sweep_time = 15000;       // minimum CW sweep time when not in fast CW mode
    }
    t = vbwSteps * (S_STATE(setting.spur_removal) ? 2 : 1) * bare_sweep_time ;           // factor in vbwSteps and spur impact
    t += (setting.repeat - 1)* REPEAT_TIME * (sweep_points);            // Add time required for repeats
  }
  return t;
}


void set_refer_output(int v)
{
  setting.refer = v;
  set_calibration_freq(setting.refer);
  dirty = true;
}

void set_decay(int d)
{
  if (d < 0 || d > 1000000)
    return;
  if (setting.frequency_step == 0) {        // decay in ms
    d = (float)d * 500.0 * (float)sweep_points / (float)setting.actual_sweep_time_us;
  }
  setting.decay = d;
  dirty = true;
}

#ifdef __QUASI_PEAK__
void set_attack(int d)
{
  if (d < 0 || d > 20000)
    return;
  if (setting.frequency_step == 0 && d>0) {        // decay in ms
    d = (float)d * 500.0 * (float)sweep_points  / (float)setting.actual_sweep_time_us;
  }
  setting.attack = d;
  dirty = true;
}
#endif

void set_noise(int d)
{
  if (d < 2 || d > 120)
    return;
  setting.noise = d;
  dirty = true;
}

void set_gridlines(int d)
{
  if (d != 0 && (d < 3 || d > 20))
    return;
  config.gridlines = d;
  config_save();
  dirty = true;
  update_grid();
}

#ifdef TINYSA4

int set_freq_corr(int f)
{
  if (f < - 100000 || f > +100000)
    return -1;
  config.setting_frequency_30mhz = 3000000000 + f * 3 ;
  ADF4351_recalculate_PFDRFout();
  config_save();
  dirty = true;
  update_grid();
  return 0;
}

int set_actual_freq(freq_t f)
{
  if (f < 3000000000 - 30000 || f > 3000000000 + 30000)
    return -1;
  config.setting_frequency_30mhz =f;
  ADF4351_recalculate_PFDRFout();
  config_save();
  dirty = true;
  update_grid();
  return 0;
}
#else
void set_10mhz(freq_t f)
{
  if (f < 9000000 || f > 11000000)
    return;
  config.setting_frequency_10mhz = f;
  config_save();
  dirty = true;
  update_grid();
}
#endif

#if 0
static setting_t saved_setting;
#endif

void set_measurement(int m)
{
#ifdef __LINEARITY__
  setting.stored[TRACE_STORED] = true;
  if (m == M_LINEARITY) {
    for (int j = 0; j < setting._sweep_points; j++)
      stored_t[j] = -150;
    setting.linearity_step = 0;
    setting.attenuate_x2 = 29*2;
    setting.auto_attenuation = false;
  }
#endif
#ifdef __FFT_DECONV__
  if (m == M_DECONV && sweep_points == 256) {
    set_storage();
    set_reflevel(-20);
  } else
    return;
#endif
  setting.measurement = m;
  dirty = true;
}
void set_lo_drive(int d)
{
  setting.lo_drive = d;
  dirty = true;
}

void set_rx_drive(int d)
{
  setting.rx_drive = d;
  dirty = true;
}

void set_level_sweep(float l)
{
  setting.level_sweep = l;
  dirty = true;
}

void set_sweep_time_us(uint32_t t)          // Set the sweep time as the user wants it to be.
{
//  if (t < MINIMUM_SWEEP_TIME)             // Sweep time of zero means sweep as fast as possible
//    t = MINIMUM_SWEEP_TIME;
  if (t > MAXIMUM_SWEEP_TIME)
    t = MAXIMUM_SWEEP_TIME;
  setting.sweep_time_us = t;
//  if (MODE_OUTPUT(setting.mode))
//    setting.actual_sweep_time_us = t;       // To ensure time displayed is correct before first sweep is completed
#if 0
  uint32_t ta = calc_min_sweep_time_us();   // Can not be faster than minimum sweep time
  if (ta < t)
    ta = t;
  setting.actual_sweep_time_us = ta;
  if (FREQ_IS_CW())
    update_grid();            // Really only needed in zero span mode
  redraw_request |= REDRAW_FREQUENCY;
#endif
  dirty = true;
}

void set_tracking_output(int t)
{
  setting.tracking_output = t;
  dirty = true;
}

//#ifndef TINYSA4
void toggle_tracking_output(void)
{
  setting.tracking_output = !setting.tracking_output;
  dirty = true;
}
//#endif

void toggle_pulse(void)
{
  setting.pulse = !setting.pulse;
  dirty = true;
}

#ifdef __DRAW_LINE__
void toggle_draw_line(void)
{
  setting.draw_line = !setting.draw_line;
  dirty = true;
}
#endif

void toggle_debug_avoid(void)
{
  debug_avoid = !debug_avoid;
  if (debug_avoid) {
    TRACE_ENABLE(TRACE_STORED_FLAG|TRACE_TEMP_FLAG);
    setting.stored[TRACE_STORED] = true;
    setting.stored[TRACE_TEMP] = true;
  } else {
    TRACE_DISABLE(TRACE_STORED_FLAG|TRACE_TEMP_FLAG);
    setting.stored[TRACE_STORED] = false;
    setting.stored[TRACE_TEMP] = false;
  }
  dirty = true;
}
#ifdef __ULTRA__
void toggle_debug_spur(void)
{
  debug_spur = !debug_spur;
  if (debug_spur) {
    TRACE_ENABLE(TRACE_STORED_FLAG|TRACE_TEMP_FLAG);
    setting.stored[TRACE_STORED] = true;
    setting.stored[TRACE_TEMP] = true;
  } else {
    TRACE_DISABLE(TRACE_STORED_FLAG|TRACE_TEMP_FLAG);
    setting.stored[TRACE_STORED] = false;
    setting.stored[TRACE_TEMP] = false;
  }
  dirty = true;
}
#endif

#ifdef TINYSA4
void toggle_debug_level(void)
{
  debug_level = !debug_level;
  dirty = true;
}


#ifndef __NEW_SWITCHES__
void toggle_high_out_adf4350(void)
{
  config.high_out_adf4350 = !config.high_out_adf4350;
  drive_dBm = (float *) (config.high_out_adf4350 ? adf_drive_dBm : si_drive_dBm);
  config_save();
  dirty = true;
}
#endif

void toggle_extra_lna(void)
{
  setting.extra_lna = !setting.extra_lna;
#ifdef TINYSA4_4
  if (setting.extra_lna)
    setting.attenuate_x2 = 0;
#endif
  set_extra_lna(setting.extra_lna);
}

void set_extra_lna(int t)
{
  setting.extra_lna = t;
#if 0
  if (setting.extra_lna) {
    setting.correction_frequency = config.correction_frequency[CORRECTION_LNA];
    setting.correction_value = config.correction_value[CORRECTION_LNA];
  } else {
    setting.correction_frequency = config.correction_frequency[CORRECTION_LOW_IN];
    setting.correction_value = config.correction_value[CORRECTION_LOW_IN];
  }
#endif
  dirty = true;
}
#endif

void toggle_mirror_masking(void)
{
  setting.mirror_masking = !setting.mirror_masking;
#ifdef __HARMONIC__
#ifdef TINYSA3
#ifndef __ULTRA
  if (setting.harmonic) {
    setting.spur_removal = setting.mirror_masking;
  }
#endif
#endif
#endif
  dirty = true;
}

void toggle_mute(void)
{
  setting.mute = !setting.mute;
  dirty = true;
}

void toggle_hambands(void)
{
  config.hambands = !config.hambands;
  dirty = true;
}

void toggle_below_IF(void)
{
  if (S_IS_AUTO(setting.below_IF ))
    setting.below_IF = false;
  else if (setting.below_IF)
    setting.below_IF = S_AUTO_OFF;
  else
    setting.below_IF = true;
  dirty = true;
}

void set_modulation(int m)
{
  setting.modulation = m;
  dirty = true;
}

#ifdef TINYSA4
void set_deviation(int d)
{
  setting.modulation_deviation_div100  = d/100;
  dirty = true;
}

void set_depth(int d)
{
  setting.modulation_depth_x100  = d;
  dirty = true;
}
#endif

void set_modulation_frequency(float f)
{
  if (f < 1)
    f = 1;
  if (f > 10000)
    f = 10000;
#ifdef TINYSA4
  if (setting.modulation == MO_WFM && f > 3500)
    f = 3500;
#endif
  setting.modulation_frequency = f;
  dirty = true;
}

void set_repeat(int r)
{
  if (r > 0 && r <= 10000) {
    setting.repeat = r;
//    dirty = true;             // No HW update required, only status panel refresh
  }
}

void set_IF(int f)
{
  if (f == 0) {
    setting.auto_IF = true;
#ifdef TINYSA4
    setting.frequency_IF = config.frequency_IF1;
#endif
  } else {
    setting.auto_IF = false;
    setting.frequency_IF = f;
  }
  dirty = true;
}

#ifdef TINYSA4
void set_IF2(int f)
{

  config.frequency_IF2 = f;
  config_save();
  dirty = true;
}

void set_R(int f)
{
  setting.R = f;
  if (f<0) {
    f = -f;
    ADF4351_R_counter(-(f % 1000));
  } else
    ADF4351_R_counter(f % 1000);
  ADF4351_spur_mode(f/1000);
  dirty = true;
}

uint32_t local_modulo = 0;

void set_modulo(uint32_t f)
{
  local_modulo = f;
  ADF4351_modulo(f);
  clear_frequency_cache();
  dirty = true;
}
#endif

void set_auto_attenuation(void)
{
  setting.auto_attenuation = true;
  if (setting.mode == M_LOW) {
#ifdef TINYSA4_4
    if (setting.extra_lna)
      setting.attenuate_x2 = 0;
    else
#endif
      setting.attenuate_x2 = 60;
  } else {
    setting.attenuate_x2 = 0;
  }
  setting.atten_step = false;
  dirty = true;
}

void set_auto_reflevel(bool v)
{
  setting.auto_reflevel = v;
}

#if 1
float level_min(void)
{
  int l;
  if (setting.mode == M_GENLOW)
    l = SL_GENLOW_LEVEL_MIN + LOW_OUT_OFFSET;
  else
    l = SL_GENHIGH_LEVEL_MIN + config.high_level_output_offset;
  return l;
}

float level_max(void)
{
  if (setting.mode == M_GENLOW)
    return SL_GENLOW_LEVEL_MAX + LOW_OUT_OFFSET;
  else
    return SL_GENHIGH_LEVEL_MAX + config.high_level_output_offset;
}

float level_range(void)
{
  int r;
  r = level_max() - level_min();
  return r;
}
#endif


#ifdef TINYSA4
float low_out_offset(void)
{
  if (config.output_is_calibrated)
    return config.low_level_output_offset;
//  if (config.input_is_calibrated)
//    return - config.low_level_offset;
  return 0;
}
#endif

static pureRSSI_t get_signal_path_loss(void){  //  loss as positive number
#ifdef TINYSA4
  if (setting.mode == M_LOW)
    return float_TO_PURE_RSSI(+9.3);      // Loss in dB, -9.5 for v0.1, -12.5 for v0.2
  return float_TO_PURE_RSSI(+29);          // Loss in dB (+ is gain)
#else
  if (setting.mode == M_LOW)
    return float_TO_PURE_RSSI(-5.5);      // Loss in dB, -9.5 for v0.1, -12.5 for v0.2
  return float_TO_PURE_RSSI(+7);          // Loss in dB (+ is gain)
#endif  
}

void correct_high_output_level(void)
{
  float a = setting.level - level_max();
#ifdef TINYSA4
#ifndef __NEW_SWITCHES__
  if (!config.high_out_adf4350) {
    float dt = Si446x_get_temp() - CENTER_TEMPERATURE;
    if (dt > 0)
      a += dt * DB_PER_DEGREE_ABOVE;  // Temperature correction
    else
      a += dt * DB_PER_DEGREE_BELOW;  // Temperature correction
  }
#endif
#endif
  if (a <= -SWITCH_ATTENUATION) {
    setting.atten_step = true;
    a = a + SWITCH_ATTENUATION;
  } else {
    setting.atten_step = false;
  }

  int d = MIN_DRIVE;
  while (drive_dBm[d] - level_max() < a && d < MAX_DRIVE)       // Find level equal or above requested level
    d++;
  //    if (d == 8 && v < -12)  // Round towards closest level
  //      d = 7;
  setting.level = drive_dBm[d] + config.high_level_output_offset - (setting.atten_step ? SWITCH_ATTENUATION : 0);
}

void set_level(float v)     // Set the output level in dB  in high/low output
{
#if 1
  if (v < level_min())
    v = level_min();
  if (v > level_max())
    v = level_max();
#else
  if (setting.mode == M_GENHIGH) {
    v -= config.high_level_output_offset;
    if (v < SL_GENHIGH_LEVEL_MIN)
      v = SL_GENHIGH_LEVEL_MIN;
    if (v > SL_GENHIGH_LEVEL_MAX)
      v = SL_GENHIGH_LEVEL_MAX;
    v += config.high_level_output_offset;
  } else {                  // This MUST be low output level
    v -= LOW_OUT_OFFSET;
    if (v < SL_GENLOW_LEVEL_MIN)
      v = SL_GENLOW_LEVEL_MIN;
    if (setting.modulation == MO_AM) {
//    if (v > SL_GENLOW_LEVEL_MAX -3)
//      v = SL_GENLOW_LEVEL_MAX - 3;
    } else
      if (v > SL_GENLOW_LEVEL_MAX)
        v = SL_GENLOW_LEVEL_MAX;

    v += LOW_OUT_OFFSET;
//    set_attenuation(setting.level - LOW_OUT_OFFSET);
  }
#endif
  setting.level = v;
#ifdef TINYSA4
  set_output_path(get_sweep_frequency(ST_CENTER), v);
#endif
  if (setting.mode == M_GENHIGH) correct_high_output_level();
  dirty = true;
}

float get_level(void)
{
#if 0
  if (setting.mode == M_GENHIGH) {
    return v; // drive_dBm[setting.lo_drive] + config.high_level_output_offset;
  } else {
//    setting.level = get_attenuation() + LOW_OUT_OFFSET;
    return setting.level;
  }
#endif
  return setting.level;
}


float get_attenuation(void)
{
  float actual_attenuation = setting.attenuate_x2 / 2.0;
  if (setting.mode == M_GENLOW) {
    return (float)( level_max() - actual_attenuation  + BELOW_MAX_DRIVE(setting.rx_drive) - ( setting.atten_step ? SWITCH_ATTENUATION : 0) );
  } else if (setting.atten_step) {
    if (setting.mode == M_LOW)
      return actual_attenuation + RECEIVE_SWITCH_ATTENUATION;
    else
      return actual_attenuation + SWITCH_ATTENUATION;
  }
  return(actual_attenuation);
}

void set_attenuation(float a)       // Is used both only in  high/low input mode
{
#if 0
  if (setting.mode == M_GENLOW) {
    a = a - level_max();               // Move to zero for max power
    if (a > 0)
      a = 0;
    if( a <  - SWITCH_ATTENUATION) {
      a = a + SWITCH_ATTENUATION;
      setting.atten_step = 1;
    } else {
      setting.atten_step = 0;
    }
    setting.rx_drive = MAX_DRIVE;        // Reduce level till it fits in attenuator range
    while (a - BELOW_MAX_DRIVE(setting.rx_drive) < - 31 && setting.rx_drive > MIN_DRIVE) {
      setting.rx_drive--;
    }
    a -= BELOW_MAX_DRIVE(setting.rx_drive);
    a = -a;
  } else
#endif
#ifdef USE_INPUT_STEP_ATTENUATE
  if (setting.mode == M_LOW && a > MAX_ATTENUATE) {
    setting.atten_step = 1;
    a = a - RECEIVE_SWITCH_ATTENUATION;
  } else
#endif
  if (setting.mode == M_HIGH && a > 0) {
    setting.atten_step = 1;
    a = a - SWITCH_ATTENUATION;
  } else
  {
    setting.atten_step = 0;
    setting.auto_attenuation = false;
  }
  dirty = true;
  if (a<0.0)
      a = 0;
  if (a> MAX_ATTENUATE)
    a = MAX_ATTENUATE;
  if (setting.mode == M_HIGH     // No attenuator in high mode
#ifdef TINYSA4
      || (setting.mode == M_LOW && setting.extra_lna)    // No attenuator with LNA
#endif
      )
    a = 0;
  if (setting.attenuate_x2 == a*2)
    return;
  setting.attenuate_x2 = a*2;
  dirty = true;
}

#ifdef __LIMITS__
void limits_update(void)
{
  bool redraw = false;
  for (int t=0;t<TRACES_MAX;t++) {
    int j =0;
    int prev = -1;
    if (setting.average[t] != AV_TABLE)
      continue;
    int count = 0;
    for (int i = 0; i<LIMITS_MAX; i++)
    {
      if (setting.limits[t][i].enabled) {
        count++;
        while (j < sweep_points && (getFrequency(j) < setting.limits[t][i].frequency /* || setting.limits[t][i].frequency == 0 */)) {
          if (prev < 0)
            measured[t][j] = setting.limits[t][i].level;
          else
            measured[t][j] = setting.limits[t][prev].level +
            (getFrequency(j) - setting.limits[t][prev].frequency) * (setting.limits[t][i].level - setting.limits[t][prev].level) /
            (setting.limits[t][i].frequency-setting.limits[t][prev].frequency);
          j++;
        }
        prev = i;
      }
    }
    if (prev>=0)
    {
      while (j < sweep_points)
        measured[t][j++] = setting.limits[t][prev].level;
      setting.stored[t] = true;
      TRACE_ENABLE(1<<t);
    } else {
      setting.stored[t] = false;
      TRACE_DISABLE(1<<t);
    }
    redraw_request|= REDRAW_AREA;
    redraw = true;
  }
  if (redraw) {
    plot_into_index(measured);
    draw_all(true);
  }
}
#endif

void copy_trace(int f, int t)
{
  if (f == t)
    return;
  for (int i=0; i<POINTS_COUNT;i++)
    measured[t][i] = measured[f][i];
  setting.stored[t] = true;
  //dirty = true;             // No HW update required, only status panel refresh
}

void store_trace(int f, int t)
{
  copy_trace(f,t);
  enableTracesAtComplete(1<<t);
  //dirty = true;             // No HW update required, only status panel refresh
}

void set_clear_storage(void)
{
//  setting.show_stored = false;
//  setting.subtract = false;
  TRACE_DISABLE(TRACE_STORED_FLAG);
  // dirty = true;             // No HW update required, only status panel refresh
}

void set_subtract_storage(void)
{
  /*
  if (!setting.subtract_stored) {
    if (!setting.show_stored)
      store_trace(0,2);
    setting.subtract_stored = true;
    setting.normalize_level = 0.0;
//    setting.auto_attenuation = false;
  } else {
    setting.subtract_stored = false;
  }
  //dirty = true;             // No HW update required, only status panel refresh
  */
}

void subtract_trace(int t, int f)
{
  if (t == f)
    return;
  if (!setting.subtract[t]) {
    setting.subtract[t] = f+1;
    setting.normalize_level = 0.0;
    setting.auto_attenuation = false;       // Otherwise noise level may move leading to strange measurements
    for (int i=0;i<POINTS_COUNT;i++)
       measured[t][i] -= measured[f][i];                   // pre-load AVER

  } else {
    for (int i=0;i<POINTS_COUNT;i++)
      measured[t][i] += measured[f][i];                   // pre-load AVER
    setting.subtract[t] = 0;
  }
}

void toggle_normalize(int t)
{
  if (!setting.normalized[t]) {
    if (setting.normalized_trace == -1) {
      copy_trace(t,TRACE_TEMP);
      setting.normalized_trace = t;
      TRACE_DISABLE(1<<TRACE_TEMP);
    }
    setting.normalized[t] = true;
    for (int i=0;i<POINTS_COUNT;i++)
      measured[t][i] -= measured[TRACE_TEMP][i];                   // pre-load AVER
    setting.auto_attenuation = false;       // Otherwise noise level may move leading to strange measurements
    setting.normalize_level = 0.0;
  } else {
    for (int f=0; f<TRACES_MAX-1;f++) {
      if (setting.normalized[f] && (setting.normalized_trace == t || f == t)) {
        for (int i=0;i<POINTS_COUNT;i++)
          measured[f][i] += measured[TRACE_TEMP][i];                   // pre-load AVER
        setting.normalized[f] = false;
      }
    }
    if (setting.normalized_trace == t) {
      setting.normalized_trace = -1;
      setting.stored[TRACE_TEMP] = false;
    }
  }
}


extern float peakLevel;
float set_actual_power(float target_level)              // Set peak level to known value
{
  if (!in_calibration && !markers[0].index)
    return 0.0;
  float actual_level = marker_to_value(0);
  float offset_correction = target_level - actual_level  ; // measured[markers[0].trace][markers[0].index];        //  offset based on difference between measured peak level and known peak level
  if (target_level == 100) offset_correction = 0;
  if (setting.mode == M_HIGH) {
    config.high_level_offset += offset_correction;
  } else if (setting.mode == M_LOW) {
#ifdef TINYSA4
//    if (in_calibration && SDU1.config->usbp->state == USB_ACTIVE)
//      shell_printf ("stage=%d, target=%5.2f, actual=%5.2f, correction=%5.2f, old correction=%5.2f\r\n", calibration_stage, target_level, actual_level, offset_correction, get_level_offset());
    if (signal_path == PATH_ULTRA) {
      if (LO_harmonic) {
        if (setting.extra_lna)
          config.harmonic_lna_level_offset += offset_correction;
        else
        config.harmonic_level_offset += offset_correction;
      } else {
        if (setting.extra_lna)
          config.ultra_lna_level_offset += offset_correction;
        else
          config.ultra_level_offset += offset_correction;
      }
    } else if (signal_path == PATH_DIRECT) {
      if (setting.extra_lna)
        config.direct_lna_level_offset += offset_correction;
      else
        config.direct_level_offset += offset_correction;
    } else if (setting.extra_lna)
      config.lna_level_offset += offset_correction;
    else
#endif
    if (setting.atten_step)
      config.receive_switch_offset += offset_correction;
    else
      config.low_level_offset += offset_correction;
  }
  dirty = true;
  config_save();
  return offset_correction;
  // dirty = true;             // No HW update required, only status panel refresh
}

void set_actual_correction_value(int current_curve,int current_curve_index, float actual_level){
  float new_offset = actual_level - marker_to_value(0) + config.correction_value[current_curve][current_curve_index];        // calculate offset based on difference between measured peak level and known peak level
  if (new_offset > -30 && new_offset < 30) {
    config.correction_value[current_curve][current_curve_index] = new_offset;
  }
}

float get_level_offset(void)
{
//  if (setting.disable_correction)
//    return 0;
  if (setting.mode == M_HIGH) {
    if (config.high_level_offset == 100)        // Offset of 100 means not calibrated
      return 0;
    return(config.high_level_offset);
  }
  if (setting.mode == M_LOW) {
//    if (!config.input_is_calibrated)
//      return 0;
    float lev;
#ifdef TINYSA4
    if (signal_path == PATH_DIRECT) {
      if (setting.extra_lna)
        lev = config.direct_lna_level_offset;
      else
        lev = config.direct_level_offset;
    } else if (signal_path == PATH_ULTRA) {
      if (LO_harmonic) {
        if (setting.extra_lna)
          lev = config.harmonic_lna_level_offset;
        else
          lev = config.harmonic_level_offset;
      } else {
        if (setting.extra_lna)
          lev = config.ultra_lna_level_offset;
        else
          lev = config.ultra_level_offset;
      }
    } else if (setting.extra_lna)
      lev = config.lna_level_offset;
    else
#endif
      lev = config.low_level_offset;
    return(lev);
  }
  if (setting.mode == M_GENLOW) {
      return(LOW_OUT_OFFSET);
  }
  if (setting.mode == M_GENHIGH) {
    return(config.high_level_output_offset);
  }
  return(0);
}

int level_is_calibrated(void)
{
  if (setting.mode == M_HIGH && config.high_level_offset != 100)
    return 1;
  if (setting.mode == M_LOW) {
    if (!config.input_is_calibrated)
      return 0;
#ifdef TINYSA4
    if (setting.extra_lna) {
      if (config.lna_level_offset != 100)
        return 1;
    } else
#endif
      if (config.low_level_offset != 100)
        return 1;
  }
  return(0);
}

void set_RBW(uint32_t rbw_x10)
{
  setting.rbw_x10 = rbw_x10;
  update_rbw();
  dirty = true;
}

#ifdef __VBW__
void set_VBW(uint32_t vbw_x100)
{
  setting.vbw_x100 = vbw_x100;
  if (vbw_x100 == 0)
    setting.repeat = 1;
  else
    setting.repeat = vbw_x100;
  dirty = true;
}
#endif

#ifdef __SPUR__
void set_spur(int v)
{
  if (setting.mode!=M_LOW)
    return;
  setting.spur_removal = v;
//  if (setting.spur_removal && actual_rbw > 360)           // moved to update_rbw
//    set_RBW(300);
  dirty = true;
}

void toggle_spur(void)
{
  if (setting.mode!=M_LOW)
    return;
#ifdef TINYSA4
  if (S_IS_AUTO(setting.spur_removal ))
    setting.spur_removal = false;
  else if (setting.spur_removal)
    setting.spur_removal = S_AUTO_OFF;
  else
    setting.spur_removal = true;
#else
  if (S_STATE(setting.spur_removal ))
    setting.spur_removal = S_OFF;
  else
    setting.spur_removal = S_ON;
#endif
  dirty = true;
}
#endif

#ifdef __HARMONIC__
void set_harmonic(int h)
{
  if (h < 0 || h > 5 || h == 1)
    return;
  setting.harmonic = h;
#if 0
  minFreq = 684000000.0;
  if ((freq_t)(setting.harmonic * 135000000)+config.frequency_IF1 >  minFreq)
    minFreq = setting.harmonic * 135000000 + config.frequency_IF1;
#endif
#if 0
  maxFreq = 9900000000.0;
  if (setting.harmonic != 0 && (MAX_LO_FREQ * setting.harmonic + config.frequency_IF1 )< 9900000000.0)
    maxFreq = (MAX_LO_FREQ * setting.harmonic + config.frequency_IF1 );
  set_sweep_frequency(ST_START, minFreq);
  set_sweep_frequency(ST_STOP, maxFreq);
#endif
  update_min_max_freq();
}
#endif

void set_step_delay(int d)                  // override RSSI measurement delay or set to one of three auto modes
{

  if ((SD_MANUAL <= d && d < 10) || d > 30000)         // values 0 (normal scan), 1 (precise scan) and 2(fast scan) have special meaning and are auto calculated
    return;
  if (d <SD_MANUAL) {
    setting.step_delay_mode = d;
    setting.step_delay = 0;
    setting.offset_delay = 0;
  } else {
    setting.step_delay_mode = SD_MANUAL;
    setting.step_delay = d;
  }
  dirty = true;
}

void set_offset_delay(int d)                  // override RSSI measurement delay or set to one of three auto modes
{
 setting.offset_delay = d;
 dirty = true;
}


void set_average(int t, int v)
{
  if (setting.average[t] == v)     // Clear calc on second click
    dirty = true;
  if (setting.average[t] == AV_TABLE && v != AV_TABLE)
    setting.stored[t] = false;
  setting.average[t] = v;
  bool enable = ((v != 0)
#ifdef __QUASI_PEAK__
      && (v != AV_QUASI)
#endif
      );
  dirty = true;
  if (enable) {
    setting.scan_after_dirty[t] = 0;
  }
//  else
//    TRACE_DISABLE(TRACE_TEMP_FLAG);
  //dirty = true;             // No HW update required, only status panel refresh
}

void toggle_LNA(void)
{
  if (S_IS_AUTO(setting.lna ))
    setting.lna = false;
  else if (setting.lna)
    setting.lna = S_AUTO_OFF;
  else
    setting.lna = true;
  dirty = true;
}

void toggle_tracking(void)
{
  setting.tracking = !setting.tracking;
  if (setting.tracking) {
#ifdef TINYSA4
   set_refer_output(0);
    set_sweep_frequency(ST_CENTER, 30000000);
#else
    set_refer_output(2);
    set_sweep_frequency(ST_CENTER, 10000000);
#endif
    set_sweep_frequency(ST_SPAN,    5000000);
  } else {
    set_refer_output(-1);
  }
  dirty = true;
}

void toggle_AGC(void)
{
  if (S_IS_AUTO(setting.agc ))
    setting.agc = false;
  else if (setting.agc)
    setting.agc = S_AUTO_ON;
  else
    setting.agc = true;
  dirty = true;
}

static unsigned char SI4432_old_v[2];

#ifdef __SI4432__
void auto_set_AGC_LNA(int auto_set, int agc)                                                                    // Adapt the AGC setting if needed
{
  unsigned char v;
  if (auto_set)
    v = 0x60; // Enable AGC and disable LNA
  else
    v = 0x40+agc; // Disable AGC and enable LNA
  int idx = MODE_SELECT(setting.mode) == SI4432_RX ? 0 : 1;
  if (SI4432_old_v[idx] != v) {
    SI4432_Sel = MODE_SELECT(setting.mode);
    SI4432_Write_Byte(SI4432_AGC_OVERRIDE, v);
    SI4432_old_v[idx] = v;
  }
#ifdef __SI4463__
  unsigned char v;
  if (auto_set) {
    v = 0x00; // Enable AGC and disable LNA
  } else {
    v = 0xa8+agc; // Disable AGC and enable LNA
  }
  if (SI4432_old_v[0] != v) {
    SI446x_set_AGC_LNA(v);
    SI4432_old_v[0] = v;
  }
#endif
}
#endif

#ifdef __SI4432__
void set_AGC_LNA(void) {
  unsigned char v = 0x40;
  if (S_STATE(setting.agc)) v |= 0x20;
  if (S_STATE(setting.lna)) v |= 0x10;
  SI4432_Write_Byte(SI4432_AGC_OVERRIDE, v);
  int idx = MODE_SELECT(setting.mode) == SI4432_RX ? 0 : 1;
  SI4432_old_v[idx] = v;
}
#endif

#ifdef __SI4463__
void set_AGC_LNA(void) {
  uint8_t v = 0;
  if (!S_STATE(setting.agc))
    v |= 0x80 + 0x20;     // Inverse!!!!
  if (S_STATE(setting.lna))
    v |= 0x0F;     // Inverse!!!!
  SI446x_set_AGC_LNA(v);
  SI4432_old_v[0] = v;
}
#endif

void set_unit(int u)
{
  if (setting.unit == u)
    return;
  float r = to_dBm(setting.reflevel);   // Get neutral unit
  float s = to_dBm(setting.scale);
//  float t = setting.trigger;            // Is always in dBm
  // float m = r - NGRIDSY*s;

  setting.unit = u;                     // Switch unit

  r = value(r);                         // Convert to target unit
  s = value(s);
  if (UNIT_IS_LINEAR(setting.unit)) {
    if (r < REFLEVEL_MIN)
      r = REFLEVEL_MIN;                          // Minimum value to ensure display
    if (r >REFLEVEL_MAX)
      r = REFLEVEL_MAX;                          // Maximum value
    set_scale(r/NGRIDY);
    set_reflevel(setting.scale*NGRIDY);
#ifdef __SI4432__
    if (S_IS_AUTO(setting.agc))
      setting.agc = S_AUTO_ON;
    if (S_IS_AUTO(setting.lna))
      setting.lna = S_AUTO_OFF;
#endif
  } else {
    r = 10 * roundf((r*1.2)/10.0);
    set_reflevel(r);
    set_scale(10);
#ifdef __SI4432__
    if (S_IS_AUTO(setting.agc))
      setting.agc = S_AUTO_ON;
    if (S_IS_AUTO(setting.lna))
      setting.lna = S_AUTO_OFF;
#endif
  }
  plot_into_index(measured);
  redraw_request|=REDRAW_AREA;
  //dirty = true;             // No HW update required, only status panel refresh
}

const float unit_scale_value[]={  1, 0.001,   0.000001, 0.000000001, 0.000000000001};
const char  unit_scale_text[]= {' ',   'm',     '\035',         'n',            'p'};

void user_set_reflevel(float level)
{
  set_auto_reflevel(false);
  if (UNIT_IS_LINEAR(setting.unit) && level < setting.scale*NGRIDY) {    // Avoid below zero level
    set_scale(level/NGRIDY);
    set_reflevel(setting.scale*NGRIDY);
  } else
    set_reflevel(level);
  redraw_request|=REDRAW_AREA;
}

void set_reflevel(float level)
{

  if (UNIT_IS_LINEAR(setting.unit)) {
    if (level < REFLEVEL_MIN)
      level = REFLEVEL_MIN;
    if (level > REFLEVEL_MAX)
      level = REFLEVEL_MAX;
  }

  setting.unit_scale_index = 0;
  setting.unit_scale = 1.0;
  while (UNIT_IS_LINEAR(setting.unit) && setting.unit_scale_index < ARRAY_COUNT(unit_scale_value) - 1) {
    if (level > unit_scale_value[setting.unit_scale_index])
      break;
    setting.unit_scale_index++;
  }
  setting.unit_scale = unit_scale_value[setting.unit_scale_index];
  setting.reflevel = level;
  set_trace_refpos(level);
//  dirty = true;
}

void round_reflevel_to_scale(void) {
  int multi = floorf((setting.reflevel + setting.scale/2)/setting.scale);
  if (UNIT_IS_LINEAR(setting.unit)) {
    if (multi < NGRIDY) {
      setting.reflevel = setting.scale*10;  // Never negative bottom
    }
  } else {

  }
  setting.reflevel = multi*setting.scale;
  set_trace_refpos(setting.reflevel);
}

void user_set_scale(float s)
{
  if (UNIT_IS_LINEAR(setting.unit))
    set_auto_reflevel(false);
  set_scale(s);
  if (UNIT_IS_LINEAR(setting.unit) && setting.reflevel < setting.scale*NGRIDY)
    set_reflevel(setting.scale*NGRIDY);
  redraw_request|=REDRAW_AREA;
}

void set_scale(float t) {
  if (UNIT_IS_LINEAR(setting.unit)) {
    if (t < REFLEVEL_MIN/10.0)
      t = REFLEVEL_MIN/10.0;
    if (t > REFLEVEL_MAX/10.0)
      t = REFLEVEL_MAX/10.0;
  } else {
    if (t > 20.0)
      t = 20.0;
    else if (t < 0.1)
      t = 0.1;
  }

  float m = 1;
//        t = t * 1.2;
  while (t > 10) { m *= 10; t/=10; }
  while (t < 1.0)  { m /= 10; t*=10; }
  if (UNIT_IS_LINEAR(setting.unit)) {
    t = ((int)(10*t+0.4999))/10.0;
  } else {
    if (t>5.0001)
      t = 10.0;
    else if (t>2.0001)
      t = 5.0;
    else if (t > 1.0001)
      t = 2.0;
    else
      t = 1.0;
  }
  t = t*m;
  setting.scale = t;
  set_trace_scale(t);
  round_reflevel_to_scale();
}

extern char low_level_help_text[12];

void set_external_gain(float external_gain)
{
  setting.external_gain = external_gain;
  int min,max;
  min = level_min();
  max = min + level_range();
  plot_printf(low_level_help_text, sizeof low_level_help_text, "%+d..%+d", min - (int)external_gain, max - (int)external_gain);
  redraw_request|=REDRAW_AREA;
  dirty = true;             // No HW update required, only status panel refresh but need to ensure the cached value is updated in the calculation of the RSSI
}

void set_trigger_level(float trigger_level)
{
  setting.trigger_level = trigger_level;
#ifdef  __TRIGGER_TRACE__
  setting.trigger_trace = 255;
#endif
  redraw_request |= REDRAW_TRIGGER | REDRAW_CAL_STATUS;
  //dirty = true;             // No HW update required, only status panel refresh
}

void set_trigger(int trigger)
{
  if (trigger == T_PRE || trigger == T_POST || trigger == T_MID) {
    setting.trigger_mode = trigger;
  } else if (trigger == T_UP || trigger == T_DOWN){
    setting.trigger_direction = trigger;
  } else if (trigger == T_DONE) {
    pause_sweep();                    // Trigger once so pause after this sweep has completed!!!!!!!
    setting.trigger = trigger;
  } else {
    sweep_mode = SWEEP_ENABLE;
    if (trigger != T_AUTO)
      setting.draw_line = false;
    setting.trigger = trigger;
  }
  redraw_request|=REDRAW_TRIGGER | REDRAW_CAL_STATUS;
  //dirty = true;             // No HW update required, only status panel refresh
}


//int GetRefpos(void) {
//  return (NGRIDY - get_trace_refpos(2)) * get_trace_scale(2);
//}

//int GetScale(void) {
//  return get_trace_refpos(2);
//}
void set_mode(int m)
{
  dirty = true;
  if (setting.mode == m)
    return;
  reset_settings(m);
//  dirty = true;
}

void set_fast_speedup(int s)
{
  setting.fast_speedup = s;
  dirty = true;
}

void set_faster_speedup(int s)
{
  setting.faster_speedup = s;
  dirty = true;
}

//
// Table for auto set sweep step/offset delays from RBW
//
#ifdef __SI4432__
static const struct {
  uint16_t rbw_x10;
  uint16_t step_delay;
  uint32_t offset_delay;
} step_delay_table[]={
#if 1
//  RBWx10 step_delay  offset_delay
  {  1910,       300,         100},
  {  1420,       350,         100},
  {   750,       450,         100},
  {   560,       650,         100},
  {   370,       700,         200},
  {   180,      1100,         300},
  {    90,      1700,         400},
  {    50,      3300,         800},
  {     0,      6400,        1600},
#else
  {  1910,       280,         100},
  {  1420,       350,         100},
  {   750,       450,         100},
  {   560,       650,         100},
  {   370,       700,         100},
  {   180,      1100,         200},
  {    90,      1700,         400},
  {    50,      3300,         400},
  {     0,      6400,        1600},
#endif
};
#endif

#ifdef __SI4463__
static const struct {
  uint16_t rbw_x10;
  uint16_t step_delay;
  uint16_t offset_delay;
  uint16_t spur_div_1000;
  int16_t   noise_level;
  float log_aver_correction;
} step_delay_table[]={
//  RBWx10 step_delay  offset_delay spur_gate (value divided by 1000)
  {  8500,       150,          150,      400,   -90,    0.7},
  {  6000,       150,          150,      300,   -95,    0.8},
  {  3000,       150,          150,      200,   -95,    1.3},
  {  1000,       300,          300,      100,   -105,   0.3},
  {   300,       400,          400,      100,   -110,   0.7},
  {   100,       900,          900,      100,   -115,   0.5},
  {    30,      1600,         1600,      100,   -120,   0.7},
  {    10,      4000,         4000,      100,   -122,   1.1},
  {     3,     18700,        18700,      100,   -125,   1.0}
};
#endif

void calculate_step_delay(void)
{
  if (setting.step_delay_mode == SD_MANUAL || setting.step_delay != 0) {        // The latter part required for selftest 3
    SI4432_step_delay = setting.step_delay;
    if (setting.offset_delay != 0)      // Override if set
      SI4432_offset_delay = setting.offset_delay;
  } else {
    SI4432_offset_delay = 0;
    if (setting.frequency_step == 0) {            // zero span mode, not dependent on selected RBW
      SI4432_step_delay = 0;
    } else {
      // Search index in table depend from RBW
      uint16_t i=0;


      for (i=0;i<ARRAY_COUNT(step_delay_table)-1;i++)
        if (actual_rbw_x10 >= step_delay_table[i].rbw_x10)
          break;
#ifdef __SI4432__
      SI4432_step_delay   = step_delay_table[i].step_delay;
      SI4432_offset_delay = step_delay_table[i].offset_delay;
      spur_gate           = actual_rbw_x10 * (100 / 2);
#endif
#ifdef __SI4463__
      SI4432_step_delay   = step_delay_table[i].step_delay;
      SI4432_offset_delay = step_delay_table[i].offset_delay;
      spur_gate           = actual_rbw_x10 * (actual_rbw_x10 > 5000 ? (100/2) : 100);
      if (spur_gate < 30000)
        spur_gate = 30000;
//      spur_gate           = step_delay_table[i].spur_div_1000 * 1000;
      noise_level         = step_delay_table[i].noise_level - PURE_TO_float(get_signal_path_loss());
      log_averaging_correction = step_delay_table[i].log_aver_correction;
#endif
      if (setting.step_delay_mode == SD_PRECISE)    // In precise mode wait twice as long for RSSI to stabilize
        SI4432_step_delay += (SI4432_step_delay>>2) ;
      if (setting.fast_speedup >0)
        SI4432_offset_delay = SI4432_step_delay / setting.fast_speedup;
      if (setting.faster_speedup >0)
        SI4432_step_delay = SI4432_step_delay / setting.faster_speedup;
    }
    if (setting.offset_delay != 0)      // Override if set
      SI4432_offset_delay = setting.offset_delay;
  }
}

//------------------------------------------
#if 0
#define CORRECTION_POINTS  10

static const freq_t correction_frequency[CORRECTION_POINTS] =
{ 100000, 200000, 400000, 1000000, 2000000, 50000000, 100000000, 200000000, 300000000, 350000000 };

static const float correction_value[CORRECTION_POINTS] =
{ +4.0, +2.0, +1.5, +0.5, 0.0, 0.0, +1.0, +1.0, +2.5, +5.0 };
#endif

/*
 * To avoid float calculations the correction values are maximum +/-16 and accuracy of 0.5 so they fit easily in 8 bits
 * The frequency steps between correction factors is assumed to be maximum 500MHz or 0x2000000 and minimum 100kHz or > 0x10000
 * The divider 1/m is pre-calculated into delta_div as 2^scale_factor * correction_step/frequency_step
 */
#define FREQ_SCALE_FACTOR 10
#define SCALE_FACTOR 5     // min scaled correction = 2^15, max scaled correction = 256 * 2^15
                            // min scaled f = 6, max scaled f =  1024

static int32_t scaled_correction_multi[CORRECTION_SIZE][CORRECTION_POINTS];
static int32_t scaled_correction_value[CORRECTION_SIZE][CORRECTION_POINTS];

#if 0                       // Not implemented
static const int8_t scaled_atten_correction[16][16] =
{
 {0, -1, -2, -2, -3, -4, -3, -1, 0, 3, 7, 14, 21, 30, 42, 54 },                     // 2.6G dB*8, 16 levels
 {0, -2, -4, -6, -7, -9, -8, -8, -11, -9, -9, -8, -7, -4, 2, 8 },                   // 3.2G
 {0, 0, 0, -1, -8, -10, -10, -12, -22, -24, -28, -30, -37, -34, -24, -13, },        // 3.8G
 {0, 0, 0, -1, -8, -10, -10, -12, -22, -24, -28, -30, -37, -34, -24, -13, },        // 4.3G
 {0, 0, 0, 1, -4, -2, 0, 0, -3, 0, 1, 6, 5, 10, 16, 22, },                          // 4.8G
 {0, 0, 1, 2, -9, -7, -6, -5, -18, -18, -17, -17, -23, -24, -25, -27, },            // 5.4G
 {0, -1, -3, -3, -21, -20, -20, -20, -31, -29, -24, -18, -4, 4, 19, 30, },          // 5.9G
};
#endif

static void calculate_correction(void)
{
  for (int c = 0; c < CORRECTION_SIZE; c++) {
  scaled_correction_value[c][0] = config.correction_value[c][0]  * (1 << (SCALE_FACTOR));
  for (int i = 1; i < CORRECTION_POINTS; i++) {
    scaled_correction_value[c][i] = config.correction_value[c][i]  * (1 << (SCALE_FACTOR));
    int32_t m = scaled_correction_value[c][i] - scaled_correction_value[c][i-1];
//    int32_t d = (setting.correction_frequency[i] - setting.correction_frequency[i-1]) >> SCALE_FACTOR;
    scaled_correction_multi[c][i] = m; // (int32_t) ( m / d );
  }
  }
}
#pragma GCC push_options
#pragma GCC optimize ("Og")             // "Os" causes problem

pureRSSI_t get_frequency_correction(freq_t f)      // Frequency dependent RSSI correction to subtract for imperfect LPF
{
  pureRSSI_t cv = 0;
  int c=CORRECTION_LOW_IN;
#ifdef TINYSA4
  if (setting.mode == M_LOW) {

    cv = float_TO_PURE_RSSI(get_level_offset());
    //
    //  ----------------   duplication of code
    //
    actual_drive = setting.lo_drive;
    if (actual_drive & 0x04){     // Automatic mixer drive
      if (LO_harmonic)
        actual_drive = 3;
//      else if (f <  DRIVE0_MAX_FREQ)       // below 600MHz
//        actual_drive = 0;                     // Never use drive zero
      else if (f < DRIVE1_MAX_FREQ || hw_if) // below 1.2GHz of v4.6
        actual_drive = 1;
      else if (f < DRIVE2_MAX_FREQ)  // below 2.1GHz
        actual_drive = 2;
      else
        actual_drive = 3;
    }
    //
    //  ----------------- end duplication of code
    //

    switch (signal_path) {
    case PATH_LOW:
      c = CORRECTION_LOW_IN;
      break;
    case PATH_ULTRA:
      if (LO_harmonic) {
        c = CORRECTION_HARM;
      } else {
        c = CORRECTION_LOW_ULTRA;
  //    if (actual_drive >= 1)
  //      cv += float_TO_PURE_RSSI(config.drive1_level_offset);           // always zero
        if (actual_drive >= 2)
          cv += float_TO_PURE_RSSI(config.drive2_level_offset);
        if (actual_drive >= 3)
          cv += float_TO_PURE_RSSI(config.drive3_level_offset);
        if (f>MAX_ABOVE_IF_FREQ)
          cv += float_TO_PURE_RSSI(config.shift3_level_offset);
      }

      break;
#ifdef CORRECTION_DIRECT
    case PATH_DIRECT:
      c = CORRECTION_DIRECT;
      break;
#endif
    }
    if (setting.extra_lna)
      c += 1;
//    if (LO_shifting && signal_path != PATH_DIRECT)
//      cv += float_TO_PURE_RSSI(actual_rbw_x10>USE_SHIFT2_RBW ? config.shift2_level_offset : config.shift1_level_offset);

  } else if (setting.mode == M_GENLOW){
    switch (signal_path) {
    case PATH_OFF:
      return 0;
    case PATH_LOW:
      c = CORRECTION_LOW_OUT;
      break;
    case PATH_ULTRA:
      c = CORRECTION_LOW_OUT_MIXER;
      break;
    case PATH_DIRECT:
      c = CORRECTION_LOW_OUT_DIRECT;
      cv += float_TO_PURE_RSSI(config.direct_level_output_offset);
      break;
    case PATH_LEAKAGE:
      c = CORRECTION_LOW_OUT_ADF;
      cv += float_TO_PURE_RSSI(config.adf_level_offset);
      break;
    }
  }
#else
  if (setting.mode == M_HIGH)
    c = CORRECTION_HIGH_IN;
  else if (setting.mode == M_GENLOW)
    c = CORRECTION_LOW_OUT;
  else if (setting.mode == M_GENHIGH) {
    cv = 0;
    goto done;
  }
#endif


#ifdef TINYSA4
#if 0                       // Not implemented
  int cf = (((f >> 28)+1)>>1) - 5;   // Correction starts at 2,684,354,560Hz round to closest correction frequency
  int ca = setting.attenuate_x2 >> 2; // One data point m 2dB step
  if (cf >= 0 && cf < 16)
    cv -= scaled_atten_correction[cf][ca]<<2;  // Shift is +5(pure RSSI) - 3 (scaled correction) = 2
#endif
#endif


#if 0
  if (setting.extra_lna) {
    if (f > 2100000000U) {
      cv += float_TO_PURE_RSSI(+13);
    } else {
      cv += float_TO_PURE_RSSI( (float)f * 6.0 / 1000000000); // +6dBm at 1GHz
    }
  }

  if (f > ULTRA_MAX_FREQ) {
    cv += float_TO_PURE_RSSI(+4);       // 4dB loss in harmonic mode
  }
#endif
#ifdef TINYSA4
  if (setting.disable_correction)
    goto done;
#endif
  int i = 0;
  while (f > config.correction_frequency[c][i] && i < CORRECTION_POINTS)
    i++;
  if (i >= CORRECTION_POINTS) {
    cv += scaled_correction_value[c][CORRECTION_POINTS-1] >> (SCALE_FACTOR - 5);
    goto done;
  }
  if (i == 0) {
    cv += scaled_correction_value[c][0] >> (SCALE_FACTOR - 5);
    goto done;
  }
  f = f - config.correction_frequency[c][i-1];
#if 0
  freq_t m = (setting.correction_frequency[i] - setting.correction_frequency[i-1]) >> SCALE_FACTOR ;
  float multi = (setting.correction_value[i] - setting.correction_value[i-1]) * (1 << (SCALE_FACTOR -1)) / (float)m;
  float cv = setting.correction_value[i-1] + ((f >> SCALE_FACTOR) * multi) / (float)(1 << (SCALE_FACTOR -1)) ;
#else
#ifdef TINYSA4
  int64_t scaled_f = f >> FREQ_SCALE_FACTOR;
#else
  int32_t scaled_f = f >> FREQ_SCALE_FACTOR;
#endif
  int32_t scaled_f_divider = (config.correction_frequency[c][i] - config.correction_frequency[c][i-1]) >> FREQ_SCALE_FACTOR;
  if (scaled_f_divider!=0)
    cv += (scaled_correction_value[c][i-1] + ((scaled_f * scaled_correction_multi[c][i])/scaled_f_divider)) >> (SCALE_FACTOR - 5) ;
  else
    cv += scaled_correction_value[c][i-1] >> (SCALE_FACTOR - 5) ;
#endif
done:
  return(cv);
}
#pragma GCC pop_options



float peakLevel;
//float min_level;
freq_t peakFreq;
int peakIndex;
int peakTrace;
float temppeakLevel;
uint16_t temppeakIndex;
// volatile int t;

void setup_sa(void)
{
#ifdef __SI4432__
  SI4432_Init();
#endif
#ifdef TINYSA3
  for (unsigned int i = 0; i < sizeof(old_freq)/sizeof(unsigned long) ; i++) {
    old_freq[i] = 0;
    real_old_freq[i] = 0;
  }
#endif
#ifdef __SI4432__
  SI4432_Sel = SI4432_RX ;
  SI4432_Receive();

  SI4432_Sel = SI4432_LO ;
  SI4432_Transmit(0);
#endif
#ifdef __PE4302__
  PE4302_init();
  PE4302_Write_Byte(0);
#endif
#ifdef TINYSA4
  SI4463_init_rx();            // Must be before ADF4351_setup!!!!
  ADF4351_Setup();
#if 0
  enable_extra_lna(false);
  enable_ultra(false);
  enable_rx_output(false);
  enable_high(false);
  enable_direct(false);
#endif
  fill_spur_table();
#endif
  #if 0           // Measure fast scan time
  setting.sweep_time_us = 0;
  setting.additional_step_delay_us = 0;
  START_PROFILE             // measure 90 points to get overhead
  SI4432_Fill(0,200);
  int t1 = DELTA_TIME;
  RESTART_PROFILE           // measure 290 points to get real added time for 200 points
  SI4432_Fill(0,0);
  int t2 = DELTA_TIME;
  int t = (t2 - t1) * 100 * (sweep_points) / 200; // And calculate real time excluding overhead for all points
#endif
}

#define __WIDE_OFFSET__
#ifdef __WIDE_OFFSET__
#define OFFSET_LOWER_BOUND -80000
#else
#define OFFSET_LOWER_BOUND 0
#endif

#ifdef TINYSA4
static int fast_counter = 0;
#endif

void set_freq(int V, freq_t freq)    // translate the requested frequency into a setting of the SI4432
{
  if (old_freq[V] == freq)       // Do not change HW if not needed
    return;
#ifdef __SI4432__
  if (V <= 1) {
    SI4432_Sel = V;
    if (freq < 240000000 || freq > 960000000) {   // Impossible frequency, simply ignore, should never happen.
      real_old_freq[V] = freq + 1; // No idea why this is done........
      return;
    }
#if 1
    if (V == 1 && setting.step_delay_mode == SD_FAST) {        // If in extra fast scanning mode and NOT SI4432_RX !!!!!!
      int delta =  freq - real_old_freq[V];

      if (real_old_freq[V] >= 480000000)    // 480MHz, high band
        delta = delta >> 1;
      if (delta > OFFSET_LOWER_BOUND && delta < 79999) { // and requested frequency can be reached by using the offset registers
#if 0
        if (real_old_freq[V] >= 480000000)
          shell_printf("%d: Offs %q HW %d\r\n", SI4432_Sel, (freq_t)(real_old_freq[V]+delta*2),  real_old_freq[V]);
        else
          shell_printf("%d: Offs %q HW %d\r\n", SI4432_Sel, (freq_t)(real_old_freq[V]+delta*1),  real_old_freq[V]);
#endif
        delta = delta * 4 / 625; // = 156.25;             // Calculate and set the offset register i.s.o programming a new frequency
        SI4432_Write_2_Byte(SI4432_FREQ_OFFSET1, (uint8_t)(delta & 0xff), (uint8_t)((delta >> 8) & 0x03));
 //       SI4432_Write_Byte(SI4432_FREQ_OFFSET2, (uint8_t)((delta >> 8) & 0x03));
        SI4432_offset_changed = true;                 // Signal offset changed so RSSI retrieval is delayed for frequency settling
        old_freq[V] = freq;
      } else {
#ifdef __WIDE_OFFSET__
        freq_t target_f;                    // Impossible to use offset so set SI4432 to new frequency
        if (freq < real_old_freq[V]) {                          // sweeping down
          if (freq - 80000 >= 480000000) {
            target_f = freq - 160000;
          } else {
            target_f = freq - 80000;
          }
          SI4432_Set_Frequency(target_f);
          SI4432_Write_2_Byte(SI4432_FREQ_OFFSET1, 0xff, 0x01);           // set offset to most positive
 //         SI4432_Write_Byte(SI4432_FREQ_OFFSET2, 0x01);
          real_old_freq[V] = target_f;
        } else {                                                // sweeping up
          if (freq + 80000 >= 480000000) {
            target_f = freq + 160000;
          } else {
            target_f = freq + 80000;
          }
          if (target_f > 960000000)
            target_f = 960000000;
          SI4432_Set_Frequency(target_f);
          SI4432_Write_2_Byte(SI4432_FREQ_OFFSET1, 0, 0x02);           // set offset to most negative
//          SI4432_Write_Byte(SI4432_FREQ_OFFSET2, 0x02);
          real_old_freq[V] = target_f;
        }
#else
        SI4432_Set_Frequency(freq);           // Impossible to use offset so set SI4432 to new frequency
        SI4432_Write_2_Byte(SI4432_FREQ_OFFSET1, 0, 0);           // set offset to zero
//        SI4432_Write_Byte(SI4432_FREQ_OFFSET2, 0);
        real_old_freq[V] = freq;
#endif
      }
    } else {
#endif
      SI4432_Set_Frequency(freq);           // Not in fast mode
      real_old_freq[V] = freq;
    }
  } 
#endif
#ifdef TINYSA4
  if (V==ADF4351_LO){
#if 0
    if (setting.step_delay_mode == SD_FAST) {        // If in fast scanning mode and NOT SI4432_RX !!!!!!
      int delta = - (freq - real_old_freq[V]);           // delta grows with increasing freq
      if (setting.frequency_step < 100000 && 0 < delta && delta < 100000) {
        SI4463_start_rx(delta / setting.frequency_step); // with increasing delta, set smaller offset
        freq = 0;
      } else {
        SI4463_start_rx(0 / setting.frequency_step);   // Start at maximum positive offset
      }
    }
#endif
    if (freq) {
#if 0   // moved
      // ----------------------------- set mixer drive --------------------------------------------
      int target_drive = setting.lo_drive;
      if (target_drive & 0x04){     // Automatic mixer drive
        if (LO_harmonic)
          target_drive = 3;
        else if (freq-970000000 <  600000000ULL)       // below 600MHz
          target_drive = 0;
        else if (freq-970000000 < 1200000000ULL) // below 1.2GHz
          target_drive = 1;
//        else if (freq-970000000 < 2000000000ULL)  // below 2GHz
//          target_drive = 2;
        else
          target_drive = 2;
      }
      ADF4351_drive(target_drive);       // Max drive
#endif
      real_old_freq[V] = ADF4351_set_frequency(V-ADF4351_LO,freq);
    }
  } else if (V==ADF4351_LO2) {
    real_old_freq[V] = ADF4351_set_frequency(V-ADF4351_LO, freq);
  } else if (V==SI4463_RX) {
    if (setting.step_delay_mode == SD_FAST && fast_counter++ < 100 && real_old_freq[V] != 0) {        // If in extra fast scanning mode and NOT SI4432_RX !!!!!!
      long delta =  (long)freq - (long)real_old_freq[V];
#define OFFSET_STEP 14.30555                // 30MHz
//#define OFFSET_STEP 12.3981
#define OFFSET_RANGE 937500  // Hz
      real_offset = delta;
      if (real_old_freq[V] >= 480000000)    // 480MHz, high band
        delta = delta >> 1;
      delta = ((float)delta) / OFFSET_STEP;     // Calculate and set the offset register i.s.o programming a new frequency
      if (delta > - 0x7fff && delta < 0x7fff) { // and requested frequency can be reached by using the offset registers
        static  int old_delta = 0x20000;
        if (old_delta != delta) {
          si_set_offset(delta);               // Signal offset changed so RSSI retrieval is delayed for frequency settling
          old_delta = delta;
        }
        goto done;
      }
    }
    fast_counter = 0;                                    // Offset tuning not possible
    real_offset = 0;
    real_old_freq[V] = SI4463_set_freq(freq);           // Not in fast mode
  }
done:
#endif
  old_freq[V] = freq;
}

#ifdef __SI4432__
void set_switch_transmit(void) {
  SI4432_Write_2_Byte(SI4432_GPIO0_CONF, 0x1f, 0x1d);// Set switch to transmit
  // SI4432_Write_Byte(SI4432_GPIO1_CONF, 0x1d);
}

void set_switch_receive(void) {
  SI4432_Write_2_Byte(SI4432_GPIO0_CONF, 0x1d, 0x1f);// Set switch to receive
//  SI4432_Write_Byte(SI4432_GPIO1_CONF, 0x1f);
}

void set_switch_off(void) {
  SI4432_Write_2_Byte(SI4432_GPIO0_CONF, 0x1d, 0x1f);// Set both switch off
//  SI4432_Write_Byte(SI4432_GPIO1_CONF, 0x1f);
}

#endif

void set_switches(int m)
{
#ifdef __SI4432__
  SI4432_Init();
  old_freq[0] = 0;
  old_freq[1] = 0;
  real_old_freq[0] = 0;
  real_old_freq[1] = 0;
  SI4432_Sel = SI4432_LO ;
  SI4432_Write_2_Byte(SI4432_FREQ_OFFSET1, 0, 0);  // Back to nominal offset
//  SI4432_Write_Byte(SI4432_FREQ_OFFSET2, 0);
#endif
  switch(m) {
case M_LOW:     // Mixed into 0
#ifdef __SI4432__
    SI4432_Sel = SI4432_RX ;
    SI4432_Receive();
    if (setting.atten_step) {   // use switch as attenuator
      set_switch_transmit();
    } else {
      set_switch_receive();
    }
    SI4432_Sel = SI4432_LO ;
    if (setting.tracking_output)
      set_switch_transmit();
    else
      set_switch_off();
//    SI4432_Receive(); For noise testing only
    SI4432_Transmit(setting.lo_drive);
    // set_calibration_freq(setting.refer);
#endif
#ifdef TINYSA4
#if 0
    SI4463_init_rx();            // Must be before ADF4351_setup!!!!
    if (setting.atten_step) {// use switch as attenuator
      enable_rx_output(true);
    } else {
       enable_rx_output(false);
    }
    enable_direct(false);
    enable_ADF_output(true, setting.tracking_output);
    enable_high(false);
    enable_extra_lna(setting.extra_lna);
    enable_ultra(false);
#endif
#endif
    set_AGC_LNA();
    break;
case M_HIGH:    // Direct into 1
mute:
#ifdef __SI4432__
    // set_calibration_freq(-1); // Stop reference output
    SI4432_Sel = SI4432_RX ; // both as receiver to avoid spurs
    set_switch_receive();
    SI4432_Receive();

    SI4432_Sel = SI4432_LO ;
    SI4432_Receive();
    if (setting.atten_step) {// use switch as attenuator
       set_switch_transmit();
     } else {
       set_switch_receive();
     }
#endif
#ifdef TINYSA4
#if 0
    if (SI4463_is_in_tx_mode())
      SI4463_init_rx();
    enable_ADF_output(false, setting.tracking_output);
    if (setting.atten_step) {// use switch as attenuator
      enable_rx_output(true);
    } else {
       enable_rx_output(false);
    }
    enable_high(true);
    enable_direct(false);
    enable_extra_lna(false);
    enable_ultra(false);
#endif
#endif
    set_AGC_LNA();
    break;
case M_GENLOW:  // Mixed output from 0
    if (setting.mute)
      goto mute;
#ifdef __SI4432__
    SI4432_Sel = SI4432_RX ;
    if (setting.atten_step) { // use switch as attenuator
      set_switch_off();
    } else {
      set_switch_transmit();
    }
    SI4432_Transmit(setting.rx_drive);

    SI4432_Sel = SI4432_LO ;
    if (setting.modulation == MO_EXTERNAL) {
      set_switch_transmit();  // High input for external LO scuh as tracking output of other tinySA
      SI4432_Receive();
    } else {
      set_switch_off();
      SI4432_Transmit(12);                 // Fix LO drive a 10dBm
    }
#endif
    break;
case M_GENHIGH: // Direct output from 1
    if (setting.mute)
      goto mute;
#ifdef TINYSA4
    enable_high(true);              // Must be first to protect SAW filters
    enable_extra_lna(false);
    enable_ultra(false);
    SI4463_init_rx();
    enable_rx_output(true);       // to protect the SI
    enable_ADF_output(true, true);
//    ADF4351_aux_drive(setting.lo_drive);
    enable_extra_lna(false);
    enable_ultra(true);                   // Open low output
#else
    SI4432_Sel = SI4432_RX ;
    SI4432_Receive();
    set_switch_receive();

    SI4432_Sel = SI4432_LO ;
    if (setting.lo_drive < 8) {
      set_switch_off(); // use switch as attenuator
    } else {
      set_switch_transmit();
    }
    SI4432_Transmit(setting.lo_drive);
  #endif
    break;
  }
}

void update_rbw(void)           // calculate the actual_rbw and the vbwSteps (# steps in between needed if frequency step is largen than maximum rbw)
{
  vbwSteps = 1;                 // starting number for all modes
  if (!MODE_INPUT(setting.mode)) {
    actual_rbw_x10 = 1;         // To force substepping of the SI4463
#ifdef TINYSA4
    goto done;
#else
    return;
#endif
  }
  frequency_step_x10 = 3000;  // default value for zero span
  if (setting.frequency_step > 0) {
    frequency_step_x10 = (setting.frequency_step)/100;
  }

  freq_t temp_actual_rbw_x10 = setting.rbw_x10;
  if (temp_actual_rbw_x10 == 0) {        // if auto rbw

    if (setting.step_delay_mode==SD_FAST) {    // if in fast scanning
      temp_actual_rbw_x10 = frequency_step_x10;
//    } else if (setting.step_delay_mode==SD_PRECISE) {
//      temp_actual_rbw_x10 = 4*frequency_step_x10;
    } else {
      temp_actual_rbw_x10 = 2*frequency_step_x10;
    }
  }
#ifdef __SI4432__
  if (temp_actual_rbw_x10 < 26)
    temp_actual_rbw_x10 = 26;
  if (temp_actual_rbw_x10 > 6000)
    temp_actual_rbw_x10 = 6000;
#endif
#ifdef __SI4463__
  if (temp_actual_rbw_x10 < 1)
    temp_actual_rbw_x10 = 1;
  if (temp_actual_rbw_x10 > 8500)
    temp_actual_rbw_x10 = 8500;
#endif
  actual_rbw_x10 = temp_actual_rbw_x10;         // Now it fits in 16 bit

#ifdef __SI4432__
  if (S_STATE(setting.spur_removal) && actual_rbw_x10 > 3000)
    actual_rbw_x10 = 2500;           // if spur suppression reduce max rbw to fit within BPF
  SI4432_Sel =  MODE_SELECT(setting.mode);
#endif
#ifdef __SI4463__
  // Not needed
#endif

  actual_rbw_x10 = set_rbw(actual_rbw_x10);  // see what rbw the be can realized


  if (setting.frequency_step > 0) {
    freq_t target_frequency_step_x10;

    if (setting.step_delay_mode==SD_FAST || setting.step_delay_mode==SD_NOISE_SOURCE) {
      target_frequency_step_x10 = frequency_step_x10;
    } else if (setting.step_delay_mode==SD_PRECISE) {
      target_frequency_step_x10 = 4*frequency_step_x10;
    } else {
      target_frequency_step_x10 = 2*frequency_step_x10;
    }

    if (target_frequency_step_x10 > actual_rbw_x10 && !(setting.step_delay_mode==SD_NOISE_SOURCE)) { // RBW too small
      vbwSteps = (target_frequency_step_x10 + actual_rbw_x10 - 1) / actual_rbw_x10; //((int)(2 * (frequency_step_x10 + (actual_rbw_x10/8)) / actual_rbw_x10)); // calculate # steps in between each frequency step due to rbw being less than frequency step
      if (vbwSteps<1)
        vbwSteps = 1;
    }
  }

  #ifdef TINYSA4
done:
  fill_spur_table();    // IF frequency depends on selected RBW
#endif
}

//#ifdef TINYSA4
//#define frequency_seatch_gate 60          // 120% of the RBW
//#else
//#define frequency_seatch_gate 100          // 200% of the RBW
//#endif

int binary_search_frequency(freq_t f)      // Search which index in the frequency tabled matches with frequency  f using actual_rbw
{
  int L = 0;
  int frequency_seatch_gate = (getFrequency(1) - getFrequency(0)) >> 1;
  if (f < getFrequency(0))
    return -1;
  if (f > getFrequency(sweep_points-1))
    return -1;
//  int R =  (sizeof frequencies)/sizeof(int) - 1;
  int R = sweep_points - 1;
  freq_t fmin =  f - frequency_seatch_gate; // actual_rbw_x10 * frequency_seatch_gate;
  freq_t fplus = f + frequency_seatch_gate; // actual_rbw_x10 * frequency_seatch_gate;
  while (L <= R) {
    int m = (L + R) / 2;
    freq_t f = getFrequency(m);
    if (f < fmin)
      L = m + 1;
    else if (f > fplus)
      R = m - 1;
    else
       return m; // index is m
  }
  return -1;
}

int index_of_frequency(freq_t f)      // Search which index in the frequency tabled matches with frequency  f using actual_rbw
{
#ifdef __BANDS__
  if (setting.multi_band && !setting.multi_trace) {
    for (int i=0;i<BANDS_MAX; i++) {
      band_t *b = &setting.bands[i];
      if (b->enabled && b->start <= f && f <b->end) {
        freq_t f_step = (b->end - b->start)/(b->stop_index - b->start_index);
        int i = ((f - b->start ) + (f_step >> 1)) / f_step + b->start_index;
        return i;
      }
    }
    return 0;
  }
#endif
  freq_t f_step = getFrequency(1) - getFrequency(0);
  if (f_step == 0)
    return 0;
  if (f < getFrequency(0))
    return -1;
  if (f > getFrequency(sweep_points-1))
    return -1;
  int i = ((f - getFrequency(0) ) + (f_step >> 1)) / f_step;
  return i;
#if 0
  //  int R =  (sizeof frequencies)/sizeof(int) - 1;
  int L = 0;
  int R = sweep_points - 1;
  freq_t fmin =  f - frequency_seatch_gate; // actual_rbw_x10 * frequency_seatch_gate;
  freq_t fplus = f + frequency_seatch_gate; // actual_rbw_x10 * frequency_seatch_gate;
  while (L <= R) {
    int m = (L + R) / 2;
    freq_t f = getFrequency(m);
    if (f < fmin)
      L = m + 1;
    else if (f > fplus)
      R = m - 1;
    else
       return m; // index is m
  }
#endif
}



void interpolate_maximum(int m)
{
  float *ref_marker_levels = measured[markers[m].trace];
  const int idx        = markers[m].index;
  markers[m].frequency = getFrequency(idx);
  if (idx > 0 && idx < sweep_points-1)
  {
    const int32_t delta_Hz = (int64_t)getFrequency(idx + 0) - getFrequency(idx + 1);
#ifdef TINYSA4
#define INTER_TYPE  double
#else
#define INTER_TYPE  float
#endif
    const INTER_TYPE y1         = ref_marker_levels[idx - 1];
    const INTER_TYPE y2         = ref_marker_levels[idx + 0];
    const INTER_TYPE y3         = ref_marker_levels[idx + 1];
    const INTER_TYPE d          = (INTER_TYPE)abs(delta_Hz) * 0.5 * (y1 - y3) / ((y1 - (2 * y2) + y3) + 1e-12);
    //const float bin      = (float)idx + d;
    markers[m].frequency   += d;
  }
}

#define MAX_MAX MARKER_COUNT
int
search_maximum(int m, freq_t center, int span)
{
  float *ref_marker_levels = measured[markers[m].trace];
#ifdef TINYSA4
  int center_index = index_of_frequency(center);
#else
  int center_index = binary_search_frequency(center);
#endif
  if (center_index < 0)
	return false;
  int from = center_index - span/2;
  int found = false;
  int to = center_index + span/2;
  int cur_max = 0;          // Always at least one maximum
  int max_index[MAX_MAX];
  if (from<0)
    from = 0;
  if (to > setting._sweep_points-1)
    to = setting._sweep_points-1;
  temppeakIndex = 0;
  temppeakLevel = ref_marker_levels[from];
  max_index[cur_max] = from;
  int downslope = true;

  for (int i = from; i <= to; i++) {
    if (downslope) {
      if (temppeakLevel > ref_marker_levels[i]) {    // Follow down
        temppeakIndex = i;                  // Latest minimum
        temppeakLevel = ref_marker_levels[i];
      } else if (temppeakLevel + setting.noise < ref_marker_levels[i]) {    // Local minimum found
        temppeakIndex = i;                          // This is now the latest maximum
        temppeakLevel = ref_marker_levels[i];
        downslope = false;
      }
    } else {
      if (temppeakLevel < ref_marker_levels[i]) {    // Follow up
        temppeakIndex = i;
        temppeakLevel = ref_marker_levels[i];
      } else if (temppeakLevel - setting.noise > ref_marker_levels[i]) {    // Local max found

        found = true;
        int j = 0;                                            // Insertion index
        while (j<cur_max && ref_marker_levels[max_index[j]] >= temppeakLevel)   // Find where to insert
          j++;
        if (j < MAX_MAX) {                                    // Larger then one of the previous found
          int k = MAX_MAX-1;
          while (k > j) {                                      // Shift to make room for max
            max_index[k] = max_index[k-1];
            //              maxlevel_index[k] = maxlevel_index[k-1];        // Only for debugging
            k--;
          }
          max_index[j] = temppeakIndex;
          //            maxlevel_index[j] = ref_marker_levels[temppeakIndex];      // Only for debugging
          if (cur_max < MAX_MAX) {
            cur_max++;
          }
          //STOP_PROFILE
        }
        temppeakIndex = i;            // Latest minimum
        temppeakLevel = ref_marker_levels[i];

        downslope = true;
      }
    }
  }
  if (false && !found) {
    temppeakIndex = from;
    temppeakLevel = ref_marker_levels[from];
    for (int i = from+1; i <= to; i++) {
      if (temppeakLevel<ref_marker_levels[i])
        temppeakIndex = i;
    }
    found = true;
  }
  markers[m].index = max_index[0];
  interpolate_maximum(m);
//  markers[m].frequency = frequencies[markers[m].index];
  return found;
}

//static int spur_old_stepdelay = 0;
#ifdef TINYSA3
static const unsigned int spur_IF =            DEFAULT_IF;       // The IF frequency for which the spur table is value
static const unsigned int spur_alternate_IF =  DEFAULT_SPUR_IF;       // if the frequency is found in the spur table use this IF frequency
static  const freq_t spur_table[] =                                 // Frequencies to avoid
{
// 580000,            // 433.8 MHz table
// 880000,    //?
 960000,
// 1487000,   //?
 1600000,
// 1837000,           // Real signal
// 2755000,           // Real signal
// 2760000,
 2960000,
 4933000,
 4960000,
 6960000,
// 6980000,
 8267000,
 8960000,
// 10000000,
 10960000,
 11600000,
 12960000,
 14933000,
 14960000,
 16960000,
 18960000,
 21600000,

// 22960000,
 24960000,
 28960000,
// 29800000,
 31600000,
 34960000,
 33930000,
// 38105000,
 40960000,
 41600000,
 49650000,
 272400000,
 287950000,
// 288029520,
 332494215,
};
const int spur_table_size = (sizeof spur_table)/sizeof(freq_t);
#endif
#ifdef TINYSA4
#define static_spur_IF  DEFAULT_IF    // The IF frequency for which the spur table is value
#define STATIC_SPUR_TABLE_SIZE 56
static  const freq_t static_spur_table[STATIC_SPUR_TABLE_SIZE] =     // Valid for IF=977.4MHz
{
 5233000,
 6300000,
 16483000,
 16783000,
 21300000,
 26134000,
 36300000,
 41134000,
 51300000,
 66000000,
 66300000,
 70800000,
 72000000,
 78000000,
 85200000,
 101134000,
 113134000,
 114000000,
 115200000,
 243881127,
 471300000,
 487750054,
 487762254,
 501300000,
 508800000,
 650974672,
 688800000,
 699667000,
 702865000,
 703094000,
 703465000,
 706616000,
 707216000,
 708667000,
 710366000,
 710966000,
 711667000,
 711667000,
 714115000,
 714668000,
 718465000,
 718800000,
 721616000,
 722216000,
 726300000,
 729715000,
 732865000,
 738667000,
 740366000,
 740966000,
 741667000,
 747865000,
 756667000,
 759116000,
 793465000,
 797216000,
};

#define CALCULATED_SPUR_TABLE_SIZE = sizeof(static_spur_table)/8

#define static_spur_IF_plus  DEFAULT_IF_PLUS      // The IF frequency for which the spur table is value
static  const freq_t static_spur_table_plus[] =     // Valid for IF=977.4MHz
{
 266906127,
 533812254,
 712374836
};

#define STATIC_SPUR_TABLE_SIZE_PLUS sizeof(static_spur_table_plus)/8

#define MAX_DYNAMIC_SPUR_TABLE_SIZE 100
static  freq_t dynamic_spur_table[MAX_DYNAMIC_SPUR_TABLE_SIZE];       // Frequencies to be calculated
static int dynamic_spur_table_size = 0;
static int always_use_dynamic_table = false;

static  freq_t *spur_table = (freq_t *)static_spur_table;
int spur_table_size = STATIC_SPUR_TABLE_SIZE;
freq_t spur_IF = static_spur_IF;

#endif

int binary_search(freq_t f)
{
  int L = 0;
  int R =  spur_table_size - 1;
  freq_t fmin =  f - spur_gate;
  freq_t fplus = f + spur_gate;
#if 0
  freq_t fmin =  f - actual_rbw_x10 * (100 / 2);
  freq_t fplus = f + actual_rbw_x10 * (100 / 2);
#endif
  while (L <= R) {
    int m = (L + R) / 2;
    if (spur_table[m] < fmin)
      L = m + 1;
    else if (spur_table[m] > fplus)
      R = m - 1;
    else
       return true; // index is m
  }
#ifdef TINYSA4  
#if 0
  if (!setting.auto_IF && setting.frequency_IF-2000000 < f && f < setting.frequency_IF -200000)
    return true;
  if(config.frequency_IF1+200000 > f && config.frequency_IF1 < f+200000)
    return true;
  if(4*config.frequency_IF1 > fmin && 4*config.frequency_IF1 < fplus)
    return true;
#endif
#endif
  return false;
}

#ifdef TINYSA4

#define ADDITIONAL_SPUR_TABLE_SIZE 8
static  const freq_t additional_spur_table[ADDITIONAL_SPUR_TABLE_SIZE] =
{
 132000000,
 153000000,
 174600000,
 197000000,
 219000000,
 242000000,
 266000000,
 289600000,
};

//binary_search_table(f, additional_spur_table, ADDITIONAL_SPUR_TABLE_SIZE, ADDITIONAL_SPUR_TABE_GATE)

#define ADDITIONAL_SPUR_TABE_GATE  1000000

int binary_search_table(freq_t f, const freq_t *table, int table_size, int gate)
{
  int L = 0;
  int R =  table_size - 1;
  freq_t fmin =  f - gate;
  freq_t fplus = f + gate;
  while (L <= R) {
    int m = (L + R) / 2;
    if (table[m] < fmin)
      L = m + 1;
    else if (table[m] > fplus)
      R = m - 1;
    else
       return true; // index is m
  }
  return false;
}
#endif


#ifdef TINYSA4
//static const uint8_t spur_div[] = {3, 3, 5, 2, 3, 4};        // 4/1 removed
//static const uint8_t spur_mul[] = {1, 1, 2, 1, 2, 3};
//#define IF_OFFSET   468750*4        //

#define RBW_FOR_STATIC_TABLE    1100

#define SPUR_FACTOR 937746
void fill_spur_table(void)
{
  freq_t corr_IF;

  if (always_use_dynamic_table) {
    spur_table = dynamic_spur_table;
    spur_table_size = dynamic_spur_table_size;
    return;
  }
  if (actual_rbw_x10 < RBW_FOR_STATIC_TABLE) {         // if less then 1100kHz use static table
    if (hw_if) {
      spur_table = (freq_t *)static_spur_table_plus;
      spur_table_size = STATIC_SPUR_TABLE_SIZE_PLUS;
      spur_IF = static_spur_IF_plus;
    } else {
      spur_table = (freq_t *)static_spur_table;
      spur_table_size = STATIC_SPUR_TABLE_SIZE;
      spur_IF = static_spur_IF;
    }
    return;
  }
  if (!setting.auto_IF)
    corr_IF = setting.frequency_IF;
  else {
    corr_IF = config.frequency_IF1;
  }
  dynamic_spur_table_size = 0;
//  dynamic_spur_table[dynamic_spur_table_size++] = 132000000;
//  dynamic_spur_table[dynamic_spur_table_size++] = 153000000;
//  dynamic_spur_table[dynamic_spur_table_size++] = 174600000;
//  dynamic_spur_table[dynamic_spur_table_size++] = 219000000;
  dynamic_spur_table[dynamic_spur_table_size++] = corr_IF/4 -SPUR_FACTOR/2;
//  dynamic_spur_table[dynamic_spur_table_size++] = 266000000;
  dynamic_spur_table[dynamic_spur_table_size++] = corr_IF/2 -SPUR_FACTOR;
  dynamic_spur_table[dynamic_spur_table_size++] = corr_IF*2/3 -SPUR_FACTOR*2/3;
  spur_table = dynamic_spur_table;
  spur_table_size = dynamic_spur_table_size;
#if 0
  return; // TODO remove spur table updating.
  for (i=0; i < sizeof(spur_div)/sizeof(uint8_t); i++)
  {

    if (!setting.auto_IF)
      corr_IF = setting.frequency_IF;
    else {
      corr_IF = config.frequency_IF1 - DEFAULT_SPUR_OFFSET/2;
      setting.frequency_IF = corr_IF;
    }
   if (i != 5)  // <------------------- Index of the 3/2 entry in the spur tables
     corr_IF -= IF_OFFSET;
   else
     corr_IF -= IF_OFFSET/2;

   freq_t target = (corr_IF * (uint64_t)spur_mul[i] ) / (uint64_t) spur_div[i];
//   volatile uint64_t actual_freq = ADF4351_set_frequency(0, target + config.frequency_IF1);
//   volatile uint64_t delta =  target + (uint64_t) config.frequency_IF1 - actual_freq ;
//   volatile uint64_t spur = target - delta;
//   spur_table[i] = spur;
   if (i==1)  // <---------------------------------index of a 3/1 entry
     spur_table[i] = target - IF_OFFSET / 12;
   else if (i == 2)  // <---------------------------------index of a 3/1 entry
     spur_table[i] = target + IF_OFFSET / 12;
   else
     spur_table[i] = target;
  }
  if (!setting.auto_IF)
    corr_IF = setting.frequency_IF;
  else {
    corr_IF = config.frequency_IF1 - DEFAULT_SPUR_OFFSET/2;
  }
  spur_table[i++] = corr_IF - IF_OFFSET*3/2;
  spur_table[i++] = corr_IF*2 - IF_OFFSET;
  spur_table_size = i;
#endif
}
#endif

enum {F_NOSPUR = 0, F_NEAR_SPUR = 1, F_AT_SPUR = 2};

int avoid_spur(freq_t f)                   // find if this frequency should be avoided
{
  if (in_selftest)
    return F_NOSPUR;
//  int window = ((int)actual_rbw ) * 1000*2;
//  if (window < 50000)
//    window = 50000;
#ifdef TINYSA4
  if (setting.mode != M_LOW /* || !setting.auto_IF */)
    return(F_NOSPUR);
#else
  if (setting.mode != M_LOW || !setting.auto_IF || actual_rbw_x10 > 3000)
    return(F_NOSPUR);
#endif
  int L = 0;
  int R =  spur_table_size - 1;
#ifdef TINYSA4
  freq_t fmin =  f - spur_gate; //*8;
  freq_t fplus = f + spur_gate; //*8;
#else
  freq_t fmin =  f - spur_gate;
  freq_t fplus = f + spur_gate;
#endif
#if 0
  freq_t fmin =  f - actual_rbw_x10 * (100 / 2);
  freq_t fplus = f + actual_rbw_x10 * (100 / 2);
#endif
  while (L <= R) {
    int m = (L + R) / 2;
    if (spur_table[m] < fmin)
      L = m + 1;
    else if (spur_table[m] > fplus)
      R = m - 1;
    else
    {
#if 0
#ifdef TINYSA4
      int w = ((unsigned int)m >= sizeof(spur_div)/sizeof(uint8_t) ? 3 : 1);
      fmin =  f - spur_gate*w;
      fplus = f + spur_gate*w;
      if (spur_table[m] < fmin || spur_table[m] > fplus)
        return F_NEAR_SPUR; // index is m
      else
#endif
#endif
        return F_AT_SPUR;
    }
  }
#ifdef TINYSA4
#if 1
  if (!setting.auto_IF && setting.frequency_IF-2000000 < f && f < setting.frequency_IF -200000)
    return true;
  if(config.frequency_IF1+200000 > f && config.frequency_IF1 < f+200000)
    return F_AT_SPUR;
#endif
  if(4*config.frequency_IF1 > fmin && 4*config.frequency_IF1 < fplus)
    return F_AT_SPUR;
#endif
  return F_NOSPUR;
}


static uint16_t max_index[MAX_MAX];
static uint16_t cur_max = 0;
float temppeakLevel = -150;
int16_t downslope = true;
int peak_finding_index;
float peak_finding_level;


void add_to_peak_finding(float *trace_data, int i) {
  // START_PROFILE
  if (i == 0 || getFrequency(i) < actual_rbw_x10 * 200) {   // Prepare peak finding
    cur_max = 0;          // Always at least one maximum
    temppeakIndex = 0;
    temppeakLevel = trace_data[0];
    max_index[0] = 0;
    downslope = true;
    peak_finding_index = 0;
    peak_finding_level = temppeakLevel;
  }
  if (cur_max == 0 && peak_finding_level <=  trace_data[i]) {
    peak_finding_index = i;
    peak_finding_level = trace_data[i];
  }
  if (downslope) {                               // If in down slope peak finding
    if (temppeakLevel > trace_data[i]) {           // Follow down
      temppeakIndex = i;                         // Latest minimum
      temppeakLevel = trace_data[i];
    } else if (temppeakLevel + setting.noise < trace_data[i] ) {    // Local minimum found
      temppeakIndex = i;                         // This is now the latest maximum
      temppeakLevel = trace_data[i];
      downslope = false;
    }
  } else {                                      // up slope peak finding
    if (temppeakLevel < trace_data[i]) {    // Follow up
      temppeakIndex = i;
      temppeakLevel = trace_data[i];
    } else if (trace_data[i] < temppeakLevel - setting.noise) {    // Local max found

      // maintain sorted peak table
      int j = 0;                                            // Insert max in sorted table
      while (j<cur_max && trace_data[max_index[j]] >= temppeakLevel)   // Find where to insert
        j++;
      if (j < MAX_MAX) {                                    // Larger then one of the previous found
        int k = MAX_MAX-1;
        while (k > j) {                                      // Shift to make room for max
          max_index[k] = max_index[k-1];
          //              maxlevel_index[k] = maxlevel_index[k-1];        // Only for debugging
          k--;
        }
        max_index[j] = temppeakIndex;
        //            maxlevel_index[j] = trace_data[temppeakIndex];      // Only for debugging
        if (cur_max < MAX_MAX) {
          cur_max++;
        }
        //STOP_PROFILE
      }
      // Insert done
      temppeakIndex = i;            // Latest minimum
      temppeakLevel = trace_data[i];

      downslope = true;
    }
  }        // end of peak finding
}



static int modulation_counter = 0;
static int cycle_counter = 0;

#define AM_MODULATION_STEPS 8
// AM modulation always
static const int am_modulation[AM_MODULATION_STEPS] =  { 5, 1, 0, 1, 5, 10, 12, 10 };         // AM modulation


#ifdef TINYSA4
static int modulation_steps = 8;
#define MAX_MODULATION_STEPS    128

// Max = 10000 / 28.8
static const int sine_wave[MAX_MODULATION_STEPS/4+1] = {0,    17,     34,     51,     68,     84,     101,    117,    133,    148,    164,    179,    193,    207,    220,    233,    246,    257,    268,    279,    289,    298,    306,    314,    321,    327,    332,    337,    341,    343,    346,    347,    347 };
//
//  Offset is 14.4Hz when below 600MHz and 28.8 when above.
//
#define HND  36     // High range near FM
#define HN2D  44     // High range near FM
#define HN3D  80     // High range near FM
#define HWD  1300    // High range wide FM 512
#define MODULATION_TABLES 4
static int fm_modulation[MAX_MODULATION_STEPS+1];
#if 0
=
{
 { 0*HND,(int)( 1.5*HND ), 2*HND, (int)(1.5*HND), 0*HND, (int)(-1.5*HND), (int)-2*HND, (int)(-1.5*HND)},                // High range, NFM
 { 0*HN2D,(int)( 1.5*HN2D ), 2*HN2D, (int)(1.5*HN2D), 0*HN2D, (int)(-1.5*HN2D), (int)-2*HN2D, (int)(-1.5*HN2D)},                // High range, NFM2
 { 0*HN3D,(int)( 1.5*HN3D ), 2*HN3D, (int)(1.5*HN3D), 0*HN3D, (int)(-1.5*HN3D), (int)-2*HN3D, (int)(-1.5*HN3D)},                // High range, NFM3
 { 0*HWD,(int)( 1.5*HWD ), 2*HWD, (int)(1.5*HWD), 0*HWD, (int)(-1.5*HWD), (int)-2*HWD, (int)(-1.5*HWD)},    // HIgh range, MO_WFM
};
#endif
#endif
#ifdef TINYSA3// Avoid sign changes in NFM
#define modulation_steps    8
#define MAX_MODULATION_STEPS    8

//
//  Offset is 156.25Hz when below 600MHz and 312.5 when above.
//
#define LND  10   // Total NFM deviation is LND * 4 * 156.25 = 5kHz when below 600MHz or 600MHz - 434MHz
#define HND  6
#define LWD  240 // Total WFM deviation is LWD * 4 * 156.25 = 75kHz when below 600MHz
#define HWD  120
#define MODULATION_TABLES 4
#define S1  1.5
static const int fm_modulation[MODULATION_TABLES][MAX_MODULATION_STEPS] =
{
 { 2*LND,(int)( (2+S1)*LND ), 4*LND, (int)((2+S1)*LND), 2*LND, (int)((2-S1)*LND), 0, (int)((2-S1)*LND)},                // Low range, MO_NFM
 { 0*LWD,(int)( S1*LWD ), 2*LWD, (int)(S1*LWD), 0*LWD, (int)(-S1*LWD), (int)-2*LWD, (int)(-S1*LWD)},    // Low range, MO_WFM
 { 2*HND,(int)( 3.5*HND ), 4*HND, (int)(3.5*HND), 2*HND, (int)(0.5*HND), 0, (int)(0.5*HND)},                // High range, MO_NFM
 { 0*HWD,(int)( 1.5*HWD ), 2*HWD, (int)(1.5*HWD), 0*HWD, (int)(-1.5*HWD), (int)-2*HWD, (int)(-1.5*HWD)},    // HIgh range, MO_WFM
};    // narrow FM modulation avoid sign changes
#endif

#undef S1
#ifdef TINYSA3
static const int fm_modulation_offset[4] =
{
   85000,
   80000,
  165000,
  160000
};
#endif


deviceRSSI_t age[POINTS_COUNT];     // Array used for 1: calculating the age of any max and 2: buffer for fast sweep RSSI values;

static pureRSSI_t correct_RSSI;
static pureRSSI_t correct_RSSI_freq;
systime_t start_of_sweep_timestamp;
static systime_t sweep_elapsed = 0;                             // Time since first start of sweeping, used only for auto attenuate
uint8_t signal_is_AM = false;
static uint8_t check_for_AM = false;
static int is_below = false;


static void calculate_static_correction(void)                   // Calculate the static part of the RSSI correction
{
  correct_RSSI =
#ifdef __SI4432__
      getSI4432_RSSI_correction()
#endif
#ifdef __SI4463__
      getSI4463_RSSI_correction()

#endif
      - get_signal_path_loss();

  pureRSSI_t temp = float_TO_PURE_RSSI(
#ifndef TINYSA4
          + get_level_offset()          // Moved to frequency dependent part
#endif
          + get_attenuation()
#ifdef TINYSA4
          - (S_STATE(setting.agc)? 0 : 33)
          - (S_STATE(setting.lna)? 12 : 0)
          + (setting.extra_lna ? (max2871?-18: -26.5) : 0)                // checked
          + (setting.mode == M_GENLOW ? (old_temp - 35.0) / 13.0 : (old_temp - 35.0) / 20.0)      // About 7.7dB per 10 degrees C in output mode, 1 dB per 20 degrees in input mode
#endif		  
          - setting.external_gain);
  correct_RSSI += temp;
}

int hsical = -1;
void clock_above_48MHz(void)
{
  if (hsical == -1)
    hsical = (RCC->CR & 0xff00) >> 8;
  if (hsical != -1) {
    RCC->CR &= RCC_CR_HSICAL;
    RCC->CR |= ( (hsical) << 8 );
    RCC->CR &= RCC_CR_HSITRIM | RCC_CR_HSION; /* CR Reset value.              */
    RCC->CR |= RCC_CR_HSITRIM_4 | RCC_CR_HSITRIM_0 | RCC_CR_HSITRIM_1;
  }
}

void clock_below_48MHz(void)
{
  if (hsical == -1)
    hsical = ( (RCC->CR & 0xff00) >> 8 );
  if (hsical != -1) {
    RCC->CR &= RCC_CR_HSICAL;
    RCC->CR |= ( (hsical) << 8 );
    RCC->CR &= RCC_CR_HSITRIM | RCC_CR_HSION; /* CR Reset value.              */
    RCC->CR |=  RCC_CR_HSITRIM_2 | RCC_CR_HSITRIM_3;
  }
}

void clock_at_48MHz(void)
{
  if (hsical == -1)
    hsical = ( (RCC->CR & 0xff00) >> 8 );
  if (hsical != -1) {
    RCC->CR &= RCC_CR_HSICAL;
    RCC->CR |= ( (hsical) << 8 );
    RCC->CR &= RCC_CR_HSITRIM | RCC_CR_HSION; /* CR Reset value.              */
    RCC->CR |= RCC_CR_HSITRIM_4;
  }
}

int force_signal_path = false;
int test_output_switch = false;
int test_output_drive = 0;
int test_path = 0;
int test_output_attenuate = 0;
#ifdef TINYSA4
bool level_error = false;
bool depth_error = false;

#endif


pureRSSI_t perform(bool break_on_operation, int i, freq_t f, int tracking)     // Measure the RSSI for one frequency, used from sweep and other measurement routines. Must do all HW setup
{
  int modulation_delay = 0;
#ifdef TINYSA3
  int modulation_table = 0;
#endif
//  int modulation_count_iter = 0;
#ifdef __NEW_SWITCHES__
//  int direct = ((setting.mode == M_LOW && config.direct  && f > DIRECT_START && f<DIRECT_STOP) || (setting.mode == M_GENLOW && f >= MINIMUM_DIRECT_FREQ && f < ultra_start) );
#else
//  const int direct = false;
#endif
#ifdef TINYSA4
  if (i == 0 && old_temp != Si446x_get_temp()) {
    old_temp = Si446x_get_temp();
    calculate_static_correction();                                          // In case temperature changed.
  }
#endif
  TRACE(10);
  if (i == 0 && dirty ) {                                                        // if first point in scan and dirty
#ifdef __ADF4351__
    clear_frequency_cache();
#endif
    calculate_correction();                                                 // pre-calculate correction factor dividers to avoid float division
    limits_update();
    set_switches(setting.mode);
#ifdef __PE4302__
    if (setting.mode == M_HIGH)
      PE4302_Write_Byte(40);  // Ensure defined input impedance of low port when using high input mode (power calibration)
    else
      PE4302_Write_Byte((int)(setting.attenuate_x2));
#endif
    set_calibration_freq(setting.refer);
    update_rbw();
    calculate_step_delay();
    old_a = -150;                                                   // clear cached level setting
    // Initialize HW
    scandirty = true;                                                       // This is the first pass with new settings
    for (int t=0;t<TRACES_MAX;t++)
      setting.scan_after_dirty[t] = 0;
    dirty = false;
    sweep_elapsed = chVTGetSystemTimeX();                              // for measuring accumulated time
    // Set for actual time pre calculated value (update after sweep)
    setting.actual_sweep_time_us = calc_min_sweep_time_us();
    // Change actual sweep time as user input if it greater minimum
    // And set start delays for 1 run
    // manually set delay, for better sync
    if (setting.sweep_time_us < 2.5 * ONE_MS_TIME){
      setting.additional_step_delay_us = 0;
      setting.sweep_time_us = 0; // set minimum
    }
    else if (setting.sweep_time_us <= 3 * ONE_MS_TIME){
      setting.additional_step_delay_us = 1;
      setting.sweep_time_us = 3000;
    }
    else if (setting.sweep_time_us > setting.actual_sweep_time_us){
      setting.additional_step_delay_us = (setting.sweep_time_us - setting.actual_sweep_time_us)/(sweep_points);
      setting.actual_sweep_time_us = setting.sweep_time_us;
    }
    else{ // not add additional correction, apply recommend time
      setting.additional_step_delay_us = 0;
      //      setting.sweep_time_us = setting.actual_sweep_time_us;
    }
    if (MODE_INPUT(setting.mode)) {
      calculate_static_correction();
#ifdef __MCU_CLOCK_SHIFT__
      if (!in_selftest) clock_above_48MHz();
      is_below = false;
#endif
      correct_RSSI_freq = get_frequency_correction(f);  // for i == 0 and freq_step == 0;
#ifdef TINYSA4
//      correct_RSSI_freq += float_TO_PURE_RSSI(direct ? +6.0 : 0); // TODO add impact of direct
#endif

    } else {
#ifdef __MCU_CLOCK_SHIFT__
      clock_at_48MHz();
#endif

#define INITIAL_MODULATION_CORRECTION   -180

#ifdef TINYSA4                      // Calculate AM/FM modulation
      if (setting.modulation == MO_AM) {
        int sine_wave_index = 1;
        config.cor_am = INITIAL_MODULATION_CORRECTION;        // Initialize with some spare
        modulation_steps = MAX_MODULATION_STEPS; // Search modulation steps that fit frequency
        while ( (modulation_delay = ((float)8000000/ modulation_steps ) / setting.modulation_frequency + config.cor_am) < 20 && modulation_steps > 4) {
          sine_wave_index <<= 1;
          modulation_steps >>= 1;
        }
        int offset347 = (setting.modulation_depth_x100 < 100 ? setting.modulation_depth_x100 - 6: setting.modulation_depth_x100) * 347 / 100;
        for (int i = 0; i < modulation_steps/4+1; i++) {
          fm_modulation[i] = offset347 * sine_wave[i*sine_wave_index]/347;
          fm_modulation[modulation_steps/2 - i] = fm_modulation[i];
          fm_modulation[modulation_steps/2 + i] = - offset347 * sine_wave[i*sine_wave_index]/347;
          fm_modulation[modulation_steps - i] = fm_modulation[modulation_steps/2 + i];
        }
        for (int i=0; i < modulation_steps; i++) {
          fm_modulation[i] = roundf(20.0*logf(1.0+(fm_modulation[i]/347.0)));
          if (fm_modulation[i] > 12)
            fm_modulation[i] = 12;
          if (fm_modulation[i] < -63)
            fm_modulation[i] = -63;
        }
      }
      if (setting.modulation == MO_WFM) {
        int sine_wave_index = 1;
        config.cor_am = INITIAL_MODULATION_CORRECTION;        // Initialize with some spare
        modulation_steps = MAX_MODULATION_STEPS; // Search modulation steps that fit frequency
        //modulation_steps = 8;  // <-----------------TEMP!!!!!
        while ( ((modulation_delay = ((float)8000000/ modulation_steps ) / setting.modulation_frequency + config.cor_am)) < 900 && modulation_steps >= 4) {
          sine_wave_index <<= 1;
          modulation_steps >>= 1;
        }
        if (modulation_steps >= 4) {
          for (int i = 0; i < modulation_steps/4+1; i++) {
            fm_modulation[i] = setting.modulation_deviation_div100 * sine_wave[i*sine_wave_index]/100;
            fm_modulation[modulation_steps/2 - i] = fm_modulation[i];
            fm_modulation[modulation_steps/2 + i] = -fm_modulation[i];
            fm_modulation[modulation_steps - i] = -fm_modulation[i];
          }
        } else {
          modulation_steps = 2;
          fm_modulation[0] = (setting.modulation_deviation_div100*347)/100;
          fm_modulation[1] = -fm_modulation[0];
        }
      }
#else
      config.cor_am = INITIAL_MODULATION_CORRECTION;
#endif


    }
    //    if (MODE_OUTPUT(setting.mode) && setting.additional_step_delay_us < 500)     // Minimum wait time to prevent LO from lockup during output frequency sweep
    //      setting.additional_step_delay_us = 500;
    // Update grid and status after
    if (break_on_operation  && MODE_INPUT(setting.mode)) {                       // during normal operation
      redraw_request |= REDRAW_CAL_STATUS;
      if (FREQ_IS_CW()) {                                       // if zero span mode
        update_grid();                                          // and update grid and frequency
      }
    }
  }
  if (i == 0) {
    for (int t=0;t<TRACES_MAX;t++)
      setting.scan_after_dirty[t] += 1;
  }
  TRACE(0);
  // ---------------------------------  Pulse at start of low output sweep --------------------------

  if ((setting.mode == M_GENLOW  && ( setting.frequency_step != 0 || setting.level_sweep != 0.0)) || (setting.mode == M_LOW && setting.pulse)) {// pulse high out
#ifdef __SI4432__
      SI4432_Sel = SI4432_LO ;
#endif
    if (i == 0) {
//      set_switch_transmit();
#ifdef __SI4432__
      SI4432_Write_Byte(SI4432_GPIO2_CONF, 0x1D) ; // Set GPIO2 output to high
#endif
#ifdef __SI4463__
      SI4463_set_gpio(0, SI446X_GPIO_MODE_DRIVE1);
#endif
    } else if (i == 1) {
//      set_switch_off();
#ifdef __SI4432__
      SI4432_Write_Byte(SI4432_GPIO2_CONF, 0x1F) ; // Set GPIO2 output to ground
#endif
#ifdef __SI4463__
      SI4463_set_gpio(0, SI446X_GPIO_MODE_DRIVE0);
#endif
    }
  }
#ifdef TINYSA4
#if 0   // moved to set_freq
  // ----------------------------- set mixer drive --------------------------------------------
  int target_drive = setting.lo_drive;
  if (target_drive & 0x04){     // Automatic mixer drive
    if (f < 100000000ULL)       // below 100MHz
      target_drive = 0;
    else if (f < 2400000000ULL) // below 2.4GHz
      target_drive = 1;
    else if (f < 3000000000ULL)  // below 3GHz
      target_drive = 2;
    else
      target_drive = 3;
  }
  ADF4351_drive(target_drive);       // Max drive
#endif
#endif
#ifdef TINYSA3
#ifdef __ULTRA__
  int target_drive = setting.lo_drive;
  if (f > ULTRA_MAX_FREQ)
    target_drive += 1;
  if (old_drive != target_drive) {
    SI4432_Drive(target_drive);
    old_drive = target_drive;
  }
#endif
#endif

  // ------------------------------------- START Set the output level ----------------------------------

  if (( setting.frequency_step != 0 || setting.level_sweep != 0.0 || (i == 0 && scandirty))) {     // Initialize or adapt output levels
#ifdef TINYSA4
    if (setting.mode == M_LOW) {
      set_input_path(f);
    }
    else
#endif
    if (setting.mode == M_GENLOW) {// if in low output mode and level sweep or frequency weep is active or at start of sweep
        float ls=setting.level_sweep;                                           // calculate and set the output level
        if (ls > 0)
          ls += 0.5;
        else if (ls < 0)
          ls -= 0.5;
        float a = ((int)((setting.level + ((float)i / sweep_points) * ls)*2.0)) / 2.0 /* + get_level_offset() */ ;
#ifdef TINYSA4
        set_output_path(f, a);
#else
        int d;
#if 0
        if (force_signal_path) {
          setting.atten_step = test_output_switch;
          d = test_output_drive;
          setting.attenuate_x2 = test_output_attenuate;
          goto set_path;
        }
#endif
        correct_RSSI_freq = get_frequency_correction(f);
        a += PURE_TO_float(correct_RSSI_freq);
        if (a != old_a) {
          old_a = a;
          a = a - level_max();                 // convert to all settings maximum power output equals a = zero
          if (a < -SWITCH_ATTENUATION) {
            a = a + SWITCH_ATTENUATION;
            setting.atten_step = true;
          } else {
            setting.atten_step = false;
          }
#define LOWEST_LEVEL MIN_DRIVE
          d = MAX_DRIVE-3;        // Start in the middle
          float blw =  BELOW_MAX_DRIVE(d);
          while (a > blw && d < MAX_DRIVE) { // Increase if needed
            d++;
            blw =  BELOW_MAX_DRIVE(d);
          }
          while (a + 28 < blw && d > LOWEST_LEVEL) { // reduce till it fits attenuator (31 - 3)
            d--;
            blw =  BELOW_MAX_DRIVE(d);
          }
          a -= blw;
          if (a > 0)
            a = 0;
          if (a < -31.5)
            a = -31.5;
          a = -a - 0.25;        // Rounding
          setting.attenuate_x2 = (int)(a * 2);
//        set_path:
          SI4432_Sel = SI4432_RX ;
          SI4432_Drive(d);
          SI4432_Sel = SI4432_RX ;
          if ( setting.atten_step)
            set_switch_receive();
          else
            set_switch_transmit();
          PE4302_Write_Byte(setting.attenuate_x2);
#if 0
          if (debug_level && SDU1.config->usbp->state == USB_ACTIVE)
             shell_printf ("level=%f, d=%d, a=%d, s=%d\r\n", setting.level, d, setting.attenuate_x2, (setting.atten_step ? 1 : 0));
#endif
        }
#endif
    }
    else if (setting.mode == M_GENHIGH) {
#ifdef TINYSA4
      if (force_signal_path) {
        enable_rx_output(!test_output_switch);
        SI4463_set_output_level(test_output_drive);
      } else
#endif
      {
      float a = setting.level - level_max();
      if (a <= -SWITCH_ATTENUATION) {
        setting.atten_step = true;
        a = a + SWITCH_ATTENUATION;
#ifdef TINYSA3
        SI4432_Sel = SI4432_LO ;
        set_switch_receive();
#else
        enable_ADF_output(true, false);
;
#endif
      } else {
        setting.atten_step = false;
#ifdef TINYSA3
        SI4432_Sel = SI4432_LO ;
        set_switch_transmit();
#else
        enable_ADF_output(true, true);

#endif
      }

      int d = MIN_DRIVE;
      while (drive_dBm[d] - level_max() < a && d < MAX_DRIVE)       // Find level equal or above requested level
        d++;
      //    if (d == 8 && v < -12)  // Round towards closest level
      //      d = 7;
      setting.level = drive_dBm[d] + config.high_level_output_offset - (setting.atten_step ? SWITCH_ATTENUATION : 0);

#ifdef __SI4432__
        SI4432_Sel = SI4432_LO ;
        SI4432_Drive(d);
#endif
#ifdef TINYSA4
#ifndef __NEW_SWITCHES__
        if (config.high_out_adf4350)
          SI4463_set_output_level(d);
        else
#endif
 //         ADF4351_aux_drive(d);
#endif
      }
    }
  }

  // ------------------------------------- END Set the output level ----------------------------------

#ifdef __SI4432__
  if (setting.mode == M_LOW && S_IS_AUTO(setting.agc) && !check_for_AM && UNIT_IS_LOG(setting.unit)) {   // If in low input mode with auto AGC and log unit
    if (f < 1500000)
      auto_set_AGC_LNA(false, f*9/1500000);
    else
      auto_set_AGC_LNA(true, 0);
  }
#endif


  // ----------------------------- Initiate modulation ---------------------------

  int *current_fm_modulation = 0;
  if (MODE_OUTPUT(setting.mode)) {
    if (setting.modulation != MO_NONE && setting.modulation != MO_EXTERNAL && setting.modulation_frequency != 0) {
      modulation_counter = 0;
      cycle_counter = 0;
      modulation_delay = ((float)8000000/ modulation_steps ) / setting.modulation_frequency;     // 8 steps so 8MHz/8
      modulation_delay += config.cor_am;
#ifdef TINYSA3
      if (setting.modulation == MO_WFM)
        modulation_table = 1;
#endif
#ifdef TINYSA4
      current_fm_modulation = (int *)fm_modulation;
#else
      if(setting.modulation == MO_WFM || setting.modulation == MO_NFM) {
        if ((setting.mode == M_GENLOW  && f > ((freq_t)480000000) - DEFAULT_IF) ||
            (setting.mode == M_GENHIGH  && f > ((freq_t)480000000) ) )
          modulation_table += 2;
        current_fm_modulation = (int *)fm_modulation[modulation_table];
        f -= fm_modulation_offset[modulation_table];           // Shift output frequency
      }
#endif
    }
  }
modulation_again:
  // -----------------------------------------------------  apply modulation for output modes ---------------------------------------
  if (MODE_OUTPUT(setting.mode) && setting.modulation != MO_NONE && setting.modulation != MO_EXTERNAL){
    if (setting.modulation == MO_AM) {               // AM modulation
#ifdef TINYSA4
      int p = setting.attenuate_x2 - fm_modulation[modulation_counter];
#else
      int p = setting.attenuate_x2 + am_modulation[modulation_counter];
#endif
      if      (p>63) { p = 63;}
      else if (p< 0) {
        p =  0;
#ifdef TINYSA4
        if (!depth_error) { depth_error = true; redraw_request |= REDRAW_CAL_STATUS; draw_all(true);}
#endif
      }
#ifdef __PE4302__
      PE4302_Write_Byte(p);
#endif
    }
    else if (current_fm_modulation) { // setting.modulation == MO_NFM || setting.modulation == MO_WFM  //FM modulation
#ifdef __SI4432__
      SI4432_Sel = SI4432_LO ;
      int offset = current_fm_modulation[modulation_counter];
      SI4432_Write_2_Byte(SI4432_FREQ_OFFSET1, (offset & 0xff ), ((offset >> 8) & 0x03 ));  // Use frequency hopping channel for FM modulation
//      SI4432_Write_Byte(SI4432_FREQ_OFFSET2, );  // Use frequency hopping channel for FM modulation
#endif
#ifdef __SI4468__
      si_fm_offset(current_fm_modulation[modulation_counter]);
#endif
    }
    modulation_counter++;
    if (modulation_counter >= modulation_steps) {
      modulation_counter = 0;
      cycle_counter++;
      if (config.cor_am == INITIAL_MODULATION_CORRECTION) {
        int cycle_time = chVTGetSystemTimeX() - start_of_sweep_timestamp;
        if (cycle_time > 1000) {    // 100 ms, System tick 10000 per second
         start_of_sweep_timestamp = chVTGetSystemTimeX();
          modulation_delay -= config.cor_am;
          int actual_delay = 800 * cycle_time / cycle_counter / modulation_steps;           // In units of 1/8 microsecond
          config.cor_am += modulation_delay - actual_delay;
          if (config.cor_am >0)
            config.cor_am = 0;
          modulation_delay += config.cor_am;
          cycle_counter = 0;
        }
        my_veryfast_delay(modulation_delay);
      }
      else
        my_veryfast_delay(modulation_delay);
    } else
      my_veryfast_delay(modulation_delay);
  }
    // -------------------------------- Acquisition loop for one requested frequency covering spur avoidance and vbwsteps ------------------------
  pureRSSI_t RSSI = float_TO_PURE_RSSI(-150);
  if (debug_avoid){                 // For debugging the spur avoidance control
	stored_t[i] = -90.0;                                  // Display when to do spur shift in the stored trace
  }
  int local_vbw_steps = vbwSteps;
  freq_t local_IF;
#ifdef TINYSA4
  local_IF = config.frequency_IF1;
#if 0
  if (setting.mode == M_LOW && setting.frequency_step > 0 && config.ultra &&
      ((f < ULTRA_MAX_FREQ &&  f > MAX_LO_FREQ - local_IF) ||
       ( f > ultra_start && f < MIN_BELOW_LO + local_IF))
      ) {
    local_vbw_steps *= 2;
  }
#endif
#endif
  TRACE(1);
  // -----------------------------------START vbwsteps loop ------------------------------------
  int t = 0;
  do {
    freq_t lf = f;
    if (local_vbw_steps > 1) {          // Calculate sub steps
#ifdef TINYSA4
	int offs_div10 = (t - (local_vbw_steps >> 1)) * 100;    // steps of x10 * settings.
      if ((local_vbw_steps & 1) == 0)                           // Uneven steps, center
        offs_div10+= 50;                              // Even, shift half step
      int offs = (offs_div10 * (int32_t)frequency_step_x10 )/ local_vbw_steps;
 //     if (setting.step_delay_mode == SD_PRECISE)
 //       offs>>=1;                                        // steps of a quarter rbw
 //     if (lf > -offs)                                   // No negative frequencies
      if (offs >= 0 || lf > (unsigned int)(-offs))
        lf += offs;
//        if (lf > MAX_LO_FREQ)
//          lf = 0;
#else
     int offs_div10 = (t - (local_vbw_steps >> 1)) * 500 / 10; // steps of half the rbw
      if ((local_vbw_steps & 1) == 0)                           // Uneven steps, center
        offs_div10+= 250 / 10;                           // Even, shift half step
      int offs = offs_div10 * actual_rbw_x10;
      if (setting.step_delay_mode == SD_PRECISE)
        offs>>=1;                                        // steps of a quarter rbw
      if (offs < 0 && ((freq_t)-offs) > lf)
        lf = 0;
      else
        lf += offs;
#endif
    }
    int spur_second_pass = false;

#ifdef TINYSA4
    int direct = (signal_path == PATH_DIRECT || signal_path == PATH_LEAKAGE);
#else
    int direct = MODE_HIGH(setting.mode);
#endif


    if (/* MODE_INPUT(setting.mode) && */ i > 0 && FREQ_IS_CW())              // In input mode in zero span mode after first setting of the LO's
      goto skip_LO_setting;                                             // No more LO changes required, save some time and jump over the code


    // -------------- START Calculate the IF -----------------------------

#ifdef __SPUR__
again:                                                              // Spur reduction jumps to here for second measurement
#endif

    local_IF=0;                                                     // For all high modes
#ifdef TINYSA4
    LO_spur_shifted = false;
    LO_mirrored = false;
    LO_shifting = false;
#endif
#ifdef __ULTRA__
//    LO_harmonic = false;
#endif
    if (MODE_LOW(setting.mode)){                                       // All low mode
      if (!setting.auto_IF)
        local_IF = setting.frequency_IF;
      else
      {
#ifdef TINYSA4
        if (actual_rbw_x10 < RBW_FOR_STATIC_TABLE && setting.mode == M_LOW && lf > static_spur_table[0] -  RBW_FOR_STATIC_TABLE * 100)
          local_IF = spur_IF; // static spur table IF
        else
          local_IF = config.frequency_IF1;
#if 0
        if ( S_IS_AUTO(setting.below_IF)) {
//          if (f < 2000000 && S_IS_AUTO(setting.spur_removal))
//            local_IF += DEFAULT_SPUR_OFFSET;
//          else // if (lf > ULTRA_MAX_FREQ || lf < local_IF/2  || ( lf + (uint64_t)local_IF< MAX_LO_FREQ && lf > 136000000ULL + local_IF) )
            local_IF += DEFAULT_SPUR_OFFSET/2;
        }
#endif
#else
        local_IF = DEFAULT_IF;
#endif
      }
      if (setting.mode == M_LOW && !direct) {
        if (tracking) {                                // VERY SPECIAL CASE!!!!!   Measure BPF
#if 0                                                               // Isolation test
          local_IF = lf;
          lf = 0;
#else
          local_IF += lf - (setting.refer == -1 ? 0 : reffer_freq[setting.refer]);    // Offset so fundamental of reffer is visible
          lf = (setting.refer == -1 ? 0 : reffer_freq[setting.refer]);
#endif
        } else {
#ifdef __ULTRA__
          if (S_IS_AUTO(setting.spur_removal)) {
            if (config.ultra && lf >= ultra_start) {
              setting.spur_removal= S_AUTO_ON;
            } else {
              setting.spur_removal= S_AUTO_OFF;
            }
          }
#endif

#ifdef __ULTRA__
          if (S_IS_AUTO(setting.below_IF)) {
            if ((freq_t)lf > MAX_ABOVE_IF_FREQ && lf <= ULTRA_MAX_FREQ && !LO_harmonic)
              setting.below_IF = S_AUTO_ON; // Only way to reach this range. Use below IF in harmonic mode
            else
              setting.below_IF = S_AUTO_OFF; // default is above IF, Use below IF in harmonic mode
          }
#endif
          if (S_STATE(setting.spur_removal)){         // If in low input mode and spur reduction is on
            if (setting.below_IF == S_AUTO_OFF &&       // Auto and not yet in below IF
#ifdef TINYSA4
                ( lf > ULTRA_MAX_FREQ ||    // Harmonic mode
                    lf < 400000000 ||       // below 400MHz use below IF
                    ( lf < MAX_ABOVE_IF_FREQ && lf > MIN_BELOW_IF_FREQ) ) // From 2.1GHz to 3.35GHz use below IF
#else
#ifdef __ULTRA__
                ( (lf > ULTRA_MAX_FREQ && (lf + local_IF) / setting.harmonic < MAX_LO_FREQ) || lf < local_IF - MIN_LO_FREQ  || ( lf + (uint32_t)local_IF< MAX_LO_FREQ && lf > MIN_BELOW_LO + local_IF) )
#else
                (lf < local_IF / 2  || lf > local_IF)
#endif
#endif
                )
            {              // below/above IF
              if ((debug_avoid && debug_avoid_second) || spur_second_pass) {
                setting.below_IF = S_AUTO_ON;
              } else {
                setting.below_IF = S_AUTO_OFF;               // use below IF in second pass
              }
            }
            else // if (setting.auto_IF)
            {
              if ((debug_avoid && debug_avoid_second) || spur_second_pass) {
#ifdef TINYSA4
                local_IF  = local_IF + DEFAULT_SPUR_OFFSET-(actual_rbw_x10 > 1000 ? 200000 : 0);    // apply IF spur shift
                LO_spur_shifted = true;
#ifdef TINYSA4
                LO_shifting = true;
#endif

              } else {
                local_IF  = local_IF; // - (actual_rbw_x10 > 5000 ? 200000 : 0);// - DEFAULT_SPUR_OFFSET/2;    // apply IF spur shift
              }
#else
                local_IF  = local_IF + 500000;                  // apply IF spur shift
              }
#endif
            }
          } else {
            int spur_flag = avoid_spur(lf);
#ifdef TINYSA4
#if 0
            if (debug_avoid) {
              if (spur_flag == F_NEAR_SPUR) {
                stored_t[i] = -70.0;                                       // Display when to do spur shift in the stored trace
  //              local_IF -= DEFAULT_SPUR_OFFSET/2;
              } else if (spur_flag == F_AT_SPUR){
                stored_t[i] = -60.0;
                // Display when to do spur shift in the stored trace
                if (debug_avoid_second) {
                  if (S_IS_AUTO(setting.below_IF) && lf < local_IF/2 - 2000000) {
                    setting.below_IF = S_AUTO_ON;
                    local_IF = local_IF;                          // No spur removal and no spur, center in IF
                  } else if (setting.auto_IF) {
                    local_IF = local_IF + DEFAULT_SPUR_OFFSET/2;
                    //                if (actual_rbw_x10 == 6000 )
                    //                  local_IF = local_IF + 50000;
                    LO_spur_shifted = true;
                  }
                }
              } else {
                stored_t[i] = -90.0;                                  // Display when to do spur shift in the stored trace
              }
            } else
#endif
            if(spur_flag) {         // check if alternate IF is needed to avoid spur.
              if (spur_flag == F_NEAR_SPUR) {
                if (debug_avoid) stored_t[i] = -70.0;                                  // Display when to do spur shift in the stored trace

//                local_IF -= DEFAULT_SPUR_OFFSET/2;
              } else if (spur_flag == F_AT_SPUR && (!debug_avoid || debug_avoid_second)) {
                if (debug_avoid) stored_t[i] = -60.0;                                  // Display when to do spur shift in the stored trace
#ifndef TINYSA4
                if (S_IS_AUTO(setting.below_IF) && lf < local_IF/2 - 2000000) {
                  setting.below_IF = S_AUTO_ON;
                  local_IF = local_IF;                          // No spur removal and no spur, center in IF
                } else
#endif
                if (setting.auto_IF) {
                  local_IF = local_IF + (actual_rbw_x10 > 2000 ? DEFAULT_SPUR_OFFSET : DEFAULT_SPUR_OFFSET/2); // TODO find better way to shift spur away at large RBW/2;
                  //                if (actual_rbw_x10 == 6000 )
                  //                  local_IF = local_IF + 50000;
                  LO_spur_shifted = true;
                }
              } else
              {
                if (debug_avoid) stored_t[i] = -90.0;                                  // Display when to do spur shift in the stored trace
              }
            }
#else
            if(spur_flag) {         // check if alternate IF is needed to avoid spur.
              local_IF = spur_alternate_IF;
              if (debug_avoid){                 // For debugging the spur avoidance control
                stored_t[i] = -60.0;                                       // Display when to do spur shift in the stored trace
              }
            }
#endif
            else
            {
#ifdef TINYSA4
              if (lf<LOW_SHIFT_FREQ) {   // below 2MHz
                local_IF += DEFAULT_SPUR_OFFSET-(actual_rbw_x10 > 1000 ? 200000 : 0);                  // Shift to avoid zero Hz peak
                LO_shifting = true;
              }
              if (config.hide_21MHz && S_IS_AUTO(setting.below_IF)) {
                if (binary_search_table(lf, additional_spur_table, ADDITIONAL_SPUR_TABLE_SIZE, ADDITIONAL_SPUR_TABE_GATE))
                {
                  setting.below_IF= S_AUTO_ON;
                } else {
                  setting.below_IF= S_AUTO_OFF;
                }
              }

#else
              local_IF = local_IF; // + DEFAULT_SPUR_OFFSET/2;                  // No spure removal and no spur, center in IF
#endif
            }
          }
        }
      } else {              // Output mode
#ifdef __ULTRA__
        if (S_IS_AUTO(setting.below_IF)) {
          if ((freq_t)lf > MAX_ABOVE_IF_FREQ && lf <= ULTRA_MAX_FREQ + 60000000)
            setting.below_IF = S_AUTO_ON; // Only way to reach this range. Use below IF in harmonic mode
          else
            setting.below_IF = S_AUTO_OFF; // default is above IF, Use below IF in harmonic mode
        }
#endif
        if (setting.modulation == MO_EXTERNAL)    // VERY SPECIAL CASE !!!!!! LO input via high port
          local_IF += lf;
      }
    } // --------------- END IF calculation ------------------------
  TRACE(2);
    // ------------- Set LO ---------------------------

    {                                           // Else set LO ('s)
      freq_t target_f;
#ifdef TINYSA4
      int inverted_f = false;
#endif
      if ((setting.mode == M_LOW ||setting.mode == M_GENLOW)  && !direct && !setting.tracking && S_STATE(setting.below_IF)) { // if in low input mode and below IF
#ifdef __ULTRA__
        if (lf < local_IF)
#endif
          target_f = local_IF-lf;                                                 // set LO SI4432 to below IF frequency
#ifdef __ULTRA__
        else {
          target_f = lf - local_IF;                                                 // set LO SI4432 to below IF frequency
#ifdef TINYSA4
          inverted_f = true;
          LO_mirrored = true;
#endif
        }
#endif
      }
      else
        target_f = local_IF+lf;                                                 // otherwise to above IF, local_IF == 0 in high mode

#ifdef __SI4432__
#ifdef __HARMONIC__
#ifdef TINYSA3
      if (setting.harmonic) {
        if (spur_second_pass) {
          if (setting.harmonic == 2)
            target_f /= setting.harmonic+1;
          else
            target_f /= setting.harmonic+2;
        }
        else
          target_f /= setting.harmonic;
      }
#endif
#endif
#ifdef __ULTRA__
      if (LO_harmonic) {
        target_f /= setting.harmonic;
#ifdef TINYSA3
        if (target_f > MAX_LO_FREQ) {
          target_f = (lf - local_IF) / setting.harmonic;
        }
#endif
//        LO_harmonic = true;
      }
#endif
      set_freq (SI4432_LO, target_f);                                                 // otherwise to above IF
#endif

// ----------------------------- START Calculate and set the AD4351 frequency  and set the RX frequency --------------------------------

#ifdef __ADF4351__
//      START_PROFILE;
      if (MODE_LOW(setting.mode) &&!direct) {
        if (config.frequency_IF2 != 0) {
          set_freq (ADF4351_LO2, config.frequency_IF2  - local_IF);          // Down from IF2 to fixed second IF in Ultra SA mode
          local_IF = config.frequency_IF2;
        }

#if 1
#define TCXO    30000000
#define TXCO_DIV3   10000000

#define AVOID_MULTI 150

#ifdef __SI5351__
        if (si5351_available) {
        if (setting.R == 0) {
          if (setting.mode == M_GENLOW) {
            if (local_modulo == 0) ADF4351_modulo(1000);
            ADF4350_shift_ref(false);
            ADF4351_R_counter(3);
          } else if (lf > 8000000 && MODE_INPUT(setting.mode)) {
            if (local_modulo == 0) ADF4351_modulo(4000);
            ADF4351_R_counter(1);

            freq_t tf = ((lf + actual_rbw_x10*AVOID_MULTI) / TCXO) * TCXO;
            if (tf + actual_rbw_x10*AVOID_MULTI >= lf  && tf < lf + actual_rbw_x10*AVOID_MULTI /* && actual_rbw_x10 < 300 */) {   // 30MHz
              ADF4350_shift_ref(true);
            } else {
              ADF4350_shift_ref(false);
            }
          } else
          if (get_sweep_frequency(ST_SPAN)<5000000) { // When scanning less then 5MHz
            if (actual_rbw_x10 <= 3000) {
              freq_t tf = ((lf + actual_rbw_x10*1000) / TXCO_DIV3) * TXCO_DIV3;
              if (tf + actual_rbw_x10*100 >= lf  && tf < lf + actual_rbw_x10*100) // 10MHz
                ADF4351_R_counter(2);    // To avoid PLL Loop shoulders at multiple of 10MHz
              else
                ADF4351_R_counter(3);     // To avoid PLL Loop shoulders
            } else
              ADF4351_R_counter(1);
          } else
            ADF4351_R_counter(1);

        } else {
          freq_t tf = ((lf + actual_rbw_x10*AVOID_MULTI) / TXCO_DIV3) * TXCO_DIV3;
          if (tf + actual_rbw_x10*AVOID_MULTI >= lf  && tf < lf + actual_rbw_x10*AVOID_MULTI  && actual_rbw_x10 < 300) {   // 30MHz
            ADF4350_shift_ref(true);
          } else {
            ADF4350_shift_ref(false);
          }
          ADF4351_R_counter(setting.R);
        }
        } else
#endif
        {
        if (setting.R == 0) {
          if (setting.mode == M_GENLOW) {
            if (max2871) {
              if (local_modulo == 0) ADF4351_modulo(1000);
              ADF4351_R_counter(-1);
            } else {
              if (local_modulo == 0) ADF4351_modulo(1000);
              ADF4351_R_counter(3);
            }
          } else if (lf < 25000000 && max2871) {
            ADF4351_R_counter(-1);
            ADF4351_modulo(200);
          } else if (lf > 8000000  && !(lf > 1000000000 && hw_if) && MODE_INPUT(setting.mode)) {
            if (local_modulo == 0) {
              if (max2871)
                ADF4351_modulo(100);
              else
                ADF4351_modulo(4000);
            }
            freq_t tf = ((lf + actual_rbw_x10*AVOID_MULTI) / TCXO) * TCXO;
            if (tf + actual_rbw_x10*AVOID_MULTI >= lf  && tf < lf + actual_rbw_x10*AVOID_MULTI /* && tf != 180000000 */ ) {   // 30MHz
              if (max2871) {
                if (lf > 59000000) {
//                  if (tf == 180000000) {
//                    ADF4351_R_counter(7);
//                  } else
                  ADF4351_R_counter(8);
                }
              } else {
                if ( (tf / TCXO) & 1 ) {    // Odd harmonic of 30MHz
                  ADF4351_R_counter(-3);
                } else {
                  if (hw_if)
                    ADF4351_R_counter(7);
                  else
                    ADF4351_R_counter(4);
                }
              }
            }
#if 0
            else if (actual_rbw_x10 < 1000) {
              freq_t tf = ((lf + actual_rbw_x10*1000) / TXCO_DIV3) * TXCO_DIV3;
              if (tf + actual_rbw_x10*100 >= lf  && tf < lf + actual_rbw_x10*100) // 10MHz
                ADF4351_R_counter(4);
              else
                ADF4351_R_counter(3);
            }
#endif
            else if (get_sweep_frequency(ST_SPAN)<5000000) { // When scanning less then 5MHz
              if (actual_rbw_x10 <= 3000) {
                freq_t tf = ((lf + actual_rbw_x10*1000) / TXCO_DIV3) * TXCO_DIV3;
                if (tf + actual_rbw_x10*1000 >= lf  && tf < lf + actual_rbw_x10*1000) // 10MHz
                  ADF4351_R_counter(-4);    // To avoid PLL Loop shoulders at multiple of 10MHz
                else {
                  if (hw_if)
                    ADF4351_R_counter(1);
                  else
                    ADF4351_R_counter(3);     // To avoid PLL Loop shoulders
                }
              } else
                ADF4351_R_counter(1);
            }
            else if (max2871)
              ADF4351_R_counter(1);
            else
              ADF4351_R_counter(1);

          } else {                          // Input above 800 MHz
            if (local_modulo == 0) {
//              if (actual_rbw_x10 >= 3000)
                if (max2871)
                  ADF4351_modulo(4000);
                else
                  ADF4351_modulo(4000);
//              else
//                ADF4351_modulo(60);
            }
#if 0
            if (setting.frequency_step < 100000) {
              ADF4351_R_counter(3);
            } else
#endif
            if (max2871)
              ADF4351_R_counter(1); // Used to be 1
            else
              ADF4351_R_counter(1); // Used to be 1
          }
        } else {
          ADF4351_R_counter(setting.R%1000);
        }
        }
#endif          // __ADF4351__
        TRACE(2.5);
        if (LO_harmonic) {
          target_f /= setting.harmonic;
        }
        // ----------------------------- set mixer drive --------------------------------------------
        if (setting.mode == M_LOW || setting.mode == M_GENLOW) {

          //
          //  ----------------- start duplication of code
          //
          actual_drive = setting.lo_drive;
          if (actual_drive & 0x04){     // Automatic mixer drive
            if (LO_harmonic)
              actual_drive = 3;
//            else if (lf <  DRIVE0_MAX_FREQ)       // below 600MHz
//              actual_drive = 0;
            else if (lf < DRIVE1_MAX_FREQ || hw_if) // below 1.2GHz
              actual_drive = 1;
            else if (lf < DRIVE2_MAX_FREQ)  // below 2GHz
              actual_drive = 2;
            else
              actual_drive = 3;
          }
          //
          //  ----------------- end duplication of code
          //
          if (signal_path == PATH_LOW || signal_path == PATH_ULTRA) ADF4351_drive(actual_drive);       // Max drive
        }
        if (signal_path == PATH_LEAKAGE)
          set_freq(ADF4351_LO, target_f);       // Never used as leakage is taken care under direct
        else {
#define MAX_COMPENSATION    50000
          if ( target_f < real_old_freq[ADF4351_LO] - MAX_COMPENSATION || target_f > real_old_freq[ADF4351_LO] + MAX_COMPENSATION )
            set_freq(ADF4351_LO, target_f);
        }

#if 1                                                               // Compensate frequency ADF4350 error with SI4468
        if (actual_rbw_x10 < 10000 || setting.frequency_step < 100000) { //TODO always compensate for the moment as this eliminates artifacts at larger RBW
        int32_t error_f = 0;
        if (real_old_freq[ADF4351_LO] > target_f) {
          error_f = real_old_freq[ADF4351_LO] - target_f;

          if (inverted_f) {
            error_f = -error_f;
            goto correct_min;
          }
        correct_plus:
         if (LO_harmonic)
            error_f *= setting.harmonic;
          if (error_f > actual_rbw_x10 * 5 || LO_harmonic) {        //RBW / 4
            local_IF += error_f;
          }
        } else if ( real_old_freq[ADF4351_LO] < target_f) {
          error_f = real_old_freq[ADF4351_LO] - target_f;
          if (inverted_f) {
            error_f = -error_f;
            goto correct_plus;
          }
        correct_min:
          if (LO_harmonic) {
            error_f *= setting.harmonic;
          }
          if ( error_f < - actual_rbw_x10 * 5 || LO_harmonic) {    //RBW / 4
            local_IF += error_f;
          }
        }
        }
#endif
      } else if (setting.mode == M_HIGH || direct) {        // direct means PATH_DIRECT or PATH_LEAKAGE
        if (signal_path == PATH_DIRECT) {
          set_freq (SI4463_RX, lf); // sweep RX, local_IF = 0 in high mode
          if (setting.tracking_output)
            set_freq (ADF4351_LO, lf);
        } else {   //PATH_LEAKAGE
          freq_t target_f = lf;
          if (LO_harmonic)
            target_f /= setting.harmonic;
          set_freq (ADF4351_LO, target_f); // sweep LO, local_IF = 0 in high mode
        }
        local_IF = 0;
      } else if (setting.mode == M_GENHIGH) {
        local_IF = 0;
#ifndef __NEW_SWITCHES__
        if (!config.high_out_adf4350)
          set_freq (SI4463_RX, lf); // sweep RX, local_IF = 0 in high mode
        else
#endif
          set_freq (ADF4351_LO, lf); // sweep LO, local_IF = 0 in high mode
      }
      // ----------------------------- END Calculate and set the AD4351 frequency and set the RX frequency --------------------------------

//      STOP_PROFILE;
#endif
    }       // ----------------- LO's set --------------------------

    TRACE(2.5)
#ifdef __MCU_CLOCK_SHIFT__
        if (setting.mode == M_LOW && !in_selftest) {         // Avoid 48MHz spur
          int set_below = false;
#ifdef TINYSA4
          if (lf < 40000000) {
            uint32_t tf = lf;
            while (tf > 4000000) tf -= 4000000;
            if (tf < 2000000 )
              set_below = true;
          } else
#endif
          if (lf > 40000000){
            uint32_t tf = lf;
            while (tf > 240000000) tf -= 240000000;       // Wrap between 0-48MHz
            while (tf >  48000000) tf -=  48000000;       // Wrap between 0-48MHz
            if (tf < 20000000 )
              set_below = true;
          }
          if (set_below) {     // If below 48MHz
            if (!is_below) {
              clock_below_48MHz();
              is_below = true;
            }
          } else {
            if (is_below) {
              clock_above_48MHz();
              is_below = false;
            }
          }
        }
#endif
   TRACE(3);
// ----------- Set IF ------------------

    if (local_IF != 0)                  // When not in one of the high modes and not in direct mode
    {
#ifdef __SI4432__
      set_freq (SI4432_RX , local_IF);
#endif
#ifdef __SI4463__
      set_freq (SI4463_RX, local_IF);   // including compensating ADF error with SI446x when not in tracking mode
#endif
    }

    if (MODE_OUTPUT(setting.mode)) {
#ifdef __SI4432__
      my_microsecond_delay(200);                 // To prevent lockup of SI4432
#endif
    }

    TRACE(4);
    // Calculate the RSSI correction for later use
    if (MODE_INPUT(setting.mode)){ // only cases where the value can change on 0 point of sweep
      if (setting.frequency_step != 0 || (i==0 && scandirty)) {
        correct_RSSI_freq = get_frequency_correction(lf);
      }
    }
  // #define DEBUG_CORRECTION
  #ifdef DEBUG_CORRECTION
    if (SDU1.config->usbp->state == USB_ACTIVE) {
      shell_printf ("%d:%Q %f\r\n", i, lf, PURE_TO_float(correct_RSSI_freq));
      osalThreadSleepMilliseconds(2);
    }
  #endif




#ifdef TINYSA4
    if (debug_frequencies ) {

      freq_t mult = (LO_harmonic ? 3 : 1);
      freq_t f_low, f_high;
      if (setting.mode == M_LOW || setting.mode == M_GENLOW) {
        if (real_old_freq[ADF4351_LO] > (real_old_freq[SI4463_RX] + real_offset))
          f_low = (mult*real_old_freq[ADF4351_LO]) - (real_old_freq[SI4463_RX] + real_offset);          // lf below LO
        else
          f_low = (real_old_freq[SI4463_RX] + real_offset) - (mult*real_old_freq[ADF4351_LO]);
        f_high = (mult*real_old_freq[ADF4351_LO]) + (real_old_freq[SI4463_RX] + real_offset);           // lf above LO
      } else
        f_low = f_high = real_old_freq[SI4463_RX] + real_offset;
     float f_error_low, f_error_high;
     float freq = getFrequency(i);
     if (setting.frequency_step == 0) {
         f_error_low  = (freq - f_low);
         f_error_high = (f_high - freq);
     } else {
       f_error_low  = (f_low - freq)/setting.frequency_step;
       f_error_high = (f_high- freq)/setting.frequency_step;
     }
     char spur = ' ';
     int delta=0;
     freq_t tf = (LO_mirrored ? f_high : f_low);
     if ( tf * 4 < real_old_freq[SI4463_RX] + real_offset) {
       delta = real_old_freq[SI4463_RX] + real_offset - 4*tf;
       if (delta < actual_rbw_x10*100)
         spur = '!';
     } else {
       delta = 4*tf - real_old_freq[SI4463_RX] + real_offset;
       if (delta < actual_rbw_x10*100)
         spur = '!';
     }
     char shifted =  (LO_shifting ? '/' : ( LO_spur_shifted ? '>' : ' '));
     if (SDU1.config->usbp->state == USB_ACTIVE)
       shell_printf ("%d:%c%c%c%cLO=%11.6Lq:%11.6Lq\tIF=%11.6Lq:%11.6Lq\tOF=%11.6d\tF=%11.6Lq:%11.6Lq\tD=%.2f:%.2f %c%c%c %d R=%d mod=%d, d=%d\r\n",
                     i,   spur, shifted,(LO_mirrored ? 'm' : ' '), (LO_harmonic ? 'h':' ' ),
                     old_freq[ADF4351_LO],real_old_freq[ADF4351_LO],
                     old_freq[SI4463_RX], real_old_freq[SI4463_RX], (int32_t)real_offset, f_low, f_high , f_error_low, f_error_high,
                     (ADF4351_frequency_changed? 'A' : ' '),
                     (SI4463_frequency_changed? 'S' : ' '),
                     (SI4463_offset_changed? 'O' : ' '),
                     correct_RSSI_freq,
                     R, ADF4351_get_modulo(),rfPower
                    );
     osalThreadSleepMilliseconds(100);
    }
#endif
    // ------------------------- end of processing when in output mode ------------------------------------------------



    skip_LO_setting:
    if (i == 0 && t == 0)                                                   // if first point in scan (here is get 1 point data)
      start_of_sweep_timestamp = chVTGetSystemTimeX();                      // initialize start sweep time

    if (MODE_OUTPUT(setting.mode)) {               // No substepping and no RSSI in output mode
      if (break_on_operation && operation_requested)          // break subscanning if requested
        return(0);         // abort
      if ( i==1 && MODE_OUTPUT(setting.mode) && setting.modulation != MO_NONE && setting.modulation != MO_EXTERNAL) { // if in output mode with modulation and LO setup done
//        i = 1;              // Everything set so skip LO setting

#if 0
#define MODULATION_CYCLES_TEST   100000
        if (in_selftest && modulation_count_iter++ >= MODULATION_CYCLES_TEST) {
          start_of_sweep_timestamp = sa_ST2US(chVTGetSystemTimeX() - start_of_sweep_timestamp)/MODULATION_CYCLES_TEST;  // uS per cycle
          return 0;
        }
#endif
        goto modulation_again;                                             // Keep repeating sweep loop till user aborts by input
      }
      return(0);
    }
    // ---------------- Prepare RSSI ----------------------

    // jump here if in zero span mode and all HW frequency setup is done.

#ifdef __FAST_SWEEP__
#ifdef __SI4432__
    if (i == 0 && setting.frequency_step == 0 && setting.trigger == T_AUTO && S_STATE(setting.spur_removal) == 0 && SI4432_step_delay == 0 && setting.repeat == 1 && setting.sweep_time_us < 100*ONE_MS_TIME) {
      // if ultra fast scanning is needed prefill the SI4432 RSSI read buffer
      if (scandirty)
        my_microsecond_delay(6400);            // Extra time to avoid gap when filling SI4468
      SI4432_Fill(MODE_SELECT(setting.mode), 0);
    }
#endif
#ifdef __SI4463__
    if (i == 0 && setting.frequency_step == 0 && setting.trigger == T_AUTO && S_STATE(setting.spur_removal) == 0 && SI4432_step_delay == 0 && setting.repeat == 1 && setting.sweep_time_us < 100*ONE_MS_TIME && setting.exp_aver == 1) {
      if (scandirty)
        my_microsecond_delay(20000);            // Extra time to avoid gap when filling SI4468
      SI446x_Fill(MODE_SELECT(setting.mode), -1);   // First get_RSSI will fail
    }
#endif
#endif
    pureRSSI_t pureRSSI;
    //    if ( i < 3)
    //      shell_printf("%d %.3f %.3f %.1f\r\n", i, local_IF/1000000.0, lf/1000000.0, subRSSI);

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
#define T_LEVEL_UNDEF       (1<<(16-T_POINTS)) // should drop after 4 shifts left
#define T_LEVEL_BELOW       1
#define T_LEVEL_ABOVE       0
    // Trigger mask, should have width T_POINTS bit
#define T_DOWN_MASK         (0b0001)           // 1 from up 1 to bottom
#define T_UP_MASK           (0b0010)           // 1 from bottom 1 to up
#define T_LEVEL_CLEAN       ~(1<<T_POINTS)     // cleanup old trigger data
#endif

    if (i == 0 && setting.frequency_step == 0 && setting.trigger != T_AUTO) { // if in zero span mode and wait for trigger to happen and NOT in trigger mode

#if 1
      volatile uint8_t trigger_lvl = PURE_TO_DEVICE_RSSI((int16_t)((float_TO_PURE_RSSI(setting.trigger_level) - correct_RSSI - correct_RSSI_freq)));
#ifdef TINYSA4
      SI446x_trigger_fill(MODE_SELECT(setting.mode), trigger_lvl, (setting.trigger_direction == T_UP), setting.trigger_mode);
#else
      SI4432_trigger_fill(MODE_SELECT(setting.mode), trigger_lvl, (setting.trigger_direction == T_UP), setting.trigger_mode);
#endif
#else
      register uint16_t t_mode;
      pureRSSI_t trigger_lvl;
      uint16_t data_level = T_LEVEL_UNDEF;
      // Calculate trigger level
      trigger_lvl = float_TO_PURE_RSSI(setting.trigger_level) - correct_RSSI - correct_RSSI_freq;

      if (setting.trigger_direction == T_UP)
        t_mode = T_UP_MASK;
      else
        t_mode = T_DOWN_MASK;
      uint32_t additional_delay = 0;// reduce noise
      if (setting.sweep_time_us >= 100*ONE_MS_TIME) additional_delay = 20;
#ifdef __SI4432__
      SI4432_Sel =  MODE_SELECT(setting.mode);
#endif
      do{                                                 // wait for trigger to happen
#ifdef __SI4432__
        pureRSSI = DEVICE_TO_PURE_RSSI((deviceRSSI_t)SI4432_Read_Byte(SI4432_REG_RSSI));
#endif
#ifdef __SI4463__
        pureRSSI = Si446x_RSSI(break_on_operation);
        if (LO_shifting && signal_path != PATH_DIRECT)
          pureRSSI += float_TO_PURE_RSSI(actual_rbw_x10>USE_SHIFT2_RBW ? config.shift2_level_offset : (lf < LOW_SHIFT_FREQ ? config.shift1_level_offset: 0.0));

#endif
        if (break_on_operation && operation_requested)                        // allow aborting a wait for trigger
          goto abort; //return 0;                                                           // abort
        // Store data level bitfield (remember only last 2 states)
        // T_LEVEL_UNDEF mode bit drop after 2 shifts
        data_level = ((data_level<<1) | (pureRSSI < trigger_lvl ? T_LEVEL_BELOW : T_LEVEL_ABOVE))&(T_LEVEL_CLEAN);
        if (data_level == t_mode)  // wait trigger
          break;
        if (additional_delay)
          my_microsecond_delay(additional_delay);
      }while(1);
#ifdef __FAST_SWEEP__
#ifdef __SI4432__
      if (S_STATE(setting.spur_removal) == 0 && SI4432_step_delay == 0 && setting.repeat == 1 && setting.sweep_time_us < 100*ONE_MS_TIME) {
        SI4432_Fill(MODE_SELECT(setting.mode), 1);                       // fast mode possible to pre-fill RSSI buffer
      }
#endif
#ifdef __SI4463__

      if (/* S_STATE(setting.spur_removal) == 0 &&  */ SI4432_step_delay == 0 && setting.repeat == 1 && setting.sweep_time_us < 100*ONE_MS_TIME) {
//        if (scandirty)
//          my_microsecond_delay(10000);            // Extra time to avoid gap when filling SI4468
        SI446x_Fill(MODE_SELECT(setting.mode), 1);                       // fast mode possible to pre-fill RSSI buffer
      }
#endif
#endif

#endif
      if (setting.trigger == T_SINGLE) {
        set_trigger(T_DONE);
      }
      start_of_sweep_timestamp = chVTGetSystemTimeX();
    }
    TRACE(5);
#ifdef TINYSA4
    if (SI4432_step_delay && (ADF4351_frequency_changed || SI4463_frequency_changed)) {
      int my_step_delay = SI4432_step_delay;
      if (lf < LOW_SHIFT_FREQ && actual_rbw_x10 == 3 && !in_step_test)
        my_step_delay = my_step_delay * 2;
//      if (LO_spur_shifted) // || SI4463_offset_changed)
//        my_step_delay = my_step_delay * 2;
#if 0   // Always have some delay before measuring RSSI
      if (old_R < 4 && actual_rbw_x10 >= 1000 && SI4463_frequency_changed && ADF4351_frequency_changed) {
        my_step_delay -= 200;                   // compensate for additional delay of setting SI4463
        if (my_step_delay < 0)
          my_step_delay = 0;
      }
#endif
      if (!in_step_test) {
        if (max2871) {
          if (old_R > 2) {
            if (my_step_delay <500)
              my_step_delay *= 3;
            else if (my_step_delay <1000)
                  my_step_delay *= 2;
            else
              my_step_delay *= 1;
          }
        } else {
        if (my_step_delay < 300 && old_R == 1) {
          if ((100000000 < lf && lf <250000000))
            my_step_delay = 300;
        } else
        if (old_R >= 5) {
          if (my_step_delay <500)
            my_step_delay *= 6;
          else if (my_step_delay <1000)
                my_step_delay *= 4;
          else
            my_step_delay *= 2;
        } else if (old_R == 4) {
          if (my_step_delay <500)
            my_step_delay *= 4;
          else if (my_step_delay <1000)
                my_step_delay *= 2;
          else if (my_step_delay <10000)
                my_step_delay += my_step_delay>>1;
          } else if (old_R == 3) {
          if (my_step_delay <500)
            my_step_delay *= 3;
          else if (my_step_delay <1000)
                my_step_delay += my_step_delay>>2;
        } else if (old_R <= -3) {
          if (my_step_delay <1000)
            my_step_delay += my_step_delay >> 1 ;
        }
        }
      }
      TRACE(5.5);
      my_microsecond_delay(my_step_delay);
      ADF4351_frequency_changed = false;
      SI4463_frequency_changed = false;
      SI4463_offset_changed = false;
    } else if (SI4432_offset_delay && SI4463_offset_changed) {
      my_microsecond_delay(SI4432_offset_delay);
      SI4463_offset_changed = false;
    }
#endif
    TRACE(6);
    //else
    {
#ifdef __SI4432__
      pureRSSI = SI4432_RSSI(lf, MODE_SELECT(setting.mode), break_on_operation);            // Get RSSI, either from pre-filled buffer
#endif
#ifdef __SI4463__
      if (real_old_freq[SI4463_RX] == 0)
        pureRSSI = 0;
      else {
        pureRSSI = Si446x_RSSI(break_on_operation);
        if (LO_shifting && (signal_path != PATH_DIRECT)) {
          if (f < 5000000)
            pureRSSI += float_TO_PURE_RSSI(actual_rbw_x10>USE_SHIFT2_RBW ? config.shift2_level_offset : (lf < LOW_SHIFT_FREQ ? config.shift1_level_offset: 0.0));
          else
            pureRSSI += float_TO_PURE_RSSI(config.shift_level_offset);
        }
      }
      if (setting.unit == U_RAW)
        pureRSSI += - float_TO_PURE_RSSI(120); // don't add correction;
      else
        pureRSSI += correct_RSSI + correct_RSSI_freq; // add correction

//#define __DEBUG_FREQUENCY_SETTING__
#ifdef __DEBUG_FREQUENCY_SETTING__                 // For debugging the frequency calculation
  stored_t[i] = -60.0 + (real_old_freq[ADF4351_LO] - f - old_freq[2])/10;
#endif
#endif
    }
//    if (pureRSSI < 400) {
//      volatile int i = 0;
//      i = i + 1;
//   }
#ifdef __ULTRA__
    float debug_rssi = PURE_TO_float(pureRSSI);
#endif
#ifdef __SPUR__
    static pureRSSI_t spur_RSSI = -1;                               // Initialization only to avoid warning.
    if ((setting.mode == M_LOW || setting.mode == M_HIGH) && S_STATE(setting.spur_removal) && !debug_avoid) {
      if (!spur_second_pass) {                                        // If first spur pass
#ifdef __ULTRA__
        if (debug_spur) {
          if (t == 0)
            temp_t[i] = debug_rssi;
          else if (temp_t[i] < debug_rssi)
            temp_t[i] = debug_rssi;
        }
#endif
        spur_RSSI = pureRSSI;                                       // remember measure RSSI
        spur_second_pass = true;
        goto again;                                                 // Skip all other processing
      } else {                              // If second  spur pass
#ifdef __ULTRA__
        if (debug_spur) {
          if (t == 0)
            stored_t[i] = debug_rssi;
          else if (stored_t[i] < debug_rssi)
            stored_t[i] = debug_rssi;
        }
#endif
        pureRSSI = ( pureRSSI < spur_RSSI ? pureRSSI : spur_RSSI);  // Take minimum of two
        if (S_IS_AUTO(setting.below_IF))
          setting.below_IF = S_AUTO_OFF;                            // make sure it is off for next pass
      }
    }
#endif

    if (RSSI < pureRSSI)                                     // Take max during subscanning
      RSSI = pureRSSI;
    t++;                                                    // one subscan done
    if (break_on_operation && operation_requested)          // break subscanning if requested
      break;         // abort
#ifdef __WATCHDOG__
    wdgReset(&WDGD1);
#endif
  } while (t < local_vbw_steps);  // till all sub steps done
  TRACE(7);
#ifdef TINYSA4
//  if (old_CFGR != orig_CFGR) {    // Never happens ???
//    old_CFGR = orig_CFGR;
//    RCC->CFGR  = orig_CFGR;
 // }
#define IGNORE_RSSI 30000
//  pureRSSI_t rssi = (RSSI>0 ? RSSI + correct_RSSI + correct_RSSI_freq : IGNORE_RSSI); // add correction
  pureRSSI_t rssi = RSSI;
  if (false) {
//  abort:
    rssi = 0;
  }
  return rssi;
#else
  return RSSI + correct_RSSI + correct_RSSI_freq; // add correction
#endif

}

static uint8_t low_count = 0;
static uint8_t sweep_counter = 0;           // Only used for HW refresh
#ifdef __BANDS__
static int last_band = -1;

void reset_band(void) {
  last_band = -1;
}
#endif

// main loop for measurement
static bool sweep(bool break_on_operation)
{
  float RSSI;
//  float local_peakLevel = -150.0;
//  int local_peakIndex = 0;
#ifdef __SI4432__
  freq_t agc_peak_freq = 0;
  float agc_peak_rssi = -150;
  float agc_prev_rssi = -150;
  int last_AGC_value = 0;
  uint8_t last_AGC_direction_up = false;
  int AGC_flip_count = 0;
#endif
  //  if (setting.mode== -1)
  //    return;
  //  START_PROFILE;
#ifdef TINYSA3
  palClearPad(GPIOB, GPIOB_LED);
#endif
#ifdef TINYSA4
  palClearLine(LINE_LED);
#endif
  float temp_min_level = 100;

  //  spur_old_stepdelay = 0;
  //  shell_printf("\r\n");

  modulation_counter = 0;                                             // init modulation counter in case needed
  int refreshing = false;
#if 0
  if (MODE_OUTPUT(setting.mode) && config.cor_nfm == 0) {                          // Calibrate the modulation frequencies at first use
    calibrate_modulation(MO_AM, &config.cor_am);
#ifdef TINYSA3
    calibrate_modulation(MO_NFM, &config.cor_nfm);
#endif
#if 0
    calibrate_modulation(MO_NFM2, &config.cor_nfm2);
    calibrate_modulation(MO_NFM3, &config.cor_nfm3);
#endif
 //   calibrate_modulation(MO_WFM, &config.cor_wfm);
    config.cor_wfm = -14;
  }
#endif

  if (dirty) {                    // Calculate new scanning solution
#ifdef __BANDS__
    if (setting.multi_band && ! setting.multi_trace) {
      set_frequencies(0,0,sweep_points);
      update_rbw();
      current_band = getBand(0);
      if (setting.trigger != T_AUTO)
        setting.trigger_level = setting.bands[current_band].level;
    }
#endif
    sweep_counter = 0;
    if (get_sweep_frequency(ST_SPAN) < 300000)  // Check if AM signal
      check_for_AM = true;
    else {
      signal_is_AM = false;
      check_for_AM = false;
    }
  } else if ( MODE_INPUT(setting.mode) && setting.frequency_step > 0) {
    sweep_counter++;
#ifdef TINYSA3
    if (sweep_counter > 50 ) {     // refresh HW after 50 sweeps
      dirty = true;
      refreshing = true;
      sweep_counter = 0;
    }
#endif
  }

  bool show_bar = ( MODE_INPUT(setting.mode) ||  setting.frequency_step != 0 || setting.level_sweep != 0.0 ? true : false);

#if 0
#ifdef TINYSA4
  float vbw_factor  =  (float)setting.frequency_step / ((float) actual_rbw_x10*50.0);
  float vbw_rssi;
#endif
#endif
#ifdef __MARKER_CACHE__
  clear_marker_cache();
#endif
  uint16_t triggered = false;
  again:                          // Waiting for a trigger jumps back to here

#ifdef __BANDS__
  if (setting.multi_band && setting.multi_trace) {
    do {
      current_band++;
      if (current_band > BANDS_MAX)
        current_band = 0;
    }
    while(!(setting.bands[current_band].enabled));
    if (setting.bands[current_band].end > setting.bands[current_band].start && last_band != current_band) {
      last_band = current_band;
      set_sweep_frequency(ST_START, setting.bands[current_band].start);
      set_sweep_frequency(ST_STOP, setting.bands[current_band].end);
//      set_step_delay(SD_FAST);
//      set_rbw(8000);
//      set_sweep_points((setting.bands[current_band].end - setting.bands[current_band].start) / 800000);
      if (setting.trigger != T_AUTO)
        setting.trigger_level = setting.bands[current_band].level;
      setting.auto_attenuation = false;
    }
  } else {
    last_band = -1;
  }
#endif
#ifdef __BEEP__
  if (setting.trigger_beep) {
    set_audio_mode(A_PWM);
    pwm_stop();
  }
  else
    set_audio_mode(A_DAC);
#endif
  setting.measure_sweep_time_us = 0;                   // start measure sweep time
  //  start_of_sweep_timestamp = chVTGetSystemTimeX();    // Will be set in perform

  sweep_again:                                // stay in sweep loop when output mode and modulation on.

  if (setting.trigger_grid) {               // for start of sweep in time grid // return (systime_t)STM32_ST_TIM->CNT; does CH_CFG_ST_FREQUENCY ticks per second
    while ((chVTGetSystemTimeX() % (setting.trigger_grid+2)) == 0);
    while ((chVTGetSystemTimeX() % (setting.trigger_grid+2)) != 0);
  }

#ifdef __ULTRA__
  if (config.ultra_start == ULTRA_AUTO) {
    freq_t u_base = (hw_if ? 900000000 : 800000000);

    if (setting.mode == M_LOW){
      if (getFrequency(sweep_points-1) <= u_base)
        ultra_start = u_base;
      else
        ultra_start = u_base - 100000000;
    } else if (setting.mode == M_GENLOW) {
      if (setting.mixer_output)
        ultra_start = MAX_LOW_OUTPUT_FREQ;
      else
        ultra_start = config.ultra_start;
    }
  }
#endif
  // ------------------------- start sweep loop -----------------------------------
  for (int i = 0; i < sweep_points ; i++) {
    debug_avoid_second = false;
    debug_avoid_label:
    debug_avoid_second = debug_avoid_second;
    freq_t current_freq = getFrequency(i);
#ifdef __BANDS__
    if (setting.multi_band && !setting.multi_trace) {
      int new_band = getBand(i);
      if (current_band != new_band) {
        current_band = new_band;
        if (setting.trigger != T_AUTO)
        setting.trigger_level = setting.bands[current_band].level;
      }
    }
#endif
    // --------------------- measure -------------------------
    pureRSSI_t rssi = perform(break_on_operation, i, current_freq, setting.tracking);   // Measure RSSI for one of the frequencies
#ifdef TINYSA4
    if (rssi == IGNORE_RSSI)
      RSSI = -174.0;
    else
#endif

      RSSI = PURE_TO_float(rssi);
    // if break back to top level to handle ui operation
    if (refreshing)
      scandirty = false;
    if ((break_on_operation && operation_requested )
#ifdef __SWEEP_RESTART__
        || (MODE_OUTPUT(setting.mode) && !setting.sweep && (setting.level_sweep != 0 || get_sweep_frequency(ST_SPAN) != 0))
#endif
            ) {                        // break loop if needed
      abort:
      if (
#ifdef TINYSA4
          progress_bar &&
#endif
          setting.actual_sweep_time_us > ONE_SECOND_TIME /* && MODE_INPUT(setting.mode) */) {
        ili9341_set_background(LCD_BG_COLOR);
        ili9341_fill(OFFSETX, CHART_BOTTOM+1, WIDTH, 1);                    // Erase progress bar
#ifdef __SWEEP_RESTART__
        refresh_sweep_menu(-1);
#endif
      }
      return false;
    }
#ifdef __SWEEP_OUTPUT__
    dacPutChannelX(&DACD2, 0, (((float)i)*config.sweep_voltage)*4.279);        // Output sweep voltage  4095 -> 3.3 Volt
#endif

    // ----------------------- in loop AGC ---------------------------------

#ifdef __SI4432__
    if (!in_selftest && setting.mode == M_HIGH && S_IS_AUTO(setting.agc) && UNIT_IS_LOG(setting.unit)) {
#define AGC_RSSI_THRESHOLD  (-55+get_attenuation())
      float local_rssi = RSSI +setting.external_gain;
      if (local_rssi > AGC_RSSI_THRESHOLD && local_rssi > agc_prev_rssi) {
        agc_peak_freq = current_freq;
        agc_peak_rssi = agc_prev_rssi = local_rssi;
      }
      if (local_rssi < AGC_RSSI_THRESHOLD)
        agc_prev_rssi = -150;
      freq_t delta_freq = current_freq - agc_peak_freq;
      if (agc_peak_freq != 0 &&  delta_freq < 2000000) {
        int max_gain = (-25 - agc_peak_rssi ) / 4;
        auto_set_AGC_LNA(false, 16 + delta_freq * max_gain / 2000000 );    // enable LNA   and stepwise gain
      }
      else
        auto_set_AGC_LNA(TRUE, 0);
    }
#endif

    // Delay between points if needed, (all delays can apply in SI4432 fill)
    if (setting.measure_sweep_time_us == 0){                                    // If not already in buffer
      if (setting.additional_step_delay_us && (MODE_INPUT(setting.mode) || setting.modulation == MO_NONE)) {     // No delay when modulation is active
        if (setting.additional_step_delay_us < 30*ONE_MS_TIME)                                                   // Maximum delay time using my_microsecond_delay
          my_microsecond_delay(setting.additional_step_delay_us);
        else {
          int tm = setting.additional_step_delay_us / ONE_MS_TIME;
          do {
            osalThreadSleepMilliseconds(tm>100?100:tm);
            if (break_on_operation && operation_requested)
              goto abort;
            tm -= 100;
          } while (tm > 0);
        }
      }
    }
    systime_t local_sweep_time = sa_ST2US(chVTGetSystemTimeX() - start_of_sweep_timestamp);
    if (setting.actual_sweep_time_us > ONE_SECOND_TIME)
      local_sweep_time = setting.actual_sweep_time_us;
    if (
#ifdef TINYSA4
          progress_bar &&
#endif
          show_bar && (( local_sweep_time > ONE_SECOND_TIME && (i & 0x07) == 0) /* || ( local_sweep_time > ONE_SECOND_TIME*10)*/ ) )
    {
      int pos = i * (WIDTH+1) / sweep_points;
      ili9341_set_background(LCD_SWEEP_LINE_COLOR);
      ili9341_fill(OFFSETX, CHART_BOTTOM+1, pos, 1);     // update sweep progress bar
      ili9341_set_background(LCD_BG_COLOR);
      ili9341_fill(OFFSETX+pos, CHART_BOTTOM+1, WIDTH-pos, 1);

      if (local_sweep_time > 10 * ONE_SECOND_TIME) {
        plot_into_index(measured);
        redraw_request |= REDRAW_CELLS | REDRAW_BATTERY | REDRAW_INBETWEEN;
        // plot trace and other indications as raster
        draw_all(true);  // flush markmap only if scan completed to prevent
      }


#ifdef __SWEEP_RESTART__
      if (MODE_OUTPUT(setting.mode) && (setting.level_sweep != 0 || get_sweep_frequency(ST_SPAN) != 0))
        refresh_sweep_menu(i);
#endif
    }
    // -----------------------  debug avoid --------------------------------
    if (debug_avoid) {
      if (!debug_avoid_second) {
        temp_t[i] = RSSI;
        debug_avoid_second = true;
        goto debug_avoid_label;
      } else {
        debug_avoid_second = false;
      }
    }
#ifdef __DOUBLE_LOOP__
  }

  // -------------------------------- Scan finished, do all postprocessing --------------------

  if (MODE_INPUT(setting.mode)) {

// #ifdef __VBW__
#if 0
#ifdef __FFT_VBW__
    if (setting.vbw_x100 != 0 && sweep_points == 256) {
      float m = 150;
      for (int i=0;i<sweep_points;i++) {
        if (m > temp_t[i])
          m = temp_t[i];
        real[i] = 0;
        imag[i] = 0;
        actual_t[i] = -150;
      }
      for (int i=0;i<sweep_points;i++) {
        real[i] = temp_t[i] - m;
      }
      FFT(real, imag, 256, false);
#if 1
      for (int i = 128 - setting.vbw_x100; i<128+setting.vbw_x100; i++) {
        real[i] = 0;
        imag[i] = 0;
      }
#endif
      FFT(real, imag, 256, true);

      for (int i=0;i<sweep_points;i++) {
        float re = real[i];
        temp_t[i] = re + m;
//        actual_t[i] = sqrtf(re*re + im*im) + m;
      }
    }
#else

  // ------------------------ do VBW processing ------------------------------
    if (setting.frequency_step) {
      int vbw_count_div2 = actual_rbw_x10 * 100 / setting.frequency_step / (setting.vbw_x100 == 0 ? 10 : setting.vbw_x100);
      while(vbw_count_div2-- > 0){
        pureRSSI_t prev = temp_t[0];
        int j;
        // first point smooth
        temp_t[0] = (prev + prev + temp_t[1])/3.0f;
        for (j=1;j<sweep_points-1;j++){
          pureRSSI_t old = temp_t[j]; // save current data point for next point smooth
          temp_t[j] = (prev + temp_t[j] + temp_t[j] + temp_t[j+1])/4;
          prev = old;
        }
        // last point smooth
        temp_t[j] = (temp_t[j] + temp_t[j] + prev)/3;
      }
    }
#endif
#endif

#ifdef  __FFT_DECONV__
    int d_width = 0;
    float d_scale = 0.0;
    float d_offset = 0.0;
    int d_start = 0;
    if (setting.average == AV_DECONV && setting.frequency_step != 0) {
      d_width = (sweep_points * (actual_rbw_x10 * 250) / get_sweep_frequency(ST_SPAN));
      d_start = sweep_points/2 - d_width/2;
      d_offset = stored_t[d_start];
      for (int i=0; i<d_width; i++)
        if (d_offset > stored_t[d_start + i])
          d_offset = stored_t[d_start + i];
//      d_offset -= 1;    // To avoid divide by zero
      for (int i=0; i<d_width; i++)
        d_scale += stored_t[d_start + i] - d_offset;
//      d_scale *= d_wid;
    }
#endif

#ifdef __FFT_DECONV__
    if (setting.average == AV_DECONV && setting.frequency_step != 0 && sweep_points == 256) {
      float m = 150;
      for (int i=0;i<sweep_points;i++) {
        if (m > temp_t[i])
          m = temp_t[i];
        real[i] = 0.000000000001;
        imag[i] = 0;
        real2[0] = 0.000000000001;
        imag2[i] = 0;
        actual_t[i] = -150;
      }
      for (int i=0;i<sweep_points;i++) {
        if (temp_t[i] > m+25)
          real[i] = temp_t[i] - m;
      }
      FFT(real, imag, 256, false);
#if 1
#if 0
      for (int i = 128 - d_width*2; i<128+d_width*2; i++) {
        real[i] = 0;
        imag[i] = 0;
      }
#endif
#if 1
      for (int i=0;i<d_width/2;i++) {
        real2[i] = (stored_t[i+d_start + d_width/2] - d_offset) / d_scale*4;
      }
      for (int i=-d_width/2;i<0;i++) {
        real2[i+256] = (stored_t[i+d_start + d_width/2] - d_offset) / d_scale*4;
      }
#else
#if 0
      for (int i=0;i<d_width;i++) {
        real2[i] = (stored_t[i+d_start] - d_offset) / d_scale;
      }
//      for (int i=-d_width/2;i<0;i++) {
//        real2[i+256] = (stored_t[i+d_start + d_width/2] - d_offset) / d_scale;
//      }
#else
      real2[0] = 1;
//      real2[1] = 1;
//      real2[255] = 1;
//      real2[255] = -0.5;
#endif
#endif
      FFT(real2, imag2, 256, false);

      for (int i=0;i<256;i++) {
        float a = real[i];
        float b = imag[i];
        float c = real2[i];
        float d = imag2[i];
        float cd2 = c*c+d*d;
static volatile int dummy;
        if (cd2 == 0)
          cd2 = 1e-24;
//          while(dummy++) ;
        real[i] = (a*c+b*d)/cd2;
        imag[i] = (b*c-a*d)/cd2;
      }
#endif
      FFT(real, imag, 256, true);

      for (int i=0;i<sweep_points;i++) {
        float re = real[i];
        float im = imag[i];
        actual_t[i] = re + m;
//        actual_t[i] = sqrtf(re*re + im*im) + m;
      }
    }
#endif

    for (int i = 0; i < sweep_points; i++) {

#if 0
      // -------------------------- smoothing -----------------------------------------
#ifdef TINYSA4
      if (vbw_factor < 1) {
        if (i == 0) {
          RSSI = /* vbw_factor * */ RSSI;
        } else {
          RSSI = vbw_factor * RSSI + (1-vbw_factor)* vbw_rssi;
        }
        vbw_rssi = RSSI;
      }
#endif
#endif

      #ifdef __FFT_DECONV__
      if (setting.average == AV_DECONV)
        RSSI = actual_t[i];
      else
#endif
        RSSI = temp_t[i];

#else
#ifdef TINYSA4
      if (MODE_OUTPUT(setting.mode)) {
        if (level_error) {
          drawMessageBox("WARNING", "Output level reduced", 2000);
          level_error = false;
          ui_mode_menu();   // Refresh menu
        }
      }
#endif

      if (MODE_INPUT(setting.mode)) {
        if (
#ifdef __TRIGGER_TRACE__
            (setting.trigger_trace == 255 && RSSI >= setting.trigger_level) ||
            ( setting.trigger_trace != 255 && RSSI >= measured[setting.trigger_trace][i])
#else
            (RSSI >= setting.trigger_level)
#endif
            )
        {
          triggered = true;
#ifdef __BEEP__
          if (setting.trigger_beep) pwm_start(4000);
#endif
        }

      for (int t=0; t<TRACES_MAX;t++) {                 // Calculate all traces
        if (setting.stored[t])
          continue;
#ifdef __ULTRA__
        if (debug_spur && t >0)
          continue;
#endif
        float RSSI_calc = RSSI;
        float *trace_data = measured[t];
#endif // __DOUBLE_LOOP__
      // ------------------------ do all RSSI calculations from CALC menu -------------------
        if (setting.normalized[t])
          RSSI_calc -= measured[TRACE_TEMP][i];
      if (setting.subtract[t]) {
        RSSI_calc = RSSI_calc - measured[setting.subtract[t]-1][i] + setting.normalize_level;
      }
#ifdef __SI4432__
      //#define __DEBUG_AGC__
#ifdef __DEBUG_AGC__                 // For debugging the AGC control
      stored_t[i] = (SI4432_Read_Byte(0x69) & 0x01f) * 3.0 - 90.0; // Display the AGC value in the stored trace
#endif
      if (check_for_AM) {
        int AGC_value = (SI4432_Read_Byte(0x69) & 0x01f) * 3 - 90;
        if (AGC_value < last_AGC_value &&  last_AGC_direction_up ) {
          AGC_flip_count++;
        } else if (AGC_value > last_AGC_value &&  !last_AGC_direction_up ) {
          AGC_flip_count++;
        }
        last_AGC_value = AGC_value;
      }
#endif
      if (scandirty || setting.average[t] == AV_OFF) {             // Level calculations
        if (setting.average[t] == AV_MAX_DECAY) age[i] = 0;
        trace_data[i] = RSSI_calc;
      } else {
        switch(setting.average[t] ) {
        case AV_MIN:      if (trace_data[i] > RSSI_calc) trace_data[i] = RSSI_calc; break;
        case AV_MAX_HOLD: if (trace_data[i] < RSSI_calc) trace_data[i] = RSSI_calc; break;
        case AV_MAX_DECAY:
          if (trace_data[i] < RSSI_calc) {
            age[i] = 0;
            trace_data[i] = RSSI_calc;
          } else {
            if (age[i] > setting.decay)
              trace_data[i] -= 0.5;
            else
              age[i] += 1;
          }
          break;
        case AV_4:  trace_data[i] = (trace_data[i]*3.0 + RSSI_calc) / 4.0; break;
        case AV_16: trace_data[i] = (trace_data[i]*15.0 + RSSI_calc) / 16.0; break;
        case AV_100:
#ifdef TINYSA4
          if (linear_averaging)
        {
#if 0
          int old_unit = setting.unit;
          setting.unit = U_WATT;            // Power averaging should always be done in Watts
          trace_data[i] = to_dBm((value(trace_data[i])*(setting.scan_after_dirty[t]-1) + value(RSSI_calc)) / setting.scan_after_dirty[t] );
          setting.unit = old_unit;
#else
          float v = (expf(trace_data[i]*(logf(10.0)/10.0)) * (setting.scan_after_dirty[t]-1) + expf(RSSI_calc * (logf(10.0)/10.0))) / setting.scan_after_dirty[t];
          trace_data[i] = logf(v)*(10.0/logf(10.0));
#endif
        }
          else
            trace_data[i] = (trace_data[i]*(setting.scan_after_dirty[t]-1) + RSSI_calc)/ setting.scan_after_dirty[t];
#else
        trace_data[i] = (trace_data[i]*(setting.scan_after_dirty[t]-1) + RSSI_calc)/ setting.scan_after_dirty[t];
#endif
        break;
#ifdef __QUASI_PEAK__
        case AV_QUASI:
        { static float old_RSSI = -150.0;
        if (i == 0) old_RSSI = trace_data[sweep_points-1];
        if (RSSI_calc > old_RSSI && setting.attack > 1)
          old_RSSI += (RSSI_calc - old_RSSI)/setting.attack;
        else if (RSSI_calc < old_RSSI && setting.decay > 1)
          old_RSSI += (RSSI_calc - old_RSSI)/setting.decay;
        else
          old_RSSI = RSSI_calc;
        trace_data[i] = old_RSSI;
        }
        break;
#endif
#if 0
        case AV_DECONV:
          trace_data[i] = temp_t[i] - temp_t[0];

          int lower = ( i - d_width + 1 < 0 ? 0 :  i - d_width + 1);
          for (int k = lower; k < i; k++)
            trace_data[i] -= trace_data[k] * (stored_t[d_start + i - k] - d_offset) / d_scale;
//          trace_data[i] /= (stored_t[d_start] - d_offset ) /d_scale;
          break;
#endif
        }
      }
//      if ( actual_t[i] > -174.0 && temp_min_level > actual_t[i])   // Remember minimum
//        temp_min_level = actual_t[i];

      // --------------------------- find peak and add to peak table if found  ------------------------


//      add_to_peak_finding(actual_t, i);

      }
      }
    }


  // ---------------------- end of postprocessing -----------------------------

  if (MODE_OUTPUT(setting.mode) && setting.modulation != MO_NONE) { // if in output mode with modulation
    if (!in_selftest)
      goto sweep_again;                                             // Keep repeating sweep loop till user aborts by input
  }
  // --------------- check if maximum is above trigger level -----------------
  if (setting.trigger != T_AUTO && setting.frequency_step > 0) {    // Trigger active
//    if (measured[peakTrace][peakIndex] < setting.trigger_level) {
    if (!triggered) {
      goto again;                                                   // not yet, sweep again
    } else {
      if (setting.trigger == T_SINGLE) {
        set_trigger(T_DONE);
      }
    }
//    scandirty = true;                // To show trigger happened
  }
  if (setting.actual_sweep_time_us > ONE_SECOND_TIME /* && MODE_INPUT(setting.mode) */) {
    // ili9341_fill(OFFSETX, CHART_BOTTOM+1, WIDTH, 1, 0);     // Erase progress bar before updating actual_sweep_time
    ili9341_set_background(LCD_BG_COLOR);
    ili9341_fill(OFFSETX, CHART_BOTTOM+1, WIDTH, 1);
#ifdef __SWEEP_RESTART__
    refresh_sweep_menu(sweep_points-1);
#endif
  }
  // ---------------------- process measured actual sweep time -----------------
  // For CW mode value calculated in SI4432_Fill
  if (setting.measure_sweep_time_us == 0)
    setting.measure_sweep_time_us = sa_ST2US(chVTGetSystemTimeX() - start_of_sweep_timestamp);

  // Update actual time on change on status panel
  uint32_t delta = abs((int)(setting.actual_sweep_time_us - setting.measure_sweep_time_us));
  if ((delta<<3) > setting.actual_sweep_time_us){ // update if delta > 1/8
    redraw_request|=REDRAW_CAL_STATUS | REDRAW_FREQUENCY;
  }
  setting.actual_sweep_time_us = setting.measure_sweep_time_us;
  // Not possible reduce sweep time, it minimum!
  if (setting.sweep_time_us < setting.actual_sweep_time_us && setting.additional_step_delay_us == 0){
// Warning!! not correct set sweep time here, you get error!!
// value update to real and after + recalculated additional delay
//    setting.sweep_time_us = setting.actual_sweep_time_us;
//    redraw_request |= REDRAW_CAL_STATUS;
//    if (FREQ_IS_CW())                                  // if zero span mode
//      update_grid();
  }
  else{
    uint32_t dt = 0;
    static uint32_t last_dt = 0;
    // selected time less then actual, need reduce delay
    if (setting.sweep_time_us < setting.actual_sweep_time_us){
      dt = (setting.actual_sweep_time_us - setting.sweep_time_us)/(sweep_points);
      if (setting.additional_step_delay_us > dt)
        setting.additional_step_delay_us-=dt;
      else
        setting.additional_step_delay_us = 0;
    }// selected time greater then actual, need increase delay
    else if (setting.sweep_time_us > setting.actual_sweep_time_us){
      dt = (setting.sweep_time_us - setting.actual_sweep_time_us)/(sweep_points);
      setting.additional_step_delay_us+=dt;
    }
    // Update info on correction on next step, after apply . Always show when changed
    if (last_dt /* && dt == 0 */){
      redraw_request|=REDRAW_CAL_STATUS;
      if (FREQ_IS_CW())                // if zero span mode
        update_grid();                 // and update grid and frequency
    }
    last_dt = dt;
  }

  // ---------------------- sweep finished,  do all postprocessing ---------------------

  if (scandirty) {
    scandirty = false;
    redraw_request |= REDRAW_CAL_STATUS;
  }

  if (MODE_OUTPUT(setting.mode) && (sweep_mode & SWEEP_ENABLE) )                            // Sweep time is calculated, we can sweep again in output mode
    goto again;                                             // Keep repeating sweep loop till user aborts by input


  // --------------------- set tracking markers from maximum table and find max/min -----------------

  peakLevel = -170;
  temp_min_level = 100;
  for (int t=(in_selftest? 0 : TRACES_MAX-1); t >= 0; t--) {        // During selftest only trace 0 is used.
    if (IS_TRACE_ENABLE(t) && !(ignore_stored && (t == TRACE_STORED))) {  // Selftest 0 1 switches off in_selftest so ignore stored trace
      float *trace_data = measured[t];
      for (int i=0;i<sweep_points; i++) {
        add_to_peak_finding(trace_data, i);
        if ( trace_data[i] > -174.0 && temp_min_level > trace_data[i])   // Remember minimum
          temp_min_level = trace_data[i];
      }
      if (cur_max == 0) {                           // Always at least one maximum per trace
        max_index[0] = peak_finding_index;
        cur_max = 1;
      } else
        peak_finding_index = max_index[0];

      if (peak_finding_index > 0 && trace_data[peak_finding_index] > peakLevel) {
        peakIndex = peak_finding_index;
        peakLevel = trace_data[peakIndex];
        peakFreq = getFrequency(peakIndex);
        peakTrace = t;
      }

      if (MODE_INPUT(setting.mode)) {               // Assign maxima found to tracking markers
        int i = 0;
        int m = 0;
        while (i < cur_max) {                                 // For all maxima found
          while (m < MARKERS_MAX) {
            if (markers[m].enabled && markers[m].mtype & M_TRACKING && markers[m].trace == t) {   // Available marker found
              markers[m].index = max_index[i];
              interpolate_maximum(m);
              m++;
              break;                          // Next maximum
            }
            m++;                              // Try next marker
          }
          i++;
        }
        while (m < MARKERS_MAX) {                  // Insufficient maxima found
          if (markers[m].enabled && markers[m].mtype & M_TRACKING  && markers[m].trace == t) {    // More available markers found
            set_marker_index(m, 0); // Enabled but no max so set to left most frequency
          }
          m++;                              // Try next marker
        }
      }
    }
  }


#define __MIRROR_MASKING__
#ifdef __MIRROR_MASKING__
#ifdef __SI4432__
  if (setting.mode == M_HIGH && setting.mirror_masking) {
    int mirror_offset = 2 * 937000 / setting.frequency_step;
//    int mask_start = 0;
//    int mask_end = 0;
    if (mirror_offset > 3) {
      for (int i = 1; i < sweep_points - mirror_offset; i++) {
        int m = i+mirror_offset;
        if (actual_t[i] > -80 && actual_t[m] < actual_t[i] - 25 && ( actual_t[m] > actual_t[m-1] || actual_t[m+1] > actual_t[m-1] ) /* && (i < mask_start || mask_start == 0) */ ) {
//          if (mask_start == 0)
//            mask_start = m;
          actual_t[m] = actual_t[m-1];
          actual_t[m+1] = actual_t[m-1];
        }
//        else {
//          if (i == mask_start)
//            i += mirror_offset;
//          mask_start =0;
//        }
      }
    }
  }

#endif
#endif
  // -------------------------- auto attenuate ----------------------------------
#ifdef TINYSA4
#define AUTO_TARGET_LEVEL   (actual_rbw_x10  >= 10 ? -35 : -40)
#define LNA_AUTO_TARGET_LEVEL   -45
#else
#define AUTO_TARGET_LEVEL   -25
#endif
#define AUTO_TARGET_WINDOW  2

  if (!in_selftest && setting.mode == M_LOW && setting.auto_attenuation
#ifdef TINYSA4_4
      && !setting.extra_lna
#endif
  ) {  // calculate and apply auto attenuate
    setting.atten_step = false;     // No step attenuate in low mode auto attenuate
    int changed = false;
    int delta = 0;
    int target_level = AUTO_TARGET_LEVEL;
#ifdef TINYSA4
    freq_t min_target_freq = get_sweep_frequency(ST_START);
    if (min_target_freq > 30000000) // 30M and lower has zero correction
      target_level += PURE_TO_float(get_frequency_correction(min_target_freq));
    if (setting.extra_lna)
      target_level = LNA_AUTO_TARGET_LEVEL;
#endif
    int actual_max_level = (peakIndex == 0 ? -100 :(int) (measured[peakTrace][peakIndex] - get_attenuation()) ) + setting.external_gain; // If no max found reduce attenuation
    if (actual_max_level < target_level && setting.attenuate_x2 > 0) {
      delta = - (target_level - actual_max_level);
    } else if (actual_max_level > target_level && setting.attenuate_x2 < 60) {
      delta = actual_max_level - target_level;
    }
    if (chVTGetSystemTimeX() - sweep_elapsed > MS2ST(1000)){
      if (( delta < -5 || delta > +5) || delta > 10 ) {
      setting.attenuate_x2 += delta + delta;
      if (setting.attenuate_x2 < 0)
        setting.attenuate_x2 = 0;
      if (setting.attenuate_x2 > 60)
        setting.attenuate_x2 = 60;
      changed = true;
      sweep_elapsed = chVTGetSystemTimeX();
      }
    }

    // Try update settings
    if (changed){
#ifdef __PE4302__
      PE4302_Write_Byte((int) get_attenuation() * 2);
#endif
      redraw_request |= REDRAW_CAL_STATUS;
#ifdef __SI4432__
      SI4432_Sel = SI4432_RX ;
#if 0                               // this should never happen
      if (setting.atten_step) {
        set_switch_transmit();          // This should never happen
      } else {
        set_switch_receive();
      }
#endif
#endif
      calculate_static_correction();            // Update correction
      //      dirty = true;                               // Needed to recalculate the correction factor
    }
  }

  // ----------------------------------  auto AGC ----------------------------------


#ifdef __SI4432__
  if (!in_selftest && MODE_INPUT(setting.mode)) {
    if (S_IS_AUTO(setting.agc)) {
      int actual_max_level = measured[peakTrace][peakIndex] - get_attenuation() + setting.external_gain;        // No need to use float
      if (UNIT_IS_LINEAR(setting.unit)) { // Auto AGC in linear mode
        if (actual_max_level > - 45)
          auto_set_AGC_LNA(false, 0); // Strong signal, no AGC and no LNA
        else
          auto_set_AGC_LNA(TRUE, 0);
      }
      if (check_for_AM) {
        if (signal_is_AM) {
          if (actual_max_level < - 40 )
            signal_is_AM = false;
        } else {
          if (AGC_flip_count > 20 && actual_max_level >= - 40)
            signal_is_AM = true;
        }
        if (signal_is_AM) {      // if log mode and AM signal
          auto_set_AGC_LNA(false, 16); // LNA on and no AGC
        } else {
          auto_set_AGC_LNA(TRUE, 0);
        }
      }
    } else
      signal_is_AM = false;
  }
#else
  signal_is_AM = false;
#endif



  // -------------------------- auto reflevel ---------------------------------
  //    if (max_index[0] > 0)
  //      temppeakLevel = actual_t[max_index[0]];

  if (!in_selftest && MODE_INPUT(setting.mode) && setting.auto_reflevel) {  // Auto reflevel

    float r = value(peakLevel);
    float s_max = r / setting.scale;                  // Peak level normalized to /div

    if (UNIT_IS_LINEAR(setting.unit)) {            // Linear scales can not have negative values
      if (setting.reflevel > REFLEVEL_MIN)  {
        if (s_max <  2)
          low_count = 5;
        else if (s_max < 4)
          low_count++;
        else
          low_count = 0;
      }
      if ((low_count > 4) || (setting.reflevel < REFLEVEL_MAX && s_max > NGRIDY) ) { // ensure minimum and maximum reflevel
        if (r < REFLEVEL_MIN)
          r = REFLEVEL_MIN;
        if (r > REFLEVEL_MAX)
          r = REFLEVEL_MAX;
        if (r != setting.reflevel) {
          //if (setting.scale * NGRIDY > r)
          set_scale(r / NGRIDY);
          set_reflevel(setting.scale*NGRIDY);
          //         dirty = false;                        // Prevent reset of SI4432
        }
      }
    } else {
#define MAX_FIT (NGRIDY-1.2)
      float s_min = value(temp_min_level)/setting.scale;
#ifdef TINYSA4
      float noise = (noise_level - setting.external_gain - (setting.extra_lna ? 20 : 0))/setting.scale;
      if (s_min < noise)
        s_min = noise;
#endif
      float s_ref = setting.reflevel/setting.scale;
      if (s_max < s_ref  - NGRIDY || s_min > s_ref  || s_max > s_ref  + 2.0) { //Completely outside or way too low
        if (s_max - s_min < NGRIDY - 2)
          set_reflevel(setting.scale*(floorf(s_min+8.8+ 1)));
        else
          set_reflevel(setting.scale*(floorf(s_max)+1));
        //        dirty = true;                               // Must be  above if(scandirty!!!!!)
      } else if (s_max > s_ref  - 0.5 || s_min > s_ref - 8.8 ) { // maximum to high or minimum to high
        set_reflevel(setting.reflevel + setting.scale);
        //        dirty = true;                               // Must be  above if(scandirty!!!!!)
      } else if (s_min < s_ref - 10.1 && s_max < s_ref -  1.5) { // minimum too low and maximum can move up
        set_reflevel(setting.reflevel - setting.scale);
        //        dirty = true;                               // Must be  above if(scandirty!!!!!)
      }
      //     dirty = false;                        // Prevent reset of SI4432
    }
  }



  // ----------------------- now follow all the special marker calculations for the measurement modes ----------------------------


#ifdef __MEASURE__
  if (setting.measurement == M_IMD && markers[0].index > 10) {                    // ----- IMD measurement
#ifdef TINYSA4
#define H_SPACING   7
#else
#define H_SPACING   4
#endif
    for (int i=1; i < MARKER_COUNT;i++)
      markers[i].enabled = search_maximum(i, getFrequency(markers[0].index)*(i+1), (i+1)*H_SPACING);
#ifdef TINYSA4
  } else if (setting.measurement == M_AM  && markers[0].index > 10) { // ----------AM measurement
    int l = markers[1].index;
    int r = markers[2].index;
    if (r < l) {
      l = markers[2].index;
      r = markers[1].index;
      markers[1].index = l;
      markers[2].index = r;
    }
    freq_t lf = getFrequency(l);
    freq_t rf = getFrequency(r);
    markers[1].frequency = lf;
    markers[2].frequency = rf;
#endif
  } else if (setting.measurement == M_OIP3  && markers[0].index > 10 && markers[1].index > 10) { // ----------IOP measurement
    int l = markers[0].index;
    int r = markers[1].index;
    if (r < l) {
      l = markers[1].index;
      r = markers[0].index;
    }
    set_marker_index(0, l);
    set_marker_index(1, r);
    freq_t lf = markers[0].frequency;
    freq_t rf = markers[1].frequency;
#ifdef TINYSA4
#define OIP3_SPAN   40
#else
#define OIP3_SPAN   12
#endif
    markers[2].enabled = search_maximum(2, lf - (rf - lf), 10);
    markers[3].enabled = search_maximum(3, rf + (rf - lf), 10);
  } else if (setting.measurement == M_PHASE_NOISE  && markers[0].index > 10) {    //  ------------Phase noise measurement
    // Position phase noise marker at requested offset
    set_marker_index(1, markers[0].index + (setting.mode == M_LOW ? WIDTH/4 : -WIDTH/4));
  } else if ((setting.measurement == M_PASS_BAND || setting.measurement == M_FM)  && markers[0].index > 10) {      // ----------------Pass band measurement
    int t1;
    int t2;
    float v = -200;
    t1 = (setting._sweep_points>>2);
    t2 = (t1<<1) + t1;
    for (int i=t1; i<t2; i++) {
      if (v < actual_t[i])
        v = actual_t[i];
    }
    t1 = 0;
    t2 = 0;
    v = v - (in_selftest ? 6.0 : 3.0);
    while (t1 < markers[0].index && actual_t[t1+1] < v)                                        // Find left -3dB point
      t1++;
    if (t1< markers[0].index)
      set_marker_index(1, t1);
    t2 = setting._sweep_points-1;;
    while (t2 > markers[0].index && actual_t[t2-1] < v)                // find right -3dB point
      t2--;
    if (t2 > markers[0].index)
      set_marker_index(2, t2);
#if 1
    int t = (t1+t2)/2;
    t1 += (t-t1)/2;
    t2 -= (t2-t)/2;
    if (t2-t1 < 100 && t2-t1 > 10 ) {
      float aver = 0.0;
      for (int i=t1;i<=t2;i++)
        aver +=actual_t[i];
      aver /= (t2-t1+1);
      float stdev=0.0;
      for (int i=t1;i<=t2;i++)
        stdev +=(actual_t[i] - aver) * (actual_t[i] - aver);
      stdev /= (t2-t1+1);
      //      stdev = sqrtf(stdev);
      flatness = stdev;
    } else
      flatness = -1;
#endif

  } else if (setting.measurement == M_AM) {      // ----------------AM measurement
    if (S_IS_AUTO(setting.agc )) {
#ifdef __SI4432__
      int actual_level = measured[peakTrace][peakIndex] - get_attenuation() + setting.external_gain;  // no need for float
      if (actual_level > -20 ) {
        setting.agc = S_AUTO_OFF;
        setting.lna = S_AUTO_OFF;
      } else if (actual_level < -45 ) {
        setting.agc = S_AUTO_ON;
        setting.lna = S_AUTO_ON;
      } else {
        setting.agc = S_AUTO_OFF;
        setting.lna = S_AUTO_ON;
      }
      set_AGC_LNA();
#endif
    }
#ifdef __CHANNEL_POWER__
  } else if (setting.measurement == M_CP || setting.measurement == M_SNR || setting.measurement == M_NF_TINYSA|| setting.measurement == M_NF_VALIDATE|| setting.measurement == M_NF_AMPLIFIER) {      // ----------------CHANNEL_POWER measurement
    freq_t bw = get_sweep_frequency(ST_SPAN)/3;
    int old_unit = setting.unit;
    setting.unit = U_WATT;
    for (int c = 0; c < 3 ;c++) {
      channel_power_watt[c] = 0.0;
      int sp_div3 = sweep_points/3;
      for (int i =0; i < sp_div3; i++) {
        channel_power_watt[c] += value(actual_t[i + c*sp_div3]);
      }
      float rbw_cor =  ((float)bw) / ((float)actual_rbw_x10 * 100.0);
      channel_power_watt[c] = channel_power_watt[c] * rbw_cor /(float)sp_div3;
      channel_power[c] = to_dBm(channel_power_watt[c]);
    }
    setting.unit = old_unit;
#endif
  }

#endif

  //  } while (MODE_OUTPUT(setting.mode) && setting.modulation != MO_NONE);      // Never exit sweep loop while in output mode with modulation
#if 0       // Read ADC
  extern   int fix_fft(short fr[], short fi[], short m, short inverse);
  extern int16_t adc_buf_read(uint32_t chsel, uint16_t *result, uint32_t count);
  trace[TRACE_STORED].enabled = true;
  adc_buf_read(ADC_CHSELR_CHSEL4, spi_buffer, 290);
#if 1           // Perform FFT on input
  int32_t zero = 0;
  for (int i=0;i<256;i++) {
    zero += spi_buffer[i];
  }
  zero = zero >> 8;
  int16_t *rfft = (int16_t *)&spi_buffer[0];
  int16_t *ifft = (int16_t *)&spi_buffer[512];
  for (int i=0;i<256;i++) {
    rfft[i] = spi_buffer[i] - zero;
    ifft[i] = rfft[i];    // Imaginary part equal to real part
    rfft[511 - i] = rfft[i];    // Mirror real
    ifft[511 - i] = -rfft[i]; // Conjugate mirror for imaginary part
  }
  fix_fft(rfft,ifft, 9,false);
#endif
  for (int i=0;i<256;i++) {             // Concert to
#if 1                                   // Linear
    stored_t[i] = (((int16_t *)spi_buffer)[i]/44.0) - 80.0;
#else
    float r = rfft[i];                  // Log
    if (r < 0)
      r = -r;
    float im = ifft[i];
    if (im < 0)
      im = -im;
    if (r == 0)
      r = 1;
    if (im==0)
      im = 1;
    stored_t[i] = (log10(r) * 2.0 + log10(im) * 2.0)/2.0 - 80.0;
#endif
  }
#endif


#ifdef __LINEARITY__
  //---------------- in Linearity measurement the attenuation has to be adapted ------------------
  if (setting.measurement == M_LINEARITY && setting.linearity_step < sweep_points) {
    setting.attenuate_x2 = (29.0 - setting.linearity_step * 30.0 / (sweep_points))*2.0;
    dirty = true;
    setting.stored[TRACE_STORED]=true;
    stored_t[setting.linearity_step] = peakLevel;
    setting.linearity_step++;
  }
#endif
  //    redraw_marker(peak_marker, FALSE);
  //  STOP_PROFILE;
#ifdef TINYSA3
  palSetPad(GPIOB, GPIOB_LED);
#endif
#ifdef TINYSA4
//  palSetLine(LINE_LED);
#endif
  
  // Enable traces at sweep complete for redraw
  if (enable_after_complete){
    TRACE_ENABLE(enable_after_complete);
    enable_after_complete = 0;
  }
  return true;
}

//------------------------------- SEARCH ---------------------------------------------

int
marker_search_left_max(int m)
{
  int i;
  float *ref_marker_levels = measured[markers[m].trace];
  int from = markers[m].index;

  int found = -1;
  if (uistat.current_trace == TRACE_INVALID)
    return -1;

  float value = ref_marker_levels[from];
  for (i = from - 1; i >= 0; i--) {
    float new_value = ref_marker_levels[i];
    if (new_value < value) {
      value = new_value;
      found = i;
    } else if (new_value > value + setting.noise )
      break;
  }

  for (; i >= 0; i--) {
    float new_value = ref_marker_levels[i];
    if (new_value > value) {
      value = new_value;
      found = i;
    } else if (new_value < value  - setting.noise )
      break;
  }
  return found;
}

int
marker_search_right_max(int m)
{
  int i;
  float *ref_marker_levels = measured[markers[m].trace];
  int from = markers[m].index;

  int found = -1;

  if (uistat.current_trace == TRACE_INVALID)
    return -1;
  float value = ref_marker_levels[from];
  for (i = from + 1; i < sweep_points; i++) {
    float new_value = ref_marker_levels[i];
    if (new_value < value) {    // follow down
      value = new_value;
      found = i;
    } else if (new_value > value + setting.noise) // larger then lowest value + noise
      break;    //  past the minimum
  }
  for (; i < sweep_points; i++) {
    float new_value = ref_marker_levels[i];
    if (new_value > value) {    // follow up
      value = new_value;
      found = i;
    } else if (new_value < value - setting.noise)
      break;
  }
  return found;
}

void markers_reset()
{
  for (uint8_t i = 0; i< MARKERS_MAX; i++) {
    markers[i].enabled = M_DISABLED;
    markers[i].mtype = M_DELTA;
    markers[i].ref = 0;
    markers[i].trace = 0;
  }
  markers[0].mtype = M_TRACKING;
  markers[0].enabled = M_ENABLED;
  active_marker = 0;
}

int marker_search_max(int m)
{
  int i = 0;
  float *ref_marker_levels = measured[markers[m].trace];
  int found = 0;

  float value = ref_marker_levels[i];
  for (; i < sweep_points; i++) {
    int new_value = ref_marker_levels[i];
    if (new_value > value) {    // follow up
      value = new_value;
      found = i;
    }
  }
  return found;
}

#define MINMAX_DELTA_X10 100


int
marker_search_left_min(int m)
{
  int i;
  float *ref_marker_levels = measured[markers[m].trace];
  int from = markers[m].index;
  int found = from;
  if (uistat.current_trace == TRACE_INVALID)
    return -1;

  int value_x10 = ref_marker_levels[from]*10;
  for (i = from - 1; i >= 0; i--) {
    int new_value_x10 = ref_marker_levels[i]*10;
    if (new_value_x10 > value_x10) {
      value_x10 = new_value_x10;        // follow up
//      found = i;
    } else if (new_value_x10 < value_x10 - MINMAX_DELTA_X10 )
      break;  // past the maximum
  }

  for (; i >= 0; i--) {
    int new_value_x10 = ref_marker_levels[i]*10;
    if (new_value_x10 < value_x10) {
      value_x10 = new_value_x10;        // follow down
      found = i;
    } else if (new_value_x10 > value_x10  + MINMAX_DELTA_X10 )
      break;
  }
  return found;
}

int
marker_search_right_min(int m)
{
  int i;
  float *ref_marker_levels = measured[markers[m].trace];
  int from = markers[m].index;
  int found = from;

  if (uistat.current_trace == TRACE_INVALID)
    return -1;
  int value_x10 = ref_marker_levels[from]*10;
  for (i = from + 1; i < sweep_points; i++) {
    int new_value_x10 = ref_marker_levels[i]*10;
    if (new_value_x10 > value_x10) {    // follow up
      value_x10 = new_value_x10;
//      found = i;
    } else if (new_value_x10 < value_x10 - MINMAX_DELTA_X10) // less then largest value_x10 - noise
      break;    // past the maximum
  }
  for (; i < sweep_points; i++) {
    int new_value_x10 = ref_marker_levels[i]*10;
    if (new_value_x10 < value_x10) {    // follow down
      value_x10 = new_value_x10;
      found = i;
    } else if (new_value_x10 > value_x10 + MINMAX_DELTA_X10) // larger then smallest value_x10 + noise
      break;
  }
  return found;
}




// -------------------- Self testing -------------------------------------------------

enum {
  TC_SIGNAL, TC_BELOW, TC_ABOVE, TC_FLAT, TC_MEASURE, TC_SET, TC_END, TC_ATTEN, TC_DISPLAY, TC_LEVEL, TC_SWITCH, TC_JUMP
};

enum {
  TP_SILENT, TPH_SILENT, TP_10MHZ, TP_10MHZEXTRA, TP_30MHZ_SWITCH, TP_30MHZ, TPH_30MHZ, TPH_30MHZ_SWITCH,
#ifdef TINYSA4
  TP_30MHZ_ULTRA, TP_30MHZ_DIRECT, TP_30MHZ_LNA,TP_SILENT_LNA,  TP_15MHZ_LNA, TP_15MHZ_LNA2
#endif
};

#define TEST_COUNT  (sizeof test_case / sizeof test_case[0])

#define W2P(w) (sweep_points * w / 100)     // convert width in % to actual sweep points

#ifdef TINYSA4
freq_t test_freq = 0;
#define CAL_LEVEL -35.30
#else
#define CAL_LEVEL   (has_esd ? -26.2 : -25)
#endif

// TODO made more compact this structure (need use aligned data)
typedef struct test_case {
  uint8_t kind;
  uint8_t setup;
  int16_t width;
  float center;      // In MHz
  float span;        // In MHz
  float pass;
  float stop;
} test_case_t;

// Use this data parser for init structure data
#define TEST_CASE_STRUCT(Condition, Preparation, Center, Span, Pass, Width, Stop) {Condition, Preparation, Width, Center, Span, Pass, Stop}

const test_case_t test_case [] =
#ifdef TINYSA4
{//                 Condition   Preparation     Center  Span    Pass    Width(%)Stop
 TEST_CASE_STRUCT(TC_BELOW,     TP_SILENT,      0.06,  0.11,   -30,      0,      -30),         // 1 Zero Hz leakage
 TEST_CASE_STRUCT(TC_BELOW,     TP_SILENT,      0.1,   0.1,   -50,    0,      0),         // 2 Phase noise of zero Hz
 TEST_CASE_STRUCT(TC_SIGNAL,    TP_30MHZ,       30,     1,      CAL_LEVEL,   10,     -85),      // 3
 TEST_CASE_STRUCT(TC_SIGNAL,    TP_30MHZ_ULTRA, 30,    1,      CAL_LEVEL,    10,     -85),      // 4 Test Ultra mode
#define TEST_SILENCE 4
 TEST_CASE_STRUCT(TC_BELOW,     TP_SILENT,      200,    100,    -70,    0,      0),         // 5  Wide band noise floor low mode
 TEST_CASE_STRUCT(TC_ABOVE,     TP_30MHZ_DIRECT,900,   10,    -90,    0,      -90),         // 6 Direct path with harmonic
 TEST_CASE_STRUCT(TC_SIGNAL,    TP_10MHZEXTRA,  30,     14,      CAL_LEVEL,    26,     -45),      // 7 BPF loss and stop band
 TEST_CASE_STRUCT(TC_FLAT,      TP_10MHZEXTRA,  30,     14,      -28,    9,     -60),       // 8 BPF pass band flatness
 TEST_CASE_STRUCT(TC_BELOW,     TP_15MHZ_LNA2,    855,    1,     -80,    0,      -80),       // 9 LPF cutoff
 TEST_CASE_STRUCT(TC_SIGNAL,    TP_15MHZ_LNA,   30,     5,      CAL_LEVEL,   10,     -90),      // 10 Flatness
 TEST_CASE_STRUCT(TC_SIGNAL,    TP_30MHZ_SWITCH,30,     7,      CAL_LEVEL,    10,     -50),      // 11 Switch isolation using high attenuation
 TEST_CASE_STRUCT(TC_DISPLAY,     TP_30MHZ,       30,     0,      CAL_LEVEL,    50,     -60),      // 12 test display
 TEST_CASE_STRUCT(TC_ATTEN,     TP_30MHZ,       30,     0,      CAL_LEVEL,    50,     -60),      // 13 Measure atten step accuracy
 TEST_CASE_STRUCT(TC_SIGNAL,    TP_30MHZ_LNA,       30,     5,      CAL_LEVEL,   10,     -75),      // 14 Measure LNA
#define TEST_END 14
 TEST_CASE_STRUCT(TC_END,       0,              0,      0,      0,      0,      0),
#define TEST_POWER  15
 TEST_CASE_STRUCT(TC_MEASURE,   TP_30MHZ,       30,     50,      CAL_LEVEL,   10,     -55),      // 12 Measure power level and noise
 TEST_CASE_STRUCT(TC_MEASURE,   TP_30MHZ,       270,    4,      -50,    10,     -75),       // 13 Measure powerlevel and noise
 TEST_CASE_STRUCT(TC_MEASURE,   TPH_30MHZ,      270,    4,      -40,    10,     -65),       // 14 Calibrate power high mode
 TEST_CASE_STRUCT(TC_END,       0,              0,      0,      0,      0,      0),
#define TEST_RBW    19
 TEST_CASE_STRUCT(TC_MEASURE,   TP_30MHZ,       30,     1,      CAL_LEVEL,    10,     -60),      // 16 Measure RBW step time
 TEST_CASE_STRUCT(TC_END,       0,              0,      0,      0,      0,      0),
 TEST_CASE_STRUCT(TC_MEASURE,   TPH_30MHZ,      300,    4,      -48,    10,     -65),       // 14 Calibrate power high mode
 TEST_CASE_STRUCT(TC_MEASURE,   TPH_30MHZ_SWITCH,300,    4,      -40,    10,     -65),       // 14 Calibrate power high mode
#define TEST_ATTEN    23
 TEST_CASE_STRUCT(TC_ATTEN,      TP_30MHZ,       30,     0,      CAL_LEVEL,    50,     -60),      // 20 Measure atten step accuracy
#define TEST_SPUR    24
 TEST_CASE_STRUCT(TC_BELOW,      TP_SILENT,     144,     8,      -95,    0,     0),       // 22 Measure 48MHz spur
#define TEST_LEVEL  25
 TEST_CASE_STRUCT(TC_LEVEL,     TP_30MHZ,       30.000,     0,      CAL_LEVEL,   50,     -55),      // 23 Measure level
 TEST_CASE_STRUCT(TC_LEVEL,     TP_30MHZ_LNA,   30.000,     0,      CAL_LEVEL,   50,     -55),      // 23 Measure level
 TEST_CASE_STRUCT(TC_LEVEL,     TPH_30MHZ,      150,     0,      CAL_LEVEL-30,   50,     -55),      // 23 Measure level
#define TEST_NOISE  28
 TEST_CASE_STRUCT(TC_LEVEL,     TP_SILENT,       201.000,     0,      -166,   50,     -166),      // 23 Measure level
#define TEST_NOISE_RBW  29
 TEST_CASE_STRUCT(TC_MEASURE,   TP_SILENT,       201,     1,      -166,    10,     -166),      // 16 Measure RBW step time
#define TEST_JUMP 30
 TEST_CASE_STRUCT(TC_JUMP,      TP_30MHZ_LNA,   30,     0.001,      -40,    0,     CAL_LEVEL),      // 16 Measure jumps
#define TEST_JUMP_HARMONIC 31
 TEST_CASE_STRUCT(TC_JUMP,      TP_30MHZ,       30,     0.001,      -40,    0,     CAL_LEVEL),      // 16 Measure jumps
};
#else
{//               Condition     Preparation     Center  Span    Pass    Width(%)Stop
 TEST_CASE_STRUCT(TC_BELOW,     TP_SILENT,      0.005,  0.01,   0,      0,      0),         // 1 Zero Hz leakage
 TEST_CASE_STRUCT(TC_BELOW,     TP_SILENT,      0.015,   0.01,   -30,    0,      0),         // 2 Phase noise of zero Hz
 TEST_CASE_STRUCT(TC_SIGNAL,    TP_30MHZ,       30,     7,      -25,    10,     -90),      // 3
 TEST_CASE_STRUCT(TC_SIGNAL,    TP_10MHZ,       30,     7,      -34,    10,     -90),      // 4
#define TEST_SILENCE 4
 TEST_CASE_STRUCT(TC_BELOW,     TP_SILENT,      200,    100,    -75,    0,      0),         // 5  Wide band noise floor low mode
 TEST_CASE_STRUCT(TC_BELOW,     TPH_SILENT,     600,    720,    -75,    0,      0),         // 6 Wide band noise floor high mode
 TEST_CASE_STRUCT(TC_SIGNAL,    TP_10MHZEXTRA,  10,     7,      -20,    27,     -80),      // 7 BPF loss and stop band
 TEST_CASE_STRUCT(TC_FLAT,      TP_10MHZEXTRA,  10,     4,      -18,    9,     -60),       // 8 BPF pass band flatness
 TEST_CASE_STRUCT(TC_BELOW,     TP_30MHZ,       450,    80,     -75,    0,      -75),       // 9 LPF cutoff
 TEST_CASE_STRUCT(TC_SIGNAL,    TP_30MHZ_SWITCH, 30,     7,      -25,    10,     -60),      // 10 Switch isolation using high attenuation
 TEST_CASE_STRUCT(TC_DISPLAY,     TP_30MHZ,       30,     0,      -25,    145,     -60),      // 11 Test display
 TEST_CASE_STRUCT(TC_ATTEN,     TP_30MHZ,       30,     0,      -25,    145,     -60),      // 12 Measure atten step accuracy
#define TEST_END 12
 TEST_CASE_STRUCT(TC_END,       0,              0,      0,      0,      0,      0),
#define TEST_POWER  13
 TEST_CASE_STRUCT(TC_MEASURE,   TP_30MHZ,       30,     7,      -25,   10,     -55),      // 12 Measure power level and noise
 TEST_CASE_STRUCT(TC_MEASURE,   TP_30MHZ,       270,    4,      -50,    10,     -75),       // 13 Measure powerlevel and noise
 TEST_CASE_STRUCT(TC_MEASURE,   TPH_30MHZ,      270,    4,      -40,    10,     -65),       // 14 Calibrate power high mode
 TEST_CASE_STRUCT(TC_END,       0,              0,      0,      0,      0,      0),
#define TEST_RBW    17
 TEST_CASE_STRUCT(TC_MEASURE,   TP_30MHZ,       30,     1,      -20,    10,     -60),      // 16 Measure RBW step time
 TEST_CASE_STRUCT(TC_END,       0,              0,      0,      0,      0,      0),
 TEST_CASE_STRUCT(TC_MEASURE,   TPH_30MHZ,      300,    4,      -48,    10,     -65),       // 14 Calibrate power high mode
 TEST_CASE_STRUCT(TC_MEASURE,   TPH_30MHZ_SWITCH,300,    4,      -40,    10,     -65),       // 14 Calibrate power high mode
#define TEST_ATTEN    21
 TEST_CASE_STRUCT(TC_ATTEN,      TP_30MHZ,       30,     0,      -25,    145,     -60),      // 20 Measure atten step accuracy
#define TEST_SPUR    22
 TEST_CASE_STRUCT(TC_BELOW,      TP_SILENT,     96,     8,      -95,    0,     0),       // 22 Measure 48MHz spur
#define TEST_LEVEL  23
 TEST_CASE_STRUCT(TC_LEVEL,   TP_30MHZ,       30,     0,      -25,   145,     -55),      // 23 Measure level
};
#endif


enum {
  TS_WAITING, TS_PASS, TS_FAIL, TS_CRITICAL
};
static const  char *const test_text [4] =
{
 "Waiting", "Pass", "Fail", "Critical"
};

static const  char *(test_fail_cause [TEST_COUNT]);
static int test_status[TEST_COUNT];
static int show_test_info = FALSE;
static volatile int test_wait = false;
static float test_value;

#ifdef TINYSA4
static freq_t spur_test_freq = 900000000;
static freq_t direct_test_freq = 180000000;

void determine_direct_test_freq(void) {
  if (hw_if)
    direct_test_freq = 870000000;
  return;

  int old_ultra = config.ultra;
  config.ultra = true;
  float max_level = -150;
  set_refer_output(0);
  for (freq_t test_freq = 900000000UL; test_freq < 1000000000UL; test_freq += 30000000) {
    dirty = true;
    float v = PURE_TO_float(perform(false, 0, test_freq, false));
    if (v > max_level) {
      max_level = v;
      direct_test_freq = test_freq;
    }
  }
  config.ultra = old_ultra;
}

static float lpf_test_level = 0;
#define LPF_TEST_FREQ 795000000
void determine_lpf_test_level(void) {
  int old_ultra = config.ultra;
  config.ultra = true;
  int old_lna = setting.extra_lna;
  setting.extra_lna = true;
  force_signal_path = true;
  test_path = 3;
  set_refer_output(1);
  dirty = true;
  lpf_test_level = PURE_TO_float(perform(false, 0, (freq_t)LPF_TEST_FREQ, false));
  dirty = true;
  force_signal_path = false;
  config.ultra = old_ultra;
  setting.extra_lna = old_lna;
}

#endif

static void test_acquire(int i)
{
  (void)i;
  pause_sweep();
  if (test_case[i].kind == TC_LEVEL) {
    float summed_peak_level = 0;
#define LEVEL_TEST_SWEEPS    2
    for (int k=0; k<LEVEL_TEST_SWEEPS; k++) {
      sweep(false);
      float local_peak_level = 0.0;
#define FROM_START  50
      for (int n = FROM_START ; n < sweep_points; n++)
        local_peak_level += actual_t[n];
      local_peak_level /= (sweep_points - FROM_START);
      summed_peak_level += local_peak_level;
    }
    peakLevel = summed_peak_level / LEVEL_TEST_SWEEPS;
  } else
    sweep(false);
  TRACE_ENABLE(TRACE_STORED_FLAG);
  plot_into_index(measured);
  redraw_request |= REDRAW_CELLS | REDRAW_FREQUENCY;
//  draw_all(TRUE);
}

int cell_printf(int16_t x, int16_t y, const char *fmt, ...);
void cell_draw_test_info(int x0, int y0)
{
#define INFO_SPACING    13
//  char self_test_status_buf[35];
  if (!show_test_info)
    return;
  int i = -2;
  do {
    i++;
    int xpos = 25 - x0;
    int ypos = 50+i*INFO_SPACING - y0;
    pixel_t color;
    if (i < 0)
      color = LCD_FG_COLOR;
    else if (test_status[i] == TS_PASS)
      color = LCD_BRIGHT_COLOR_GREEN;
    else if (test_status[i] == TS_CRITICAL)
      color = LCD_TRACE_3_COLOR;          // Yellow
    else if (test_status[i] == TS_FAIL)
      color = LCD_BRIGHT_COLOR_RED;
    else
      color = LCD_BRIGHT_COLOR_BLUE;
    ili9341_set_foreground(color);
    if (i == -1) {
      cell_printf(xpos, ypos, FONT_s"Self test status:");
    } else if (test_case[i].kind == TC_END) {
      if (test_wait)
        cell_printf(xpos, ypos, FONT_s"Touch screen to continue");
      continue;
    } else {
      cell_printf(xpos, ypos, FONT_s"Test %d: %s%s", i+1, test_fail_cause[i], test_text[test_status[i]] );
      continue;
    }
  } while (test_case[i].kind != TC_END);
}

int validate_signal_within(int i, float margin)
{
  test_fail_cause[i] = "Signal level ";
  float test_level = test_case[i].pass;
#ifdef TINYSA4
  if (test_case[i].setup == TP_15MHZ_LNA) {
    test_level = lpf_test_level;
    margin = 5;
  }
  if (fabsf(config.low_level_offset) > 10)
    return(TS_FAIL);
#endif
  if (fabsf(peakLevel-test_level) > 2*margin) {
    return TS_FAIL;
  }
  if (fabsf(peakLevel-test_level) > margin) {
    return TS_CRITICAL;
  }
  if (setting.measurement == M_PASS_BAND) {
    peakFreq = (markers[2].frequency + markers[1].frequency)/2;
    markers[0].frequency = peakFreq;
    markers[0].index = (markers[2].index + markers[1].index)/2;
#ifdef TINYSA4
#define FLATNESS_CRITERIA 2
#else
#define FLATNESS_CRITERIA 0.8
#endif
    if (flatness > FLATNESS_CRITERIA) {
      test_fail_cause[i] = "Flatness ";
      return TS_FAIL;
    }
  }
  freq_t c_freq = (freq_t)(test_case[i].center * 1000000);
#ifdef TINYSA4
  if (test_case[i].setup == TP_30MHZ_DIRECT)
    c_freq = direct_test_freq;
  if (test_case[i].setup == TP_15MHZ_LNA)
    c_freq = LPF_TEST_FREQ;
#endif
  test_fail_cause[i] = "Frequency ";
  if (peakFreq < c_freq - 500000 || c_freq + 500000 < peakFreq )
    return TS_FAIL;
  test_fail_cause[i] = "";
  return TS_PASS;
}

int validate_peak_below(int i, float margin) {
  return(test_case[i].pass - peakLevel > margin);
}

int validate_below(int tc, int from, int to) {
  int status = TS_PASS;
  float threshold=stored_t[from];
  float sum = 0;
  int sum_count = 0;
  for (int j = from; j < to; j++) {
    sum += actual_t[j];
    sum_count++;
    if (actual_t[j] > threshold) {
      status = TS_FAIL;
      break;
    }
  }
  sum = sum / sum_count;
  if (sum > threshold - 5)
    status = TS_CRITICAL;
  if (status != TS_PASS)
    test_fail_cause[tc] = "Above ";
  return(status);
}

int validate_flatness(int i) {          // This assumes only trace 1 is active
  volatile int j,k;
  test_fail_cause[i] = "Passband ";
  for (j = peakIndex; j < setting._sweep_points; j++) {
    if (actual_t[j] < peakLevel - 15)    // Search right -3dB
      break;
  }
  for (k = peakIndex; k > 0; k--) {
    if (actual_t[k] < peakLevel - 15)    // Search left -3dB
      break;
  }
//  shell_printf("Width %d between %d and %d\n\r", j - k, 2* W2P(test_case[i].width), 3* W2P(test_case[i].width) );
  if (j - k < 2* W2P(test_case[i].width))
      return(TS_FAIL);
  if (j - k > 3* W2P(test_case[i].width))
      return(TS_FAIL);
  test_fail_cause[i] = "";
  return(TS_PASS);
}

const float atten_step[7] = { 0.0, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0 };

#ifdef TINYSA4
bool saved_direct;
freq_t saved_direct_start;
freq_t saved_direct_stop;
#endif


int test_validate(int i);

int validate_atten(int i) {
  int status = TS_PASS;
  float reference_peak_level = 0.0;
  test_fail_cause[i] = "Attenuator ";
//  for (int j= 0; j < 64; j++ ) {
  for (int j= 0; j < 7; j++ ) {
    //    float a = ((float)j)/2.0;
    float a = atten_step[j];
    set_attenuation(a);
    setting.repeat = 10;
    test_acquire(TEST_LEVEL);                        // Acquire test, does also the averaging.
    test_validate(TEST_LEVEL);                       // Validate test, does nothing actually
    if (j == 0)
      reference_peak_level = peakLevel;
    else {
      //      if (SDU1.config->usbp->state == USB_ACTIVE)  shell_printf("Attenuation %.2fdB, measured level %.2fdBm, delta %.2fdB\n\r",a, summed_peak_level, summed_peak_level - reference_peak_level);
#define ATTEN_TEST_CRITERIA 1.5
      if (peakLevel - reference_peak_level <= -ATTEN_TEST_CRITERIA || peakLevel - reference_peak_level >= ATTEN_TEST_CRITERIA) {
        status = TS_FAIL;
      }
    }
  }
  if (status == TS_PASS)
    test_fail_cause[i] = "";
  return(status);
}

int validate_display(int tc)
{
  test_fail_cause[tc] = "Display ";
  if (!display_test()) {
    return(TS_FAIL);
  }
  test_fail_cause[tc] = "";
  return(TS_PASS);
}

int validate_above(int tc) {
  int status = TS_FAIL;
  for (int j = 0; j < setting._sweep_points; j++) {
    if (actual_t[j] > stored_t[j]+5) {
      status = TS_PASS;
      break;
    } else if (actual_t[j] > stored_t[j]) {
      status = TS_CRITICAL;
    }
  }
  if (status != TS_PASS)
    test_fail_cause[tc] = "Above ";
  return(status);
}

int validate_level(int i) {
  int status = TS_PASS;
  test_fail_cause[i] = "Level ";
#if 0
  #define LEVEL_TEST_CRITERIA 3
  if (peakLevel - test_case[i].pass <= -LEVEL_TEST_CRITERIA || peakLevel - test_case[i].pass >= LEVEL_TEST_CRITERIA) {
    status = TS_FAIL;
  } else
#endif
    test_fail_cause[i] = "";
  return(status);
}

#ifdef TINYSA4
float measure_jump(int i) {
  redraw_request |= REDRAW_AREA | REDRAW_CAL_STATUS;
  draw_all(TRUE);
  float left=0,
      right = 0;
  int h_p = setting._sweep_points/2;
  for (int j = 0; j < h_p; j++) {
    left += actual_t[j];
  }
  left /= h_p;
  for (int j = h_p; j < h_p*2; j++) {
    right += actual_t[j];
  }
  right /= h_p;
  if (i <= 1)              // for 2MHz jump
    return(right-left); // returns level jump low to high frequency.
  return (left - right);
}
#endif

int test_validate(int i)
{
//  draw_all(TRUE);
#ifdef TINYSA4
  direct_test = false;
#endif
  int current_test_status = TS_PASS;
  switch (test_case[i].kind) {
  case TC_SET:
    if (test_case[i].pass == 0) {
      if (test_value != 0)
        set_actual_power(test_value);
    } else
      set_actual_power(test_case[i].pass);
    goto common;
  case TC_MEASURE:
  case TC_SIGNAL:           // Validate signal
  common: current_test_status = validate_signal_within(i, 10.0);
    if (current_test_status == TS_PASS) {            // Validate noise floor
      current_test_status = validate_below(i, 0, setting._sweep_points/2 - W2P(test_case[i].width));
      if (current_test_status == TS_PASS) {
        current_test_status = validate_below(i, setting._sweep_points/2 + W2P(test_case[i].width), setting._sweep_points);
      }
      if (current_test_status != TS_PASS)
        test_fail_cause[i] = "Stopband ";
    }
    if (current_test_status == TS_PASS && test_case[i].kind == TC_MEASURE)
      test_value = peakLevel;
    else
      test_value = 0;           //   Not valid
  break;
  case TC_ABOVE:   // Validate signal above curve
    current_test_status = validate_above(i);
    break;
  case TC_BELOW:   // Validate signal below curve
    current_test_status = validate_below(i, 0, setting._sweep_points);
    break;
  case TC_FLAT:   // Validate passband flatness
    current_test_status = validate_flatness(i);
    break;
  case TC_ATTEN:
    current_test_status = validate_atten(i);        // Measures and validates the attenuator
    break;
  case TC_LEVEL:
    current_test_status = validate_level(i);
    break;
  case TC_DISPLAY:
    current_test_status = validate_display(i);
    break;
  }

  // Report status

  if (current_test_status != TS_PASS || test_case[i+1].kind == TC_END)
    test_wait = true;
  test_status[i] = current_test_status;     // Must be set before draw_all() !!!!!!!!
  //  draw_frequencies();
//  draw_cal_status();
//  redraw_request |= REDRAW_CAL_STATUS;
  redraw_request |= REDRAW_AREA | REDRAW_CAL_STATUS;
  draw_all(TRUE);
  return current_test_status;
}

void test_prepare(int i)
{
  setting.measurement = M_OFF;
  markers[1].enabled = M_DISABLED;
  markers[2].enabled = M_DISABLED;
  setting.stored[TRACE_STORED] = true;
  setting.tracking = false; //Default test setup
  setting.atten_step = false;
#ifdef TINYSA4
  force_signal_path = false;
  setting.frequency_IF = config.frequency_IF1;                // Default frequency
  setting.extra_lna = false;
#else
  setting.frequency_IF = DEFAULT_IF;                // Default frequency
#endif
#ifdef __ULTRA__
  config.ultra = true;
  ultra_start = 2000000000;
#endif
  setting.auto_IF = true;
  setting.auto_attenuation = false;
  setting.attenuate_x2 = 0;
  setting.spur_removal = S_OFF;
  in_selftest = true;

  switch(test_case[i].setup) {                // Prepare test conditions
  case TPH_SILENT:                             // No input signal
    set_mode(M_HIGH);
    goto common_silent;
#ifdef TINYSA4
  case TP_SILENT_LNA:                             // No input signal
#endif
  case TP_SILENT:                             // No input signal
    set_mode(M_LOW);
common_silent:
    set_refer_output(-1);
    setting.stored[TRACE_STORED] = true;
    for (int j = 0; j < setting._sweep_points; j++)
      stored_t[j] = test_case[i].pass;
    in_selftest = false;                    // Otherwise spurs will be visible
    break;
  case TP_30MHZ_SWITCH:
    set_mode(M_LOW);
    set_refer_output(0);
    goto common;
  case TP_10MHZEXTRA:                         // Swept receiver
    set_mode(M_LOW);
    setting.tracking = true; //Sweep BPF
    setting.auto_IF = false;
#ifdef TINYSA4
    setting.frequency_IF = config.frequency_IF1 + STATIC_DEFAULT_SPUR_OFFSET/3;  // This is the place where the
    set_refer_output(0);
#else
    setting.frequency_IF = DEFAULT_IF+210000;                // Center on SAW filters
    set_refer_output(2);
#endif
    markers[1].enabled = M_ENABLED;
    markers[1].mtype = M_DELTA;
    markers[2].enabled = M_ENABLED;
    markers[2].mtype = M_DELTA;
    setting.measurement = M_PASS_BAND;
    goto common;
  case TP_10MHZ:                              // 10MHz input
    set_mode(M_LOW);
    set_refer_output(2);
    setting.step_delay_mode = SD_PRECISE;
//        set_step_delay(1);                      // Precise scanning speed
#ifdef __SPUR__
#ifdef TINYSA4
    setting.spur_removal = S_AUTO_OFF;
#else
    setting.spur_removal = S_ON;
#endif
#endif
 common:
   setting.stored[TRACE_STORED] = true;
    for (int j = 0; j < setting._sweep_points/2 - W2P(test_case[i].width); j++)
      stored_t[j] = test_case[i].stop;
    for (int j = setting._sweep_points/2 + W2P(test_case[i].width); j < setting._sweep_points; j++)
#ifdef TINYSA4
      stored_t[j] = test_case[i].stop;
#else
      stored_t[j] = test_case[i].stop - (i == 6?3:0);
#endif
    for (int j = setting._sweep_points/2 - W2P(test_case[i].width); j < setting._sweep_points/2 + W2P(test_case[i].width); j++) {
#ifdef TINYSA4
      if (test_case[i].setup == TP_15MHZ_LNA)
        stored_t[j] = lpf_test_level;
      else
#endif
      stored_t[j] = test_case[i].pass;
    }
    break;
#ifdef TINYSA4
  case TP_15MHZ_LNA:
    determine_lpf_test_level();
    goto simple;
  case TP_15MHZ_LNA2:
    force_signal_path = true;
    test_path = 1;
    goto simple;
  case TP_30MHZ_DIRECT:
  case TP_30MHZ_ULTRA:
  case TP_30MHZ_LNA:
#endif
  case TP_30MHZ:
#ifdef TINYSA4
    simple:
#endif
    set_mode(M_LOW);
#ifdef TINYSA4
    maxFreq = 9900000000ULL;            // needed to measure the LPF rejection
#else
    maxFreq = 2000000000;            // needed to measure the LPF rejection
#endif
    set_refer_output(0);
    dirty = true;
 //   set_step_delay(1);                      // Do not set !!!!!
#ifdef __SPUR__
    setting.spur_removal = S_ON;
#endif

    goto common;
  case TPH_30MHZ_SWITCH:
  case TPH_30MHZ:
    set_mode(M_HIGH);
    set_refer_output(0);
    setting.spur_removal = S_ON;
    goto common;
  }
  switch(test_case[i].setup) {                // Prepare test conditions
#ifdef TINYSA4
  case TP_30MHZ_ULTRA:
    ultra_start = 0;
    break;
  case TP_30MHZ_DIRECT:
    ultra_start = (hw_if ? 900000000 : 800000000);
    direct_test = true;
    determine_direct_test_freq();
    break;
  case TP_SILENT_LNA:
  case TP_30MHZ_LNA:
    setting.extra_lna = true;
    break;
  case TP_15MHZ_LNA:
  case TP_15MHZ_LNA2:
    set_refer_output(1);
    setting.extra_lna = true;
    break;
#endif
  case TP_30MHZ_SWITCH:
    set_attenuation(32);                        // This forces the switch to transmit so isolation can be tested
    break;
  case TPH_30MHZ_SWITCH:
    set_attenuation(0);
    setting.atten_step = true;               // test high switch isolation
    break;
  default:
    set_attenuation(0.0);
  }
  TRACE_ENABLE(TRACE_STORED_FLAG);
  setting.stored[TRACE_STORED] = true;
  set_reflevel(test_case[i].pass+10);
#ifdef TINYSA4
  if (test_case[i].kind == TC_JUMP) {
    set_sweep_frequency(ST_CENTER, test_freq);
    setting.repeat = 10;
  }
  else
#endif
  {
    freq_t c_freq = (freq_t)(test_case[i].center * 1000000);
#ifdef TINYSA4
    if (test_case[i].setup == TP_30MHZ_DIRECT)
      c_freq = direct_test_freq;
    if (test_case[i].setup == TP_15MHZ_LNA)
      c_freq = LPF_TEST_FREQ;
#endif
    set_sweep_frequency(ST_CENTER, c_freq);
  }
  set_sweep_frequency(ST_SPAN, (freq_t)(test_case[i].span * 1000000));
  draw_cal_status();
}

extern void menu_autosettings_cb(int item, uint16_t data);

#ifdef TINYSA4
int last_spur = 0;
int add_spur(int f, float p)
{
  for (int i = 0; i < last_spur; i++) {
    if (temp_t[i]-1 <= f && f <= temp_t[i]+1) {
      stored_t[i] += 1;
      if (stored2_t[i] < p)
        stored2_t[i] = p;
      return stored_t[i];
    }
  }
  if (last_spur < POINTS_COUNT) {
    temp_t[last_spur] = f;
#if TRACES_MAX == 4
    stored2_t[last_spur] = p;
#endif
    stored_t[last_spur++] = 1;
  }
  return 1;
}

void sort_dynamic_spur_table(void) {
  for (int counter = 0 ; counter < dynamic_spur_table_size - 1; counter++)
  {
    for (int counter1 = 0 ; counter1 < dynamic_spur_table_size - counter - 1; counter1++)
    {
      if (dynamic_spur_table[counter1] > dynamic_spur_table[counter1+1]) //increasing frequency
      {
        float swap_var = dynamic_spur_table[counter1]; dynamic_spur_table[counter1] = dynamic_spur_table[counter1+1]; dynamic_spur_table[counter1+1] = swap_var;
      }
    }
  }
}

void sort_spur_freq(void) {
  for (int counter = 0 ; counter < last_spur - 1; counter++)
  {
    for (int counter1 = 0 ; counter1 < last_spur - counter - 1; counter1++)
    {
      if (temp_t[counter1] > temp_t[counter1+1]) //increasing frequency
      {
        float swap_var = temp_t[counter1]; temp_t[counter1] = temp_t[counter1+1]; temp_t[counter1+1] = swap_var;
              swap_var = stored_t[counter1]; stored_t[counter1] = stored_t[counter1+1]; stored_t[counter1+1] = swap_var;
              swap_var = stored2_t[counter1]; stored2_t[counter1] = stored2_t[counter1+1]; stored2_t[counter1+1] = swap_var;
      }
    }
  }
}

void sort_spur_level(void) {
  for (int counter = 0 ; counter < last_spur - 1; counter++)
  {
    for (int counter1 = 0 ; counter1 < last_spur - counter - 1; counter1++)
    {
      if (stored2_t[counter1] < stored2_t[counter1+1]) // decreasing level
      {
        float swap_var = temp_t[counter1]; temp_t[counter1] = temp_t[counter1+1]; temp_t[counter1+1] = swap_var;
              swap_var = stored_t[counter1]; stored_t[counter1] = stored_t[counter1+1]; stored_t[counter1+1] = swap_var;
              swap_var = stored2_t[counter1]; stored2_t[counter1] = stored2_t[counter1+1]; stored2_t[counter1+1] = swap_var;
      }
    }
  }
}

void sort_spur_count(void) {
  for (int counter = 0 ; counter < last_spur - 1; counter++)
  {
    for (int counter1 = 0 ; counter1 < last_spur - counter - 1; counter1++)
    {
      if (stored_t[counter1] < stored_t[counter1+1]) // decreasing count
      {
        float swap_var = temp_t[counter1]; temp_t[counter1] = temp_t[counter1+1]; temp_t[counter1+1] = swap_var;
              swap_var = stored_t[counter1]; stored_t[counter1] = stored_t[counter1+1]; stored_t[counter1+1] = swap_var;
              swap_var = stored2_t[counter1]; stored2_t[counter1] = stored2_t[counter1+1]; stored2_t[counter1+1] = swap_var;
      }
    }
  }
}

#endif

//static bool test_wait = false;
static int test_step = 0;

void self_test(int test)
{
  bool no_wait = false;
#ifdef TINYSA4
  bool old_ultra = config.ultra;
  config.ultra = true;
  if (adc_vbat_read() < 3800) {
    drawMessageBox("Battery low", "Charge before testing", 2000);
    goto quit;
  }
#endif
  setting.external_gain = 0;                  // Must be 0
//  set_sweep_points(POINTS_COUNT);
  if (test == 0) {
    if (test_wait ) {
      if (test_case[test_step].kind == TC_END || setting.test_argument != 0)
        goto resume2;
      else
        goto resume;
    }
    // Disable waterfall on selftest
    if (setting.waterfall)
      disable_waterfall();
    reset_settings(M_LOW);                      // Make sure we are in a defined state
    in_selftest = true;
    ignore_stored = true;
    menu_autosettings_cb(0, 0);
    for (uint16_t i=0; i < TEST_COUNT; i++) {          // All test cases waiting
      if (test_case[i].kind == TC_END)
        break;
      test_status[i] = TS_WAITING;
      test_fail_cause[i] = "";
    }
    show_test_info = TRUE;
    test_step=0;
    test_step = setting.test_argument;
    if (test_step != 0) {
      if (test_step < 0) {
        test_step = -test_step;
        no_wait = true;
      }
      test_step -= 1;
    }
    do {
      test_prepare(test_step);
      test_acquire(test_step);                        // Acquire test
      test_status[test_step] = test_validate(test_step);                       // Validate test

      if (test_step == 2) {
        if (peakLevel < -60) {
          test_step = TEST_END;
          ili9341_set_foreground(LCD_BRIGHT_COLOR_RED);
          ili9341_drawstring_7x13("Signal level too low", 30, 140);
          ili9341_drawstring_7x13("Did you connect high and low ports with cable?", 0, 210);
          goto resume2;
        }

      }
      if (test_status[test_step] != TS_PASS) {
        if (no_wait) {
          peakFreq = 0;   // Avoid changing IF
          goto quit;
        }
        resume:
        test_wait = true;
        if (!check_touched())
          return;
//        wait_user();
      }
      test_step++;
    } while (test_case[test_step].kind != TC_END && setting.test_argument == 0 );
    if (no_wait) {
      goto quit;
    }
//    draw_all(TRUE);
    ili9341_set_foreground(LCD_BRIGHT_COLOR_GREEN);
    ili9341_drawstring_7x13("Self test complete", 50, 202);
    ili9341_drawstring_7x13("Touch screen to continue", 50, 215);
   resume2:
    test_wait = true;
    if (!check_touched())
      return;
quit:
#ifdef TINYSA4
  config.ultra = old_ultra;
#endif
    sweep_mode = SWEEP_ENABLE;
    test_wait = false;
    if (setting.test_argument == 0) ili9341_clear_screen();
#ifdef TINYSA4
    config_recall();
    config.cor_am = 0;
    config.cor_nfm = 0;
    config.cor_wfm = 0;
#endif
    in_selftest = false;
    ignore_stored = false;
    reset_settings(M_LOW);
    set_refer_output(-1);
#ifdef TINYSA4
  } else if (test == 1) {
    float average, p;
    freq_t start = get_sweep_frequency(ST_START);
    freq_t stop  = get_sweep_frequency(ST_STOP);
    debug_avoid = false;
    in_selftest = true;               // Spur search
    reset_settings(M_LOW);
    test_prepare(TEST_SILENCE);
    setting.stored[TRACE_STORED] = true;    // Prevent overwriting when used
    setting.stored[TRACE_TEMP] = true;
    setting.stored[TRACE_STORED2] = true;
    setting.below_IF = S_OFF;
    freq_t f;
#ifdef TINYSA4
    setting.frequency_step = 3000;
#else
   setting.auto_IF = false;
   setting.frequency_IF=DEFAULT_IF;
   setting.frequency_step = 30000;
 #endif
   if (setting.test_argument > 0) {
     setting.frequency_step = setting.test_argument;
     shell_printf("\n\rResetting spur table\n\r");
    //  int i = 0;                     // Index in spur table (temp_t)
     dynamic_spur_table_size = 0;         // Reset table before scanning
   } else
     shell_printf("\n\rAdding to current dynamic spur table\n\r");

    set_RBW(setting.frequency_step/100);
    last_spur = 0;
    for (int j = 0; j < 4; j++) {
      int k=0;
      f = start;
      average = PURE_TO_float(perform(false, 0, f, false));
      vbwSteps = 1;
      f += setting.frequency_step;
      average += PURE_TO_float(perform(false, 1, f, false));
      average /= 2.0;
      f += setting.frequency_step;
      shell_printf("\n\rStarting with average of %4.2f and IF at %DHz and step of %DHz till %DHz\n\r", average, setting.frequency_IF, setting.frequency_step, stop );
//      while (f < NORMAL_MAX_FREQ && !global_abort) {
      while (f < stop && !global_abort) {
        if ((k++ % 1000) == 0)
          shell_printf("Pass %d, freq %D\r", j+1, f);
        int cnt = 0;
        p = 0;
#define SPUR_CHECK_COUNT 2 // 4
        do {
          cnt++;
          p = PURE_TO_float(perform(false, 1, f, false));
#ifdef TINYSA4
#define SPUR_DELTA  10
#else
#define SPUR_DELTA  15
#endif
        } while ( average + SPUR_DELTA < p  && cnt < SPUR_CHECK_COUNT);
        if (cnt == SPUR_CHECK_COUNT && average + SPUR_DELTA < p) {
          shell_printf("Pass %d, %4.2fdBm spur at %DkHz with count %d\n\r", j+1, p,f/1000, add_spur(f, p));
        }
        average = (average*19+p)/20;
        f += setting.frequency_step;
      }
    }
    shell_printf("\n\rTable for IF at %D and step of %D\n\r", setting.frequency_IF, setting.frequency_step);
    shell_printf("Freq(kHz), count, level(dBm), level with spur removal\n\r");
    in_selftest = false;
    setting.spur_removal = S_ON;
    sort_spur_count();                  // Reduce table to most certain spurs
    if (last_spur > MAX_DYNAMIC_SPUR_TABLE_SIZE)
      last_spur = MAX_DYNAMIC_SPUR_TABLE_SIZE;
    sort_spur_level();                  // Reduce table to only strongest spurs
    if (last_spur > MAX_DYNAMIC_SPUR_TABLE_SIZE)
      last_spur = MAX_DYNAMIC_SPUR_TABLE_SIZE;
    sort_spur_freq();
//    dynamic_spur_table_size = 0;
    for (int j = 0; j < last_spur; j++) {
      if ((int)stored_t[j] >= 1 && j < MAX_DYNAMIC_SPUR_TABLE_SIZE && (int)stored_t[j] > SPUR_CHECK_COUNT-2 && stored_t[j] - stored2_t[j] > 2) {
        shell_printf("%d, %d, %4.2f, %4.2f\n\r", ((int)temp_t[j])/1000, (int)stored_t[j], stored2_t[j], PURE_TO_float(perform(false, 1, (freq_t)temp_t[j], false)));
        dynamic_spur_table[dynamic_spur_table_size++] = temp_t[j];
      }
    }
    sort_dynamic_spur_table();
    always_use_dynamic_table = true;
    reset_settings(M_LOW);
    set_sweep_frequency(ST_START, start);
    set_sweep_frequency(ST_STOP, stop);
#endif
  } else if (false && test == 2) {                                   // Attenuator test
    in_selftest = true;
    reset_settings(M_LOW);
#if 1
    float reference_peak_level = 0;
    int c = 0;
    for (int j= 0; j < 64; j += 4 ) {
      test_prepare(TEST_LEVEL);
      set_attenuation(((float)j)/2.0);
      if (setting.test_argument)
        set_sweep_frequency(ST_CENTER, ((freq_t)setting.test_argument));
#ifdef __ULTRA__
      ultra_start = (config.ultra_start == ULTRA_AUTO ? ULTRA_THRESHOLD : config.ultra_start);
#endif
      test_acquire(TEST_LEVEL);                        // Acquire test
	  test_validate(TEST_LEVEL);                       // Validate test
      if (j == 0)
        reference_peak_level = peakLevel;
      shell_printf("Attenuation %.2fdB, measured level %.2fdBm, delta %.2fdB\n\r",((float)j)/2.0, peakLevel, peakLevel - reference_peak_level);
      if ((j % 4)  == 0) {
        age[c++] = (uint8_t)((int)((peakLevel - reference_peak_level) * 8)+128);
      }

    }
    shell_printf("  {");
    for (int i=0; i < 16;i++)
      shell_printf("%d, ", (int)(((int)age[i])-128));
    shell_printf("}\n\r");

#else
    test_prepare(TEST_ATTEN);
    test_acquire(TEST_ATTEN);                        // Acquire test
    test_validate(TEST_ATTEN);                       // Validate test
#endif
    reset_settings(M_LOW);
#ifdef TINYSA4
  } else if (test == 3) {                       // RBW step time search
#define R_TABLE_SIZE 5
static int R_table[R_TABLE_SIZE] = {1,3,-3,4,5};

    int calculate_step[R_TABLE_SIZE][SI4432_RBW_count];
    in_step_test = true;
    in_selftest = true;
    ui_mode_normal();
    test_prepare(TEST_RBW);
//    reset_settings(M_LOW);
    setting.auto_IF = false;
#ifdef TINYSA4
    int old_setting_r = setting.R;
    setting.R = 1;                              // force to highest scan speed
    setting.frequency_IF=config.frequency_IF1;
    setting.step_delay = 25000;
#else
    setting.frequency_IF=DEFAULT_IF;
    setting.step_delay = 8000;
#endif
    for (int j= 0; j < SI4432_RBW_count; j++ ) {
      if (setting.test_argument != 0)
        j = setting.test_argument;
// do_again:
#ifdef TINYSA4
      for (int r=0;r<R_TABLE_SIZE;r++) {
      setting.R = R_table[r];                              // force to highest scan speed
#endif
      test_prepare(TEST_RBW);
      setting.spur_removal = S_OFF;
#if 1               // Disable for offset baseline scanning
      setting.step_delay_mode = SD_NORMAL;
      setting.repeat = 1;
#else
      setting.step_delay_mode = SD_FAST;
      setting.repeat = 20;
#endif
      setting.step_delay = setting.step_delay * 5 / 4;
      if (setting.step_delay < 1000)
        setting.step_delay = 1000;
      setting.offset_delay = setting.step_delay ;
      setting.rbw_x10 = force_rbw(j);
#ifdef TINYSA4
      shell_printf("RBW=%5.1f, R=%+d, ",setting.rbw_x10/10.0, setting.R);
#else
      shell_printf("RBW=%5.1f, ",setting.rbw_x10/10.0);
#endif
#if 0
      set_sweep_frequency(ST_SPAN, (freq_t)(setting.rbw_x10 * 1000));     // Wide
#else
      if (setting.rbw_x10 < 1000)
        set_sweep_frequency(ST_SPAN, (freq_t)(setting.rbw_x10 * 5000));   // Narrow
      else
        set_sweep_frequency(ST_SPAN, (freq_t)(18000000));
#endif
      test_acquire(TEST_RBW);                        // Acquire test
      test_validate(TEST_RBW);                       // Validate test
//      if (test_value == 0) {
//        setting.step_delay = setting.step_delay * 4 / 5;
//        goto do_again;
//      }
 //     if (peakLevel < -35) {
 //       shell_printf("Peak level too low, abort\n\r");
 //       return;
 //     }
      float aver_noise = 0;
      for (int i=0;i<50;i++)
         aver_noise += actual_t[i];
      aver_noise /= 50;
      float saved_aver_noise = aver_noise;
      shell_printf("Start level=%6.1f, noise=%4.1f, delay=%5d\n\r",peakLevel, aver_noise, setting.step_delay);

#if 1                                                                       // Enable for step delay tuning
      float saved_peakLevel = peakLevel;
      while (setting.step_delay > 10 && test_value != 0 && test_value > saved_peakLevel - 1.5 && aver_noise < saved_aver_noise + 2) {
        test_prepare(TEST_RBW);
        setting.spur_removal = S_OFF;
        setting.step_delay_mode = SD_NORMAL;
        setting.step_delay = setting.step_delay * 4 / 5;
        if (setting.rbw_x10 < 1000)
          set_sweep_frequency(ST_SPAN, (freq_t)(setting.rbw_x10 * 5000));
        else
          set_sweep_frequency(ST_SPAN, (freq_t)(18000000));

//        setting.repeat = 10;
        test_acquire(TEST_RBW);                        // Acquire test
        test_validate(TEST_RBW);                       // Validate test
        aver_noise = 0;
        for (int i=0;i<50;i++)
           aver_noise += actual_t[i];
        aver_noise /= 50;
//        shell_printf(" Level, noise, delay = %f, %f, %d\n\r",peakLevel, aver_noise, setting.step_delay);
      }

      setting.step_delay = setting.step_delay * 5 / 4;          // back one level
#else
      setting.step_delay = setting.step_delay * 4 / 5;

#endif
#ifdef TINYSA4
      setting.offset_delay = 10000;
#else
      setting.offset_delay = 1600;
#endif
#if 0                       // Enable for offset tuning stepping
      test_value = saved_peakLevel;
      if ((uint32_t)(setting.rbw_x10 * 1000) / (sweep_points) < 8000) {           // fast mode possible
        while (setting.offset_delay > 0 && test_value != 0 && test_value > saved_peakLevel - 1.5) {
          test_prepare(TEST_RBW);
          setting.step_delay_mode = SD_FAST;
          setting.offset_delay = setting.offset_delay * 4 / 5;
          setting.spur_removal = S_OFF;
          if (setting.rbw_x10 < 1000)
            set_sweep_frequency(ST_SPAN, (freq_t)(setting.rbw_x10 * 5000));   // 50 times RBW
          else
            set_sweep_frequency(ST_SPAN, (freq_t)(18000000));     // Limit to 18MHz
//          setting.repeat = 10;
          test_acquire(TEST_RBW);                        // Acquire test
          test_validate(TEST_RBW);                       // Validate test
      shell_printf(" Offset delay %f, %d\n\r",peakLevel, setting.offset_delay);
        }
        setting.offset_delay = setting.offset_delay * 5 / 4;            // back one level
      }
#endif
#ifdef TINYSA4
      calculate_step[r][j] = setting.step_delay;
      shell_printf("RBW=%5.1f, R=%+d, End level=%6.1f, noise=%4.1f, delay=%5d, fast delay=%d\n\r",setting.rbw_x10/10.0, setting.R, peakLevel, aver_noise, setting.step_delay, setting.offset_delay);
#else
      calculate_step[j] = setting.step_delay;
      shell_printf("End level=%5.1f, noise=%6.1f, delay=%5d, fast delay = %d\n\r",peakLevel, aver_noise, setting.step_delay, setting.offset_delay);
#endif
      shell_printf("---------------------------------------------\n\r");
      }
      if (setting.test_argument != 0)
        break;
    }

#ifdef TINYSA4
      setting.R = old_setting_r;
#endif
    for (int j= 0; j < SI4432_RBW_count; j++ ) {
#ifdef TINYSA4
      for (int r=0;r<R_TABLE_SIZE;r++) {
        shell_printf("RBW=%4.1f, R=%+d, time=%5d, fast delay=%4d\n\r",force_rbw(j)/10.0, R_table[r], calculate_step[r][j], setting.offset_delay);
      }
#else
      shell_printf("RBW=%4.1f, time=%5d, fast delay=%4d\n\r",force_rbw(j)/10.0, calculate_step[r][j], setting.offset_delay);
#endif
    }
    in_step_test = false;
    in_selftest = false;
    reset_settings(M_LOW);
    setting.step_delay_mode = SD_NORMAL;
    setting.step_delay = 0;
#endif
#if 0                   // No longer needed
  } else if (false && test == 4) {           // Calibrate modulation frequencies
    reset_settings(M_LOW);
    set_mode(M_GENLOW);
    set_sweep_frequency(ST_CENTER, (freq_t)30000000);
    set_sweep_frequency(ST_SPAN, (freq_t)0);
    setting.modulation = MO_AM;
    setting.modulation_frequency = 5000;
    in_selftest = true;
    config.cor_am = 0;
    perform(false,0, 30000000, false);
    perform(false,1, 30000000, false);
    config.cor_am = -(start_of_sweep_timestamp - (ONE_SECOND_TIME / setting.modulation_frequency))/8;

    setting.modulation = MO_NFM;
    setting.modulation_frequency = 5000;
    in_selftest = true;
    config.cor_nfm = 0;
    perform(false,0, 30000000, false);
    perform(false,1, 30000000, false);
    config.cor_nfm = -(start_of_sweep_timestamp - (ONE_SECOND_TIME / setting.modulation_frequency))/8;
#ifdef TINYSA4
    setting.modulation = MO_NFM2;
    setting.modulation_frequency = 5000;
    in_selftest = true;
    config.cor_nfm2 = 0;
    perform(false,0, 30000000, false);
    perform(false,1, 30000000, false);
    config.cor_nfm2 = -(start_of_sweep_timestamp - (ONE_SECOND_TIME / setting.modulation_frequency))/8;

    setting.modulation = MO_NFM3;
    setting.modulation_frequency = 5000;
    in_selftest = true;
    config.cor_nfm3 = 0;
    perform(false,0, 30000000, false);
    perform(false,1, 30000000, false);
    config.cor_nfm3 = -(start_of_sweep_timestamp - (ONE_SECOND_TIME / setting.modulation_frequency))/8;
#endif
    setting.modulation = MO_WFM;
    setting.modulation_frequency = 5000;
    in_selftest = true;
    config.cor_wfm = 0;
    perform(false,0, 30000000, false);
    perform(false,1, 30000000, false);
    config.cor_wfm = -(start_of_sweep_timestamp - (ONE_SECOND_TIME / setting.modulation_frequency))/8;

    //    shell_printf("\n\rCycle time = %d\n\r", start_of_sweep_timestamp);
    reset_settings(M_LOW);
#endif
  } else if (false && test == 5) {
//    reset_settings(M_LOW);                      // Make sure we are in a defined state
    in_selftest = true;
    switch (setting.test_argument) {
    case 0:
      touch_draw_test();
      area_width  = AREA_WIDTH_NORMAL;
      area_height = AREA_HEIGHT_NORMAL;
      break;
    case 1:
      reset_settings(M_LOW);
      set_sweep_frequency(ST_START, 0);
      set_sweep_frequency(ST_STOP, 50000000);
      break;
    case 2:
      reset_settings(M_LOW);
      set_sweep_frequency(ST_START, 300000000);
      set_sweep_frequency(ST_STOP, NORMAL_MAX_FREQ);
      break;
    case 3:
      reset_settings(M_HIGH);
      set_sweep_frequency(ST_START, 300000000);
      set_sweep_frequency(ST_STOP, NORMAL_MAX_FREQ);
      break;
    case 4:
      reset_settings(M_GENLOW);
      set_sweep_frequency(ST_CENTER, 20000000);
      set_sweep_frequency(ST_SPAN, 0);
      setting.mute = false;
      break;
    case 5:
      reset_settings(M_GENHIGH);
      set_sweep_frequency(ST_CENTER, 320000000);
      set_sweep_frequency(ST_SPAN, 0);
      break;
    }
#ifdef TINYSA4
  } else if (test == 6) {
    in_selftest = true;               // MCU Spur search
    reset_settings(M_LOW);
    test_prepare(TEST_SPUR);
    set_RBW(300);
#ifdef TINYSA4
    setting.extra_lna = true;
#endif
    for (int i = 0; i < 31; i++) {
      hsical = (RCC->CR & 0xff00) >> 8;
      RCC->CR &= RCC_CR_HSICAL;
      RCC->CR |= ( (hsical) << 8 );
      RCC->CR &= RCC_CR_HSITRIM | RCC_CR_HSION; /* CR Reset value.              */
      RCC->CR |= (i << 3 ) & RCC_CR_HSITRIM;
      test_acquire(TEST_SPUR);                        // Acquire test
      shell_printf("%d: %9.3q\n\r",i, peakFreq);
      test_validate(TEST_SPUR);                       // Validate test
    }
#endif
#ifdef TINYSA4
  } else if (test == 7) {                       // RBW level test, param -1 keeps correction
    int arg = setting.test_argument;
    in_selftest = true;
    ui_mode_normal();
    set_scale(2);
    set_reflevel(-22);
    shell_printf("\n\r");
    float first_level=-35.0;
//    setting.R = 3;
    if (arg == -1) {
      switch_SI4463_RSSI_correction(false);
      setting.test_argument = 0;
    }
    for (int j= SI4432_RBW_count-1; j >= 0; j-- ) {
      if (setting.test_argument != 0)
        j = setting.test_argument;
      test_prepare(TEST_LEVEL);
      setting.rbw_x10 = force_rbw(j);
      osalThreadSleepMilliseconds(200);
//      setting.spur_removal = S_ON;
      setting.R = 3;
      set_average(0,AV_100);
      test_acquire(TEST_LEVEL);                        // Acquire test
      test_acquire(TEST_LEVEL);                        // Acquire test
      test_acquire(TEST_LEVEL);                        // Acquire test
      test_acquire(TEST_LEVEL);                        // Acquire test
      test_acquire(TEST_LEVEL);                        // Acquire test
      test_validate(TEST_LEVEL);                       // Validate test
      if (j == SI4432_RBW_count-1)
        first_level = peakLevel;
      shell_printf("RBW = %7.1fk, level = %6.2f, corr = %6.2f\n\r",actual_rbw_x10/10.0 , peakLevel, (first_level - peakLevel)*10.0 );
      if (setting.test_argument != 0)
        break;
    }
#if 1
    for (int k = 0; k< 4; k++) {
      shell_printf("\n\r%d ", k);
      for (int j= SI4432_RBW_count-1; j >= 0; j-- ) {
        if (setting.test_argument != 0)
          j = setting.test_argument;
        test_prepare(TEST_RBW);
//        setting.step_delay_mode = SD_PRECISE;
        set_repeat(5);
        setting.rbw_x10 = force_rbw(j);
        set_sweep_frequency(ST_SPAN, (freq_t)(setting.rbw_x10 * (1000 << k)));
        set_average(0,AV_100);
        test_acquire(TEST_RBW);                        // Acquire test
        test_acquire(TEST_RBW);                        // Acquire test
        test_acquire(TEST_RBW);                        // Acquire test
        test_acquire(TEST_RBW);                        // Acquire test
        test_acquire(TEST_RBW);                        // Acquire test
        test_validate(TEST_RBW);                       // Validate test
//        if (j == SI4432_RBW_count-1)
//          first_level = peakLevel;
//        shell_printf("RBW = %7.1fk, level = %6.2f, corr = %6.2f\n\r",actual_rbw_x10/10.0 , peakLevel, (first_level - peakLevel)*10.0 );
        shell_printf("%6.2f ", (first_level - peakLevel)*10.0 );
        if (setting.test_argument != 0)
          break;
        if (operation_requested) goto abort;
      }
    }
#endif
abort:
    shell_printf("\n\r");
    setting.R = 0;
    switch_SI4463_RSSI_correction(true);
    reset_settings(M_LOW);
  } else if (test == 8) {                       // RBW level test
    in_selftest = true;
    ui_mode_normal();
//    set_scale(2);
    set_reflevel(-100);
    shell_printf("\n\r");
    float first_level=-166.0;
//    setting.R = 3;
    switch_SI4463_RSSI_correction(false);
    for (int j= SI4432_RBW_count-1; j >= 0; j-- ) {
      if (setting.test_argument != 0)
        j = setting.test_argument;
      test_prepare(TEST_NOISE);
      markers[0].mtype = M_NOISE | M_AVER;
      setting.rbw_x10 = force_rbw(j);
      setting.extra_lna = true;
      osalThreadSleepMilliseconds(200);
      set_average(0,AV_100);
      for (int w=0; w<50; w++) {
        test_acquire(TC_LEVEL);                        // Acquire test
      }
      test_validate(TEST_NOISE);                       // Validate test
      peakLevel += - logf(actual_rbw_x10*100.0) * (10.0/logf(10.0))
     #ifdef TINYSA4
         + SI4463_noise_correction_x10/10.0
     #endif
         ;
      if (j == SI4432_RBW_count-1)
        first_level = peakLevel;
      shell_printf("RBW = %7.1fk, level = %6.2f, corr = %6.2f\n\r",actual_rbw_x10/10.0 , peakLevel, (first_level - peakLevel)*10.0 );
      if (setting.test_argument != 0)
        break;
    }
#if 1               // Does not center on frequency!!!!!

    for (int k = 0; k< 4; k++) {
      shell_printf("\n\r%d ", k);
      for (int j= SI4432_RBW_count-1; j >= 0; j-- ) {
        if (setting.test_argument != 0)
          j = setting.test_argument;
        test_prepare(TEST_NOISE_RBW);
//        setting.step_delay_mode = SD_PRECISE;
        set_repeat(5);
        setting.rbw_x10 = force_rbw(j);
        setting.extra_lna = true;
        osalThreadSleepMilliseconds(200);
        markers[0].mtype = M_NOISE | M_AVER;
        set_sweep_frequency(ST_SPAN, (freq_t)(setting.rbw_x10 * (1000 << k)));
        set_average(0,AV_100);
        test_acquire(TC_LEVEL);                        // Acquire test
        test_acquire(TC_LEVEL);                        // Acquire test
        test_acquire(TC_LEVEL);                        // Acquire test
        test_acquire(TC_LEVEL);                        // Acquire test
        test_acquire(TC_LEVEL);                        // Acquire test
        test_validate(TEST_NOISE_RBW);                       // Validate test
        peakLevel += - logf(actual_rbw_x10*100.0) * (10.0/logf(10.0))
       #ifdef TINYSA4
           + SI4463_noise_correction_x10/10.0
       #endif
           ;
        //        if (j == SI4432_RBW_count-1)
        //          first_level = peakLevel;
        //        shell_printf("RBW = %7.1fk, level = %6.2f, corr = %6.2f\n\r",actual_rbw_x10/10.0 , peakLevel, (first_level - peakLevel)*10.0 );
        shell_printf("%6.2f ", (first_level - peakLevel)*10.0 );
        if (setting.test_argument != 0)
          break;
        if (operation_requested) goto abort;
      }
      if (operation_requested) break;
    }
#endif
    shell_printf("\n\r");
    setting.R = 0;
    switch_SI4463_RSSI_correction(true);
    reset_settings(M_LOW);
  } else if (test == 9) {                       // temperature level
    in_selftest = true;
    ui_mode_normal();
    set_scale(2);
    set_reflevel(-22);
    float first_level=-23.5;
    while (Si446x_get_temp() < 45.0) {
//    setting.R = 3;
      test_prepare(TEST_LEVEL);
      test_acquire(TEST_LEVEL);                        // Acquire test
      test_validate(TEST_LEVEL);                       // Validate test
      shell_printf("Temp = %4.1f, level = %6.2f, delta = %6.2f\n\r",Si446x_get_temp() , peakLevel, (first_level - peakLevel)*10.0 );
      if (operation_requested) break;
    }
  } else if (test == 10) {          // Test 30MHz spurs
//    reset_settings(M_LOW);
    set_refer_output(-1);
    if (setting.test_argument > 0)
      set_R(((int)setting.test_argument) % 10);
    int freq_step = 30;
    if (setting.test_argument > 9)
      freq_step = (((int)setting.test_argument)/10) * 1000000 ;
        set_attenuation(0);
    int test_case = TEST_POWER;
    for (freq_t f=freq_step; f<900000000; f += freq_step) {
      set_sweep_points(51);
      set_sweep_frequency(ST_CENTER, f);
      set_sweep_frequency(ST_SPAN, 3000);
      test_acquire(test_case);                        // Acquire test
      test_validate(test_case);
      shell_printf("Freq = %8.3fMHz, level = %6.2f\n\r", ((float)peakFreq) / 1000000.0, peakLevel);
      if (operation_requested) break;
    }
    set_sweep_points(450);
    reset_settings(M_LOW);
#endif
  }


  show_test_info = FALSE;
  in_selftest = false;
  test_wait = false;
  sweep_mode = SWEEP_ENABLE;
}

void reset_calibration(void)
{
  config.input_is_calibrated = false;
  config.output_is_calibrated = false;
  config.low_level_offset = 0;
  config.low_level_output_offset = 0;

}

void calibrate_modulation(int modulation, int8_t *correction)
{
  if (*correction == 0) {
    setting.modulation = modulation;
    setting.modulation_frequency = 7000;
    dirty = true;
    in_selftest = true;
    perform(false,0, 30000000, false);
    perform(false,1, 30000000, false);
    in_selftest = false;
    *correction = -start_of_sweep_timestamp;  // uses 10 microsecond delay
    setting.modulation = M_OFF;
    setting.modulation_frequency = 400;
  }
}

#define CALIBRATE_RBWS  1
const int power_rbw [5] = { 100, 300, 30, 10, 3 };

#ifdef TINYSA4

#define JUMP_FREQS 7
#define HARMONIC_JUMP   5
freq_t jump_freqs[JUMP_FREQS] = {LOW_SHIFT_FREQ, LOW_SHIFT_FREQ, DRIVE1_MAX_FREQ, DRIVE2_MAX_FREQ, 0, 0, 0};

void set_jump_freq(freq_t a, freq_t b, freq_t c) {
  jump_freqs[4] = a;
  jump_freqs[5] = b;
  jump_freqs[6] = c;
}

void set_jump_config(int i, float v) {
  switch (i) {
  case 0:
    config.shift1_level_offset = v;
    break;
  case 1:
    config.shift2_level_offset = v;
    break;
  case 2:
    config.drive2_level_offset = v;
    break;
  case 3:
    config.drive3_level_offset = v;
    break;
  case 4:
    config.shift3_level_offset = v;
    break;
  case 5:
    config.harmonic_level_offset = v;
    break;
  case 6:
    break;
  }
}

float get_jump_config(int i) {
  switch (i) {
  case 0:
    return config.shift1_level_offset;
  case 1:
    return config.shift2_level_offset;
  case 2:
    return config.drive2_level_offset;
  case 3:
    return config.drive3_level_offset;
  case 4:
    return config.shift3_level_offset;
  case 5:
    return config.harmonic_level_offset;
  case 6:
    return 0.0;
  }
  return 0;
}

enum {CS_NORMAL, CS_LNA, CS_SWITCH, CS_ULTRA, CS_ULTRA_LNA, CS_DIRECT_REF, /* CS_DIRECT,*/ CS_DIRECT_LNA, CS_SPUR_REF, CS_SPUR_ERROR, CS_HARMONIC, CS_HARMONIC_LNA, /* CS_BPF_REF, CS_BPF, */ CS_CORRECTION_REF, CS_CORRECTION_LNA, CS_MAX };
#define ULTRA_HARMONIC_CAL_FREQ 5340000000
#else
enum {CS_NORMAL, CS_SWITCH, CS_MAX };
#endif

void calibration_busy(void) {
  ili9341_set_foreground(LCD_BRIGHT_COLOR_GREEN);
  ili9341_drawstring_7x13("Calibration busy", 40, 220);
}


#ifdef TINYSA4
void calibrate_harmonic(void)
{
  int old_sweep_points = setting._sweep_points;
  int old_ultra = config.ultra;
  config.ultra = true;
//  setting.auto_IF = true;                         // set in selftest
//  setting.frequency_IF = config.frequency_IF1;    // set in selftest
  set_sweep_points(450);
//  setting.scale = 1;
//  set_trace_scale(1);

  {
    test_freq = jump_freqs[HARMONIC_JUMP];
    set_jump_config(HARMONIC_JUMP, -2);
    test_prepare(TEST_JUMP_HARMONIC);
    set_RBW(1000);
//    set_auto_reflevel(true);
    set_reflevel(-20);
    setting.repeat = 10;
    test_acquire(TEST_JUMP_HARMONIC);                        // Acquire test
    if (peakLevel < -60) {
      ili9341_set_foreground(LCD_BRIGHT_COLOR_RED);
      ili9341_drawstring_7x13("Signal level too low or not on frequency", 30, 200);
      goto quit;
    }
    set_jump_config(HARMONIC_JUMP, get_jump_config(HARMONIC_JUMP) + measure_jump(HARMONIC_JUMP));
    calibration_busy();
    chThdSleepMilliseconds(500);
    test_acquire(TEST_JUMP_HARMONIC);                        // Acquire test
    set_jump_config(HARMONIC_JUMP, get_jump_config(HARMONIC_JUMP) + measure_jump(HARMONIC_JUMP));
    calibration_busy();
    chThdSleepMilliseconds(500);
  }
  config.ultra = old_ultra;
  setting.scale = 10;
  set_trace_scale(10);
  force_signal_path = false;
  config_save();
  ili9341_set_foreground(LCD_BRIGHT_COLOR_GREEN);
  ili9341_drawstring_7x13("Calibration complete", 40, 200);
  quit:
  config.ultra = old_ultra;
  ili9341_drawstring_7x13("Touch screen to continue", 40, 220);
  wait_user();
  ili9341_clear_screen();
  set_sweep_points(old_sweep_points);
  in_selftest = false;
  reset_settings(M_LOW);
  set_refer_output(-1);
  test_wait = false;
}
#endif

void calibrate(void)
{
  int local_test_status;
  int old_sweep_points = setting._sweep_points;
#ifdef TINYSA4
  int old_ultra = config.ultra;
  config.ultra = true;
  freq_t old_correction = config.setting_frequency_30mhz;
  config.setting_frequency_30mhz = 3000000000;
  ADF4351_recalculate_PFDRFout();
  dirty = true;

//  setting.auto_IF = true;                         // set in selftest
//  setting.frequency_IF = config.frequency_IF1;    // set in selftest
  float direct_level=0.0;
  setting.test_argument = -7;
  self_test(0);
  int if_error = peakFreq - 30000000;
  if (if_error > -1000000 && if_error < 1000000) {
    config.frequency_IF1 += if_error;
    setting.auto_IF = true;
    setting.frequency_IF = config.frequency_IF1;
    fill_spur_table();
  }
  if (peakLevel < -40 || peakLevel > -30)
    goto low_level;
  determine_direct_test_freq();
#if 1   // Jump calibration not yet enabled
  //for (int j = 0; j < CALIBRATE_RBWS; j++) {
  //  set_RBW(power_rbw[j]);

  setting.scale = 1;
  set_trace_scale(1);
  for (int i =0; i<JUMP_FREQS-1; i++) { // Don't do harmonic
    test_freq = jump_freqs[i];
    set_jump_config(i, -2);
    test_prepare(TEST_JUMP);
    set_RBW(1000);
    set_auto_reflevel(true);
    setting.repeat = 10;
    //setting.scale = 1;
    if (i == 5) {                       // Not used !!!!!
      set_sweep_points(51);
      set_sweep_frequency(ST_SPAN,    50);
      setting.rbw_x10 = 3;
      test_path = 3;      // Ultra lna path
      force_signal_path = true;
//      set_reflevel(-95);
    } else if (i == 4) {
//        set_sweep_points(51);
        set_sweep_frequency(ST_SPAN,    1000);
        set_RBW(100);
//        setting.rbw_x10 = 3;
        test_path = 3;      // Ultra lna path
        force_signal_path = true;
        setting.spur_removal = S_OFF;
//        set_reflevel(-95);
    } else if (i <= 1) {
      if (i == 1)
        set_RBW(8500);
      set_refer_output(5);          // 2MHz
      setting.spur_removal = S_OFF;
    } else {
//      set_reflevel(-70);
      set_refer_output(0);          // 30MHz
      setting.spur_removal = S_AUTO_OFF;
    }

    test_acquire(TEST_JUMP);                        // Acquire test
    set_reflevel(actual_t[sweep_points/3]+5);
    plot_into_index(measured);
    redraw_request |= REDRAW_AREA | REDRAW_CAL_STATUS;
    draw_all(TRUE);
    set_jump_config(i, get_jump_config(i) + measure_jump(i));
    calibration_busy();
    chThdSleepMilliseconds(100);
    test_acquire(TEST_JUMP);                        // Acquire test
    set_jump_config(i, get_jump_config(i) + measure_jump(i));
    calibration_busy();
    chThdSleepMilliseconds(100);
  }
  setting.scale = 10;
  set_trace_scale(10);
  set_sweep_points(POINTS_COUNT);
  force_signal_path = false;

#endif
#endif
  reset_calibration();
  in_calibration = true;
#ifdef TINYSA4
  int current_correction_calibration = 0;
#endif
  for (calibration_stage = CS_NORMAL; calibration_stage < CS_MAX ; calibration_stage++) {
    for (int k = 0; k<3; k++) {     // max 3 retries
      float offset = 0.0;
      for (int j= 0; j < CALIBRATE_RBWS; j++ ) {
#if 1
        reset_settings(M_LOW);
        set_refer_output(0);
#ifdef TINYSA4
        config.ultra = true;          // Enable ultra
        maxFreq = 12000000000;
        set_attenuation(0);
#else
        set_attenuation(10);
#endif
        set_sweep_frequency(ST_CENTER, 30000000);
#ifdef TINYSA4
        set_sweep_frequency(ST_SPAN,    1000);
        markers[0].mtype |= M_AVER;
        setting.repeat = 10;
#else
        set_sweep_frequency(ST_SPAN,    5000000);
        setting.repeat = 10;
#endif
        setting.rbw_x10 = 3000;
        int test_case = TEST_POWER;

        //      setting.atten_step = false;
        //#ifdef TINYSA4
        //       set_extra_lna(false);
        //       setting.below_IF = S_AUTO_OFF;
        //#endif
        switch(calibration_stage) {
        case CS_NORMAL:
          force_signal_path = false;
          break;
        case CS_SWITCH:
          setting.atten_step = true;
          force_signal_path = false;
          break;
#ifdef TINYSA4
        case CS_ULTRA:
          test_path = 2;      // Ultra path
          force_signal_path = true;
          break;
        case CS_ULTRA_LNA:
          test_path = 3;      // Ultra lna path
          force_signal_path = true;
          break;
        case CS_LNA:
          test_path = 1;      // Normal lna path
          force_signal_path = true;
          break;

        case CS_SPUR_REF:
          setting.spur_removal = S_OFF;
          goto spur_common;
        case CS_SPUR_ERROR:
          setting.spur_removal = S_AUTO_OFF;
          config.shift_level_offset = 0;
          spur_common:
          set_sweep_frequency(ST_CENTER, spur_test_freq);
          test_path = 2;      // Ultra
          force_signal_path = true;
          break;
        case CS_DIRECT_REF:
          test_path = 0;      // Normal
          setting.spur_removal = S_OFF;
          direct_common:
          set_sweep_frequency(ST_CENTER, direct_test_freq);
          force_signal_path = true;
          break;
#if 0
        case CS_DIRECT:
          test_path = 4;      // Direct path at 900MHz
          goto direct_common;
#endif
        case CS_DIRECT_LNA:
          test_path = 5;      // Direct lna path at 900MHz
          goto direct_common;
        case CS_HARMONIC:
          test_path = 6;      // harmonic path
          force_signal_path = true;
          break;
        case CS_HARMONIC_LNA:
          test_path = 7;      // harmonic lna path
          force_signal_path = true;
          break;
        case CS_CORRECTION_REF:
          if (!max2871)
            goto done;
          set_sweep_frequency(ST_CENTER, config.correction_frequency[1][to_calibrate[current_correction_calibration]]);
          test_path = 0;
          force_signal_path = true;
          set_refer_output(2);
          break;
        case CS_CORRECTION_LNA:
          set_sweep_frequency(ST_CENTER, config.correction_frequency[1][to_calibrate[current_correction_calibration]]);
          test_path = 1;
          force_signal_path = true;
          set_refer_output(2);
          break;
#endif
        }
        set_average(0, AV_100);
        for (int m=1; m<=1; m++) {
          test_acquire(test_case);                        // Acquire test
          local_test_status = test_validate(test_case);
          calibration_busy();
        }
        local_test_status = TS_PASS;            // Must be forced because test_validate calculates pass band wrong
#else
        //    set_RBW(power_rbw[j]);
        //    set_sweep_points(21);
#if 0
        int test_case = TEST_POWER;
        test_prepare(test_case);
        setting.step_delay_mode = SD_PRECISE;
#ifndef TINYSA4
        setting.agc = S_ON;
        setting.lna = S_OFF;
        //      set_RBW(6000);
#else
        set_RBW(3000);
#endif
        set_attenuation(10);
        set_repeat(5);
        setting.spur_removal = S_OFF;
        set_average(0,AV_100);
        test_acquire(test_case);                        // Acquire test
        test_acquire(test_case);                        // Acquire test
        test_acquire(test_case);                        // Acquire test
        local_test_status = test_validate(test_case);                       // Validate test
#else
        int test_case = TEST_LEVEL;
#ifdef TINYSA4
        if (calibrate_lna)
          test_case += 1;
#endif
        test_prepare(test_case);
        set_RBW(3000);
        set_attenuation(10);
        set_average(0,AV_100);
        test_acquire(test_case);                        // Acquire test
        test_acquire(test_case);                        // Acquire test
        test_acquire(test_case);                        // Acquire test
        test_acquire(test_case);                        // Acquire test
        local_test_status = test_validate(test_case);                       // Validate test also sets attenuation if zero span
#endif
#endif
        if ((calibration_stage == CS_NORMAL && peakLevel < -40)
#ifdef TINYSA4
            || (calibration_stage == CS_LNA && peakLevel < -40)
            || (calibration_stage == CS_ULTRA && peakLevel < -40)
            || (calibration_stage == CS_DIRECT_LNA && peakLevel < direct_level - 50)
#endif
        ) {
#ifdef TINYSA4
low_level:
#endif
          ili9341_set_foreground(LCD_BRIGHT_COLOR_RED);
          ili9341_drawstring_7x13("Signal level incorrect", 30, 200);
          ili9341_drawstring_7x13("Check cable between connectors", 30, 220);
          goto quit;
        }
        //    chThdSleepMilliseconds(1000);
        if (local_test_status != TS_PASS) {
          ili9341_set_foreground(LCD_BRIGHT_COLOR_RED);
          ili9341_drawstring_7x13("Calibration failed", 30, 220);
          goto quit;
        }
#ifdef TINYSA4
        if (calibration_stage == CS_CORRECTION_REF) {
          direct_level = marker_to_value(0);
        } else if (calibration_stage == CS_CORRECTION_LNA) {
          if (direct_level > -90) // Check if on harmonic
            set_actual_correction_value(1,to_calibrate[current_correction_calibration], direct_level);
          if ((unsigned int) current_correction_calibration < sizeof(to_calibrate)/sizeof(int) - 1) {
            current_correction_calibration++;
            calibration_stage -= 2;             // skip back to REF measurement
          }
        }
        else if (calibration_stage == CS_SPUR_REF)
          direct_level = marker_to_value(0);                // Re-use direct_level;
        else if (calibration_stage == CS_SPUR_ERROR)
           config.shift_level_offset = direct_level -  marker_to_value(0);
        else if (calibration_stage == CS_DIRECT_REF)
          direct_level = marker_to_value(0);
//        else if (calibration_stage == CS_DIRECT)
//          offset = set_actual_power(direct_level);
        else if (calibration_stage == CS_DIRECT_LNA){
          offset = set_actual_power(direct_level);
          config.direct_level_offset = config.direct_lna_level_offset - (config.low_level_offset - config.lna_level_offset);
        }
        else
#endif
          offset = set_actual_power(CAL_LEVEL);           // Should be -23.5dBm (V0.2) OR 25 (V0.3)
        calibration_busy();
        chThdSleepMilliseconds(100);
      }
      if (offset > -0.2 && offset < 0.2)
        k = 3;
    }
  }
#ifdef TINYSA4
done:
#endif
  setting.below_IF = S_AUTO_OFF;
  in_calibration = false;
#ifdef TINYSA4
  config.setting_frequency_30mhz = old_correction;
  config.ultra = old_ultra;
#endif

  #if 0               // No high input calibration as CAL OUTPUT is unreliable

  set_RBW(100);
  test_prepare(TEST_POWER+1);
  test_acquire(TEST_POWER+1);                        // Acquire test
  float last_peak_level = peakLevel;
  local_test_status = test_validate(TEST_POWER+1);                       // Validate test
  chThdSleepMilliseconds(1000);

  config.high_level_offset = 0;           /// Preliminary setting

  for (int j = 0; j < CALIBRATE_RBWS; j++) {
    set_RBW(power_rbw[j]);
    test_prepare(TEST_POWER+2);
    test_acquire(TEST_POWER+2);                        // Acquire test
    local_test_status = test_validate(TEST_POWER+2);                       // Validate test
//    if (local_test_status != TS_PASS) {                       // Do not validate due to variations in SI4432
//      ili9341_set_foreground(BRIGHT_COLOR_RED);
//      ili9341_drawstring_7x13("Calibration failed", 30, 120);
//      goto quit;
//    } else
      set_actual_power(last_peak_level);
      chThdSleepMilliseconds(1000);
  }

#endif
  config.input_is_calibrated = true;
#ifdef TINYSA4
  config.ultra = old_ultra;
#endif
  config_save();
  ili9341_set_foreground(LCD_BRIGHT_COLOR_GREEN);
  ili9341_drawstring_7x13("Calibration complete", 40, 220);
quit:
#ifdef TINYSA4
  config.ultra = old_ultra;
#endif
  ili9341_drawstring_7x13("Touch screen to continue", 40, 240);
  wait_user();
  ili9341_clear_screen();
  set_sweep_points(old_sweep_points);
  in_selftest = false;
//  set_refer_output(-1);
  reset_settings(M_LOW);
  set_refer_output(-1);
  test_wait = false;
}


#ifdef TINYSA4

#define PI  3.1415926535897932384626433832795


// Fast Fourier Transform. length must be exactly 2^n.
// inverse = true computes InverseFFT
// inverse = false computes FFT.
// Overwrites the real and imaginary arrays in-place

void FFT(float *real, float *imag, int length, bool inverse)
{

    float wreal, wpreal, wimag, wpimag, theta;
    float tempreal, tempimag, tempwreal, direction;

    int Addr, Position, Mask, BitRevAddr, PairAddr;
    int m, k;


    direction = -1.0;       // direction of rotating phasor for FFT

    if(inverse)
        direction = 1.0;    // direction of rotating phasor for IFFT

    //  bit-reverse the addresses of both the real and imaginary arrays
    //  real[0..length-1] and imag[0..length-1] are the paired complex numbers

    for (Addr=0; Addr<length; Addr++)
    {
        // Derive Bit-Reversed Address
        BitRevAddr = 0;
        Position = length >> 1;
        Mask = Addr;
        while (Mask)
        {
            if(Mask & 1)
                BitRevAddr += Position;
            Mask >>= 1;
            Position >>= 1;
        }

        if (BitRevAddr > Addr)              // Swap
        {
            float s;
            s = real[BitRevAddr];           // real part
            real[BitRevAddr] = real[Addr];
            real[Addr] = s;
            s = imag[BitRevAddr];           // imaginary part
            imag[BitRevAddr] = imag[Addr];
            imag[Addr] = s;
        }
    }

    // FFT, IFFT Kernel

    for (k=1; k < length; k <<= 1)
    {
        theta = direction * PI / (float)k;
        wpimag = sinf(theta);
        wpreal = cosf(theta);
        wreal = 1.0;
        wimag = 0.0;

        for (m=0; m < k; m++)
        {
            for (Addr = m; Addr < length; Addr += (k*2))
            {
                PairAddr = Addr + k;

                tempreal = wreal * real[PairAddr] - wimag * imag[PairAddr];
                tempimag = wreal * imag[PairAddr] + wimag * real[PairAddr];
                real[PairAddr] = real[Addr] - tempreal;
                imag[PairAddr] = imag[Addr] - tempimag;
                real[Addr] += tempreal;
                imag[Addr] += tempimag;
            }
            tempwreal = wreal;
            wreal = wreal * wpreal - wimag * wpimag;
            wimag = wimag * wpreal + tempwreal * wpimag;
        }
    }

    if(inverse)                         // Normalize the IFFT coefficients
        for(int i=0; i<length; i++)
        {
            real[i] /= (float)length;
            imag[i] /= (float)length;
        }

}

#endif


#pragma GCC pop_options

#if 0                           // fixed point FFT

/* fix_fft.c - Fixed-point in-place Fast Fourier Transform  */
/*
  All data are fixed-point short integers, in which -32768
  to +32768 represent -1.0 to +1.0 respectively. Integer
  arithmetic is used for speed, instead of the more natural
  floating-point.

  For the forward FFT (time -> freq), fixed scaling is
  performed to prevent arithmetic overflow, and to map a 0dB
  sine/cosine wave (i.e. amplitude = 32767) to two -6dB freq
  coefficients. The return value is always 0.

  For the inverse FFT (freq -> time), fixed scaling cannot be
  done, as two 0dB coefficients would sum to a peak amplitude
  of 64K, overflowing the 32k range of the fixed-point integers.
  Thus, the fix_fft() routine performs variable scaling, and
  returns a value which is the number of bits LEFT by which
  the output must be shifted to get the actual amplitude
  (i.e. if fix_fft() returns 3, each value of fr[] and fi[]
  must be multiplied by 8 (2**3) for proper scaling.
  Clearly, this cannot be done within fixed-point short
  integers. In practice, if the result is to be used as a
  filter, the scale_shift can usually be ignored, as the
  result will be approximately correctly normalized as is.

  Written by:  Tom Roberts  11/8/89
  Made portable:  Malcolm Slaney 12/15/94 malcolm@interval.com
  Enhanced:  Dimitrios P. Bouras  14 Jun 2006 dbouras@ieee.org
*/

#define N_WAVE      1024    /* full length of Sinewave[] */
#define LOG2_N_WAVE 10      /* log2(N_WAVE) */

/*
  Henceforth "short" implies 16-bit word. If this is not
  the case in your architecture, please replace "short"
  with a type definition which *is* a 16-bit word.
*/

/*
  Since we only use 3/4 of N_WAVE, we define only
  this many samples, in order to conserve data space.
*/
short Sinewave[N_WAVE-N_WAVE/4] = {
      0,    201,    402,    603,    804,   1005,   1206,   1406,
   1607,   1808,   2009,   2209,   2410,   2610,   2811,   3011,
   3211,   3411,   3611,   3811,   4011,   4210,   4409,   4608,
   4807,   5006,   5205,   5403,   5601,   5799,   5997,   6195,
   6392,   6589,   6786,   6982,   7179,   7375,   7571,   7766,
   7961,   8156,   8351,   8545,   8739,   8932,   9126,   9319,
   9511,   9703,   9895,  10087,  10278,  10469,  10659,  10849,
  11038,  11227,  11416,  11604,  11792,  11980,  12166,  12353,
  12539,  12724,  12909,  13094,  13278,  13462,  13645,  13827,
  14009,  14191,  14372,  14552,  14732,  14911,  15090,  15268,
  15446,  15623,  15799,  15975,  16150,  16325,  16499,  16672,
  16845,  17017,  17189,  17360,  17530,  17699,  17868,  18036,
  18204,  18371,  18537,  18702,  18867,  19031,  19194,  19357,
  19519,  19680,  19840,  20000,  20159,  20317,  20474,  20631,
  20787,  20942,  21096,  21249,  21402,  21554,  21705,  21855,
  22004,  22153,  22301,  22448,  22594,  22739,  22883,  23027,
  23169,  23311,  23452,  23592,  23731,  23869,  24006,  24143,
  24278,  24413,  24546,  24679,  24811,  24942,  25072,  25201,
  25329,  25456,  25582,  25707,  25831,  25954,  26077,  26198,
  26318,  26437,  26556,  26673,  26789,  26905,  27019,  27132,
  27244,  27355,  27466,  27575,  27683,  27790,  27896,  28001,
  28105,  28208,  28309,  28410,  28510,  28608,  28706,  28802,
  28897,  28992,  29085,  29177,  29268,  29358,  29446,  29534,
  29621,  29706,  29790,  29873,  29955,  30036,  30116,  30195,
  30272,  30349,  30424,  30498,  30571,  30643,  30713,  30783,
  30851,  30918,  30984,  31049,  31113,  31175,  31236,  31297,
  31356,  31413,  31470,  31525,  31580,  31633,  31684,  31735,
  31785,  31833,  31880,  31926,  31970,  32014,  32056,  32097,
  32137,  32176,  32213,  32249,  32284,  32318,  32350,  32382,
  32412,  32441,  32468,  32495,  32520,  32544,  32567,  32588,
  32609,  32628,  32646,  32662,  32678,  32692,  32705,  32717,
  32727,  32736,  32744,  32751,  32757,  32761,  32764,  32766,
  32767,  32766,  32764,  32761,  32757,  32751,  32744,  32736,
  32727,  32717,  32705,  32692,  32678,  32662,  32646,  32628,
  32609,  32588,  32567,  32544,  32520,  32495,  32468,  32441,
  32412,  32382,  32350,  32318,  32284,  32249,  32213,  32176,
  32137,  32097,  32056,  32014,  31970,  31926,  31880,  31833,
  31785,  31735,  31684,  31633,  31580,  31525,  31470,  31413,
  31356,  31297,  31236,  31175,  31113,  31049,  30984,  30918,
  30851,  30783,  30713,  30643,  30571,  30498,  30424,  30349,
  30272,  30195,  30116,  30036,  29955,  29873,  29790,  29706,
  29621,  29534,  29446,  29358,  29268,  29177,  29085,  28992,
  28897,  28802,  28706,  28608,  28510,  28410,  28309,  28208,
  28105,  28001,  27896,  27790,  27683,  27575,  27466,  27355,
  27244,  27132,  27019,  26905,  26789,  26673,  26556,  26437,
  26318,  26198,  26077,  25954,  25831,  25707,  25582,  25456,
  25329,  25201,  25072,  24942,  24811,  24679,  24546,  24413,
  24278,  24143,  24006,  23869,  23731,  23592,  23452,  23311,
  23169,  23027,  22883,  22739,  22594,  22448,  22301,  22153,
  22004,  21855,  21705,  21554,  21402,  21249,  21096,  20942,
  20787,  20631,  20474,  20317,  20159,  20000,  19840,  19680,
  19519,  19357,  19194,  19031,  18867,  18702,  18537,  18371,
  18204,  18036,  17868,  17699,  17530,  17360,  17189,  17017,
  16845,  16672,  16499,  16325,  16150,  15975,  15799,  15623,
  15446,  15268,  15090,  14911,  14732,  14552,  14372,  14191,
  14009,  13827,  13645,  13462,  13278,  13094,  12909,  12724,
  12539,  12353,  12166,  11980,  11792,  11604,  11416,  11227,
  11038,  10849,  10659,  10469,  10278,  10087,   9895,   9703,
   9511,   9319,   9126,   8932,   8739,   8545,   8351,   8156,
   7961,   7766,   7571,   7375,   7179,   6982,   6786,   6589,
   6392,   6195,   5997,   5799,   5601,   5403,   5205,   5006,
   4807,   4608,   4409,   4210,   4011,   3811,   3611,   3411,
   3211,   3011,   2811,   2610,   2410,   2209,   2009,   1808,
   1607,   1406,   1206,   1005,    804,    603,    402,    201,
      0,   -201,   -402,   -603,   -804,  -1005,  -1206,  -1406,
  -1607,  -1808,  -2009,  -2209,  -2410,  -2610,  -2811,  -3011,
  -3211,  -3411,  -3611,  -3811,  -4011,  -4210,  -4409,  -4608,
  -4807,  -5006,  -5205,  -5403,  -5601,  -5799,  -5997,  -6195,
  -6392,  -6589,  -6786,  -6982,  -7179,  -7375,  -7571,  -7766,
  -7961,  -8156,  -8351,  -8545,  -8739,  -8932,  -9126,  -9319,
  -9511,  -9703,  -9895, -10087, -10278, -10469, -10659, -10849,
 -11038, -11227, -11416, -11604, -11792, -11980, -12166, -12353,
 -12539, -12724, -12909, -13094, -13278, -13462, -13645, -13827,
 -14009, -14191, -14372, -14552, -14732, -14911, -15090, -15268,
 -15446, -15623, -15799, -15975, -16150, -16325, -16499, -16672,
 -16845, -17017, -17189, -17360, -17530, -17699, -17868, -18036,
 -18204, -18371, -18537, -18702, -18867, -19031, -19194, -19357,
 -19519, -19680, -19840, -20000, -20159, -20317, -20474, -20631,
 -20787, -20942, -21096, -21249, -21402, -21554, -21705, -21855,
 -22004, -22153, -22301, -22448, -22594, -22739, -22883, -23027,
 -23169, -23311, -23452, -23592, -23731, -23869, -24006, -24143,
 -24278, -24413, -24546, -24679, -24811, -24942, -25072, -25201,
 -25329, -25456, -25582, -25707, -25831, -25954, -26077, -26198,
 -26318, -26437, -26556, -26673, -26789, -26905, -27019, -27132,
 -27244, -27355, -27466, -27575, -27683, -27790, -27896, -28001,
 -28105, -28208, -28309, -28410, -28510, -28608, -28706, -28802,
 -28897, -28992, -29085, -29177, -29268, -29358, -29446, -29534,
 -29621, -29706, -29790, -29873, -29955, -30036, -30116, -30195,
 -30272, -30349, -30424, -30498, -30571, -30643, -30713, -30783,
 -30851, -30918, -30984, -31049, -31113, -31175, -31236, -31297,
 -31356, -31413, -31470, -31525, -31580, -31633, -31684, -31735,
 -31785, -31833, -31880, -31926, -31970, -32014, -32056, -32097,
 -32137, -32176, -32213, -32249, -32284, -32318, -32350, -32382,
 -32412, -32441, -32468, -32495, -32520, -32544, -32567, -32588,
 -32609, -32628, -32646, -32662, -32678, -32692, -32705, -32717,
 -32727, -32736, -32744, -32751, -32757, -32761, -32764, -32766,
};

/*
  FIX_MPY() - fixed-point multiplication & scaling.
  Substitute inline assembly for hardware-specific
  optimization suited to a particluar DSP processor.
  Scaling ensures that result remains 16-bit.
*/
inline short FIX_MPY(short a, short b)
{
    /* shift right one less bit (i.e. 15-1) */
    int c = ((int)a * (int)b) >> 14;
    /* last bit shifted out = rounding-bit */
    b = c & 0x01;
    /* last shift + rounding bit */
    a = (c >> 1) + b;
    return a;
}

/*
  fix_fft() - perform forward/inverse fast Fourier transform.
  fr[n],fi[n] are real and imaginary arrays, both INPUT AND
  RESULT (in-place FFT), with 0 <= n < 2**m; set inverse to
  0 for forward transform (FFT), or 1 for iFFT.
*/
int fix_fft(short fr[], short fi[], short m, short inverse)
{
    int mr, nn, i, j, l, k, istep, n, scale, shift;
    short qr, qi, tr, ti, wr, wi;

    n = 1 << m;

    /* max FFT size = N_WAVE */
    if (n > N_WAVE)
        return -1;

    mr = 0;
    nn = n - 1;
    scale = 0;

    /* decimation in time - re-order data */
    for (m=1; m<=nn; ++m) {
        l = n;
        do {
            l >>= 1;
        } while (mr+l > nn);
        mr = (mr & (l-1)) + l;

        if (mr <= m)
            continue;
        tr = fr[m];
        fr[m] = fr[mr];
        fr[mr] = tr;
        ti = fi[m];
        fi[m] = fi[mr];
        fi[mr] = ti;
    }

    l = 1;
    k = LOG2_N_WAVE-1;
    while (l < n) {
        if (inverse) {
            /* variable scaling, depending upon data */
            shift = 0;
            for (i=0; i<n; ++i) {
                j = fr[i];
                if (j < 0)
                    j = -j;
                m = fi[i];
                if (m < 0)
                    m = -m;
                if (j > 16383 || m > 16383) {
                    shift = 1;
                    break;
                }
            }
            if (shift)
                ++scale;
        } else {
            /*
              fixed scaling, for proper normalization --
              there will be log2(n) passes, so this results
              in an overall factor of 1/n, distributed to
              maximize arithmetic accuracy.
            */
            shift = 1;
        }
        /*
          it may not be obvious, but the shift will be
          performed on each data point exactly once,
          during this pass.
        */
        istep = l << 1;
        for (m=0; m<l; ++m) {
            j = m << k;
            /* 0 <= j < N_WAVE/2 */
            wr =  Sinewave[j+N_WAVE/4];
            wi = -Sinewave[j];
            if (inverse)
                wi = -wi;
            if (shift) {
                wr >>= 1;
                wi >>= 1;
            }
            for (i=m; i<n; i+=istep) {
                j = i + l;
                tr = FIX_MPY(wr,fr[j]) - FIX_MPY(wi,fi[j]);
                ti = FIX_MPY(wr,fi[j]) + FIX_MPY(wi,fr[j]);
                qr = fr[i];
                qi = fi[i];
                if (shift) {
                    qr >>= 1;
                    qi >>= 1;
                }
                fr[j] = qr - tr;
                fi[j] = qi - ti;
                fr[i] = qr + tr;
                fi[i] = qi + ti;
            }
        }
        --k;
        l = istep;
    }
    return scale;
}

/*
  fix_fftr() - forward/inverse FFT on array of real numbers.
  Real FFT/iFFT using half-size complex FFT by distributing
  even/odd samples into real/imaginary arrays respectively.
  In order to save data space (i.e. to avoid two arrays, one
  for real, one for imaginary samples), we proceed in the
  following two steps: a) samples are rearranged in the real
  array so that all even samples are in places 0-(N/2-1) and
  all imaginary samples in places (N/2)-(N-1), and b) fix_fft
  is called with fr and fi pointing to index 0 and index N/2
  respectively in the original array. The above guarantees
  that fix_fft "sees" consecutive real samples as alternating
  real and imaginary samples in the complex array.
*/
int fix_fftr(short f[], int m, int inverse)
{
    int i, N = 1<<(m-1), scale = 0;
    short tt, *fr=f, *fi=&f[N];

    if (inverse)
        scale = fix_fft(fi, fr, m-1, inverse);
    for (i=1; i<N; i+=2) {
        tt = f[N+i-1];
        f[N+i-1] = f[i];
        f[i] = tt;
    }
    if (! inverse)
        scale = fix_fft(fi, fr, m-1, inverse);
    return scale;
}
#endif
