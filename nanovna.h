/* Copyright (c) 2014-2015, TAKAHASHI Tomohiro (TTRFTECH) edy555@gmail.com
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

#ifdef TINYSA_F303
#ifdef TINYSA_F072
#error "Remove comment for #ifdef TINYSA_F303"
#endif
#ifndef TINYSA4
#define TINYSA4
#endif
#define TINYSA4_PROTO
#endif

#ifdef TINYSA_F072
#ifdef TINYSA_F303
#error "Remove comment for #ifdef TINYSA_F072"
#endif
#ifndef TINYSA3
#define TINYSA3
#endif
#endif

// Need enable HAL_USE_SPI in halconf.h
#define __USE_DISPLAY_DMA__

//#define __DEBUG_SPUR__

#define __SA__
#ifdef TINYSA3
#define __SI4432__
#endif
#ifdef TINYSA4
//#define __DISABLE_HOT_INSERT__
#define __SI4463__
#define __SI4468__
#define __ADF4351__
#define __NEW_SWITCHES__
#define __ULTRA_OUT__        // Use ADF output over LOW out
#define __SI5351__
#endif
#define __PE4302__
//#define __SIMULATION__
#define __SCROLL__                // Add waterfall option
#define __LEVEL_METER__
#define __DRAW_LINE__
#define __ICONS__
#define __MEASURE__
#define __LINEARITY__             // Not available
#define __SELFTEST__              // Add selftest option (not fully disable it)
#define __CALIBRATE__             // Add calibration menu and functions
//#define __WATCHDOG__
//#ifdef __WATCHDOG__  // still resets when entering the Ultra code
//#if (HAL_USE_WDG != TRUE)
//#error "HAL_USE_WDG must be set to true"
//#endif
//#if (STM32_WDG_USE_IWDG != TRUE)
//#error "STM32_WDG_USE_IWDG must be set to true"
//#endif
//#endif
#define __FAST_SWEEP__            // Pre-fill SI4432 RSSI buffer  to get fastest sweep in zero span mode
// #define __AUDIO__
#define __SPUR__                  // Does spur reduction by shifting IF
#define __USE_SERIAL_CONSOLE__  // Enable serial I/O connection (need enable HAL_USE_SERIAL as TRUE in halconf.h)
#ifdef __USE_SERIAL_CONSOLE__
//#if (HAL_USE_SERIAL != TRUE)
//#error "HAL_USE_SERIAL must be set to true"
//#endif
#endif
#define __SINGLE_LETTER__         // Add fast console commands
#define __NICE_BIG_FONT__         // Add not scaled big font for menus
#define __QUASI_PEAK__            // Add quasi peak average option
#define __REMOTE_DESKTOP__        // Add remote desktop option
#define __LISTEN__
#define __CHANNEL_POWER__
#define __LIMITS__
#define __CURVE_EDIT__
#ifdef TINYSA3
#define __HAS_DFU__
#define __MCU_CLOCK_SHIFT__
#define __HAM_BAND__
#endif
#ifdef TINYSA4
#define __BANDS__
#define __BEEP__
#define __MULTI_BAND__
#define __HAM_BAND__
#define __TRIGGER_TRACE__
#define __MCU_CLOCK_SHIFT__
#define __ULTRA__
#define __USE_RTC__               // Enable RTC clock
#define __USE_SD_CARD__           // Enable SD card support
//#define __SD_CARD_LOAD__          // Allow run commands from SD card (config.ini in root), if enabled __SD_FILE_BROWSER__ scripts run from *.cmd in it
#define __SD_CARD_DUMP_FIRMWARE__ // Allow dump firmware to SD card
#define __SD_FILE_BROWSER__
#define __LCD_BRIGHTNESS__        // LCD or hardware allow change brightness, add menu item for this
#define __HARMONIC__
#define __NOISE_FIGURE__
#define __VBW__
#define __SWEEP_RESTART__
// #define DIRECT_CORRECTION        // Not enough space for config in one flash page.
#define DB_PER_DEGREE_BELOW               0.056
#define DB_PER_DEGREE_ABOVE               0.069
#define CENTER_TEMPERATURE          34.0
#define __WAIT_CTS_WHILE_SLEEPING__
#define __MARKER_CACHE__
#define TINYSA4_4
#ifdef TINYSA4_4
//#define __SI5351__
#endif
//#define __FFT_VBW__
//#define __FFT_DECONV__
#else
//#define __ULTRA__
//#define __HARMONIC__
//#define __USE_FREQ_TABLE__      // Enable use table for frequency list
#endif

#ifdef __BANDS__
#define __PWM__
#endif


#ifdef TINYSA3
typedef uint32_t freq_t;
#define VARIANT(X,Y) (X)
#define DEFAULT_IF  433800000
#define DEFAULT_SPUR_IF 434000000
#define DEFAULT_MAX_FREQ    350000000
#define MAX_LO_FREQ         959800000UL
#define MIN_LO_FREQ         240000000UL
#define MIN_BELOW_LO         550000000UL
#define ULTRA_MAX_FREQ      1390000000UL
//#define DEFAULT_MAX_FREQ    527000000
#define HIGH_MIN_FREQ_MHZ   240
#define HIGH_MAX_FREQ_MHZ   959
#endif
#ifdef TINYSA4
typedef uint64_t freq_t;
#define FREQ_MULTIPLIER 100         // Multiplier of the 30MHz reference to get accurate frequency correction
#define VARIANT(X,Y) (Y)
#define DEFAULT_IF  ((freq_t)977400000)
#define DEFAULT_IF_PLUS  ((freq_t)1069500000)
extern uint16_t hw_if;
#define DEFAULT_SPUR_OFFSET ((freq_t)(actual_rbw_x10 > 3000 ? 1500000 : 1000000))
#define STATIC_DEFAULT_SPUR_OFFSET ((freq_t) 1500000)

#define MAX_LOW_OUTPUT_FREQ ((freq_t)1130000000)
#define HIGH_MIN_FREQ_MHZ   136// 825
#define HIGH_MAX_FREQ_MHZ   1130
#define MINIMUM_DIRECT_FREQ  823000000ULL
#define ULTRA_AUTO         10000000000ULL // 10GHz


//#define LOW_MAX_FREQ         800000000ULL
//#define MIN_BELOW_LO         550000000ULL   // not used????
#define DRIVE0_MAX_FREQ      600000000ULL            // LO drive 0
#define DRIVE1_MAX_FREQ     1200000000ULL           // LO drive 1
#define DRIVE2_MAX_FREQ     2100000000ULL           // LO drive 2
#define LOW_SHIFT_FREQ      2000000ULL              // shift IF to avoid zero Hz within IF

#define USE_SHIFT2_RBW  4000        // use shift2_level_offset if actual_rbw_x10 is larger then this.
#ifdef __NEW_SWITCHES__
#define DIRECT_START config.direct_start
#define DIRECT_STOP  config.direct_stop
#endif
#endif
/*
 * main.c
 */
#ifdef __SA__
#ifdef TINYSA4
#define POINTS_COUNT     450
#else
#define POINTS_COUNT     290
#endif
#ifdef TINYSA4
#define MARKER_COUNT    8
#define TRACES_MAX 4
#else
#define MARKER_COUNT    4
#define TRACES_MAX 3
#endif

#define TRACE_ACTUAL    0           // order linked to colors in palette!!!!!
#if TRACES_MAX == 3
#define TRACE_TEMP      (LCD_TRACE_3_COLOR - LCD_TRACE_1_COLOR)
#else
#define TRACE_TEMP      (LCD_TRACE_4_COLOR - LCD_TRACE_1_COLOR)
#define TRACE_STORED2    (LCD_TRACE_3_COLOR - LCD_TRACE_1_COLOR)
#endif
#define TRACE_STORED    (LCD_TRACE_2_COLOR - LCD_TRACE_1_COLOR)
//#define TRACE_AGE       3
#define TRACE_INVALID  -1

#define actual_t  measured[TRACE_ACTUAL]
#define stored_t  measured[TRACE_STORED]
#if TRACES_MAX == 4
#define stored2_t  measured[TRACE_STORED2]
#endif
#define temp_t    measured[TRACE_TEMP]
// #define age_t     measured[TRACE_AGE]

extern const char * const trc_channel_name[];

#ifdef TINYSA3
#define HALF_FREQ 0x80000000UL
 typedef int32_t long_t;
 extern bool has_esd;
 #define CORRECTION_POINTS  10       // Frequency dependent level correction table entries
#define CORRECTION_LOW_IN   0
#define CORRECTION_HIGH_IN  1
#define CORRECTION_LOW_OUT  2
#define CORRECTION_HIGH_OUT 3
// #define CORRECTION_LOW_OUT        2    // Must be same order as output path options!!!!!
 #define CORRECTION_SIZE    3
#endif
#ifdef TINYSA4
 typedef int64_t long_t;
#define HALF_FREQ 0x800000000000000ULL
 #define CORRECTION_POINTS  20       // Frequency dependent level correction table entries
 #define CORRECTION_LOW_IN   0
 #define CORRECTION_LNA      1
 #define CORRECTION_LOW_ULTRA 2
 #define CORRECTION_LNA_ULTRA 3
 #define CORRECTION_DIRECT         4
 #define CORRECTION_LNA_DIRECT     5
#define CORRECTION_HARM            6
#define CORRECTION_LNA_HARM        7
 #define CORRECTION_LOW_OUT        8    // Must be same order as output path options!!!!!
 #define CORRECTION_LOW_OUT_DIRECT 9
 #define CORRECTION_LOW_OUT_ADF    10
 #define CORRECTION_LOW_OUT_MIXER  11
 #define CORRECTION_SIZE           12

 extern freq_t ULTRA_MAX_FREQ;           // Start of harmonic mode
 extern freq_t MAX_LO_FREQ;
 extern freq_t MAX_ABOVE_IF_FREQ;           // Range to use for below IF
 extern freq_t MIN_BELOW_IF_FREQ;          // Range to use for below IF
 extern freq_t ULTRA_THRESHOLD;
 extern freq_t NORMAL_MAX_FREQ;
 extern int max2871;
 extern void set_freq_boundaries(void);
#endif
typedef float measurement_t[TRACES_MAX][POINTS_COUNT];
extern measurement_t measured;
#endif

extern freq_t minFreq;
extern freq_t maxFreq;
#define START_MIN minFreq
#define STOP_MAX maxFreq

extern const char TINYSA_VERSION[];
#ifdef TINYSA4
extern uint16_t hwid;
#endif
#define MAX_FREQ_TYPE 5
enum stimulus_type {
  ST_START=0, ST_STOP, ST_CENTER, ST_SPAN, ST_CW, ST_DUMMY      // Last is used in marker ops
};

void set_sweep_points(uint16_t points);
void update_frequencies(void);
void update_bands(void);
void set_sweep_frequency(int type, freq_t frequency);
freq_t get_sweep_frequency(int type);
void my_microsecond_delay(int t);
void my_veryfast_delay(int t);
float my_atof(const char *p);
freq_t my_atoui(const char *p);
int shell_printf(const char *fmt, ...);
int usage_printf(const char *fmt, ...);
void clear_backup(void);
const char *get_hw_version_text(void);

#ifdef __REMOTE_DESKTOP__
extern uint8_t remote_mouse_down;
extern uint8_t auto_capture;
typedef struct {
  char new_str[6];
  int16_t x;
  int16_t y;
  int16_t w;
  int16_t h;
} remote_region_t;
void send_region(remote_region_t *rd, uint8_t * buf, uint16_t size);
#endif

void set_marker_frequency(int m, freq_t f);
void set_marker_time(int m, float f);
void set_marker_index(int m, int16_t idx);
void toggle_sweep(void);
void resume_once(uint16_t c);
#ifdef TINYSA4
void set_deviation(int d);
void set_depth(int d);
extern int LO_harmonic;
#endif
void toggle_mute(void);
void toggle_pulse(void);
void toggle_draw_line(void);
void load_default_properties(void);

enum {
  AV_OFF, AV_MIN, AV_MAX_HOLD, AV_MAX_DECAY, AV_4, AV_16, AV_100, AV_QUASI, AV_TABLE, AV_DECONV
};

enum {
  M_LOW, M_HIGH, M_GENLOW, M_GENHIGH, M_ULTRA
};

enum {
  MO_NONE, MO_AM, MO_NFM,
#ifdef TINYSA4
  MO_NFM2, MO_NFM3,
#endif
  MO_WFM, MO_EXTERNAL, MO_MAX
};

#define MODE_OUTPUT(x)  ((x) == M_GENLOW || (x) == M_GENHIGH )
#ifdef __ULTRA__
#define MODE_INPUT(x)  ((x) == M_LOW || (x) == M_HIGH || (x) == M_ULTRA )
#else
#define MODE_INPUT(x)  ((x) == M_LOW || (x) == M_HIGH )
#endif
#define MODE_HIGH(x)  ((x) == M_HIGH || (x) == M_GENHIGH )
#define MODE_LOW(x)  ((x) == M_LOW || (x) == M_GENLOW )

#ifdef __SI4432__
#define SI4432_RX                          0
#define SI4432_LO                          1
#define MODE_SELECT(x) (MODE_HIGH(x) ? SI4432_LO : SI4432_RX)
#endif
#ifdef __SI4468__
// Not use mode
#define MODE_SELECT(x) (MODE_HIGH(x) ? 1 : 0)
#endif

#define SWEEP_ENABLE    0x01
#define SWEEP_ONCE      0x02
#define SWEEP_CALIBRATE 0x04
#define SWEEP_SELFTEST  0x08
#define SWEEP_REMOTE    0x10
#ifdef __LISTEN__
//#define SWEEP_LISTEN    0x20
//#define SWEEP_FACTORY    0x20
#endif
#define SWEEP_CALIBRATE_HARMONIC 0x40
#define SWEEP_UI_MODE   0x80

extern uint8_t sweep_mode;
extern uint8_t completed;
extern const char * const info_about[];

#ifdef TINYSA4
void toggle_extra_lna(void);
void set_extra_lna(int t);

enum { A_DAC, A_PWM };
void set_audio_mode(uint16_t new_mode);
void pwm_start(int f);
void pwm_stop(void);
#ifdef __BANDS__
void reset_band(void);
#endif
#endif

// ------------------------------- sa_core.c ----------------------------------


extern float level_min(void);
extern float level_max(void);
extern float level_range(void);
extern float channel_power[3];
extern float channel_power_watt[3];
extern const char * const unit_string[];
extern uint16_t vbwSteps;
#ifdef __ULTRA__
extern freq_t ultra_start;
//extern bool ultra;
#endif
#ifdef TINYSA4
extern float measured_noise_figure;
extern float *drive_dBm;
extern bool level_error;
extern bool depth_error;
#else
extern const int8_t drive_dBm [];
#endif
extern int force_signal_path;
extern int test_output_switch;
extern int test_output_drive;
extern int test_output_attenuate;
extern int test_path;
extern uint8_t signal_is_AM;
extern const uint32_t reffer_freq[];
extern freq_t minFreq;
extern freq_t maxFreq;
int level_is_calibrated(void);
void reset_settings(int);
void update_min_max_freq(void);
//void ui_process_touch(void);
void SetPowerGrid(int);
void SetRefLevel(float);
void set_refer_output(int);
void toggle_below_IF(void);
int get_refer_output(void);
void set_attenuation(float);
float get_attenuation(void);
float get_level(void);
void set_harmonic(int);
void store_trace(int f, int t);
void subtract_trace(int t, int f);
//extern int setting.harmonic;
int search_is_greater(void);
void set_auto_attenuation(void);
void set_auto_reflevel(bool);
int is_paused(void);
float set_actual_power(float);
void set_actual_correction_value(int current_curve,int current_curve_index, float local_actual_level);
extern const int to_calibrate[6];
void SetGenerate(int);
void set_RBW(uint32_t rbw_x10);
#ifdef __VBW__
void set_VBW(uint32_t vbw_x100);
#endif
void set_lo_drive(int d);
void set_rx_drive(int d);
void set_IF(int f);
void set_step_delay(int t);
void set_offset_delay(int t);
void set_repeat(int);
void set_level_sweep(float);
void set_level(float);
void set_sweep_time_us(uint32_t);
//extern int setting.repeat;
//extern int setting.rbw;
#ifdef __SPUR__
//extern int setting.spur;
void set_spur(int v);
void toggle_spur(void);
void toggle_mirror_masking(void);
#endif
void set_average(int t, int);
//extern int setting.average;
void  set_storage(void);
void  set_clear_storage(void);
void  set_subtract_storage(void);
void  toggle_normalize(int);
void set_level_meter_or_waterfall(void);
void disable_waterfall(void);
#ifdef __LEVEL_METER__
void disable_level_meter(void);
#endif
void set_mode(int);
int GetMode(void);
void set_reflevel(float);
void user_set_reflevel(float);
#define REFLEVEL_MAX 9999.0
#define REFLEVEL_MIN    1.0e-12
void set_scale(float);
void user_set_scale(float);
void AllDirty(void);
void MenuDirty(void);
void toggle_LNA(void);
void toggle_AGC(void);
void redrawHisto(void);
void self_test(int);
void set_decay(int);
void set_attack(int);
void set_noise(int);
void toggle_tracking_output(void);
extern int32_t frequencyExtra;
void set_modulation(int);
void set_modulation_frequency(float);
int search_maximum(int m, freq_t center, int span);
//extern int setting.modulation;
void set_measurement(int);
// extern int settingSpeed;
//extern int setting.step_delay;
void sweep_remote(void);
void calculate_step_delay(void);
extern int generic_option_cmd( const char *cmd, const char *cmd_list, int argc, char *argv);
extern bool global_abort;
#ifdef __ULTRA__
void toggle_ultra(void);
void enable_ultra(int);
#endif
#ifdef TINYSA4
void clear_frequency_cache(void);
#ifndef __NEW_SWITCHES__
void toggle_high_out_adf4350(void);
extern int high_out_adf4350;
#endif
int set_actual_freq(freq_t);
void set_jump_freq(freq_t a, freq_t b, freq_t c);
int set_freq_corr(int);
void set_IF2(int f);
void set_R(int f);
extern void set_modulo(uint32_t f);
extern uint32_t local_modulo;
extern void fill_spur_table(void);
extern float low_out_offset(void);
extern float high_out_offset(void);
#define LOW_OUT_OFFSET low_out_offset()
#define HIGH_OUT_OFFSET high_out_offset()
extern bool debug_avoid;
extern bool progress_bar;

extern void toggle_debug_avoid(void);
extern float log_averaging_correction;
#else
void set_10mhz(freq_t);
#define LOW_OUT_OFFSET config.low_level_output_offset
#define HIGH_OUT_OFFSET config.high_level_output_offset
#endif
#ifdef __ULTRA__
extern bool debug_level;
extern void toggle_debug_level(void);
extern bool debug_spur;
extern void toggle_debug_spur(void);
#endif
#ifdef __AUDIO__
/*
 * dsp.c
 */
// 5ms @ 48kHz
#define AUDIO_BUFFER_LEN 96

extern int16_t rx_buffer[AUDIO_BUFFER_LEN * 2];

#define STATE_LEN 32
#define SAMPLE_LEN 48

#ifdef ENABLED_DUMP
extern int16_t ref_buf[];
extern int16_t samp_buf[];
#endif
#endif
#ifdef __VNA__
void dsp_process(int16_t *src, size_t len);
void reset_dsp_accumerator(void);
void calculate_gamma(float *gamma);
void fetch_amplitude(float *gamma);
void fetch_amplitude_ref(float *gamma);
#endif

#ifdef __AUDIO__
/*
 * tlv320aic3204.c
 */

extern void tlv320aic3204_init(void);
extern void tlv320aic3204_set_gain(int lgain, int rgain);
extern void tlv320aic3204_select(int channel);

#endif

/*
 * plot.c
 */

// Offset of plot area
#define OFFSETX 30
#define OFFSETY 0

#define NGRIDY 10
// GRIDX calculated depends from frequency span
#ifdef __SCROLL__
extern  uint16_t _grid_y;
#define GRIDY  _grid_y
extern uint16_t graph_bottom;
#ifdef TINYSA4
#define SUPER_WATERFALL  90
#define BIG_WATERFALL   180
#define SMALL_WATERFALL 240
#define BIG_NUMBER_SPACE BIG_WATERFALL
#else
#define BIG_WATERFALL   90
#define SMALL_WATERFALL 180
#define BIG_NUMBER_SPACE SMALL_WATERFALL
#endif
#define NO_WATERFALL    CHART_BOTTOM
#define CHART_BOTTOM   (LCD_HEIGHT-10)
#define SCROLL_GRIDY      (HEIGHT_SCROLL / NGRIDY)
#define NOSCROLL_GRIDY    (CHART_BOTTOM / NGRIDY)
#else
#define GRIDY             (CHART_BOTTOM / NGRIDY)
#endif

#define SD_CARD_START   (LCD_HEIGHT-40-20)
#define BATTERY_START   (LCD_HEIGHT-40)

#define WIDTH  (LCD_WIDTH - OFFSETX)
#define HEIGHT (GRIDY*NGRIDY)

#define FREQUENCIES_XPOS1 OFFSETX
#define FREQUENCIES_XPOS2 (LCD_WIDTH-120)
#define FREQUENCIES_YPOS  (LCD_HEIGHT-8)

//
#define CELLOFFSETX 0
#define AREA_WIDTH_NORMAL  (CELLOFFSETX + WIDTH)
#define AREA_HEIGHT_NORMAL (              HEIGHT)

#define GRID_X_TEXT       (AREA_WIDTH_NORMAL - 7*5)

// Marker start drag distance (can be bigger for various display resolution)
#define MARKER_PICKUP_DISTANCE 20
// Trigger line start drag distance (can be bigger for various display resolution)
#define DRAG_TRIGGER_DISTANCE  10

// Smith/polar chart
//#define P_CENTER_X (CELLOFFSETX + WIDTH/2)
//#define P_CENTER_Y (HEIGHT/2)
//#define P_RADIUS   (HEIGHT/2)

// Menu Button
// Maximum menu buttons count
#ifdef TINYSA4
#define MENU_BUTTON_MAX        16
#define MENU_BUTTON_MIN         9
#else
#define MENU_BUTTON_MAX        16
#define MENU_BUTTON_MIN         7
#endif
#define MENU_BUTTON_WIDTH      80
#define MENU_BUTTON_BORDER      1
#define KEYBOARD_BUTTON_BORDER  2
#define FORM_BUTTON_BORDER      2

#define MENU_BUTTON_HEIGHT_N(n)   (LCD_HEIGHT/(n)-1)

#define BROWSER_BUTTON_BORDER         1
// Browser window settings
#define FILES_COLUMNS               (LCD_WIDTH/160)                                // columns in browser
#define FILES_ROWS                   10                                            // rows in browser
#define FILES_PER_PAGE              (FILES_COLUMNS*FILES_ROWS)                     // FILES_ROWS * FILES_COLUMNS
#define FILE_BOTTOM_HEIGHT           20                                            // Height of bottom buttons (< > X)
#define FILE_BUTTON_HEIGHT          ((LCD_HEIGHT - FILE_BOTTOM_HEIGHT)/FILES_ROWS) // Height of file buttons

// Define message box width
#ifdef TINYSA4
#define MESSAGE_BOX_WIDTH     300
#else
#define MESSAGE_BOX_WIDTH     180
#endif
// Form button (at center screen better be less LCD_WIDTH - 2*OFFSETX)
#define MENU_FORM_WIDTH    (LCD_WIDTH - 2*OFFSETX)

// Num Input height at bottom
#define NUM_INPUT_HEIGHT   32

extern uint16_t area_width;
extern uint16_t area_height;

// Define marker size (can be 0 or 1)
#ifdef TINYSA3
#define _MARKER_SIZE_         0
#endif
#ifdef TINYSA4
#define _MARKER_SIZE_         1
#endif
// font
extern const uint8_t x5x7_bits [];
extern const uint8_t x7x11b_bits [];
extern const uint8_t x10x14_bits[];
extern const uint8_t numfont16x22[];

#define FONT_SMALL            0
#define FONT_NORMAL           1

#define FONT_START_CHAR    0x16
#define FONT_MAX_WIDTH        7
#define FONT_WIDTH            5
#define FONT_GET_HEIGHT       7
#define FONT_STR_HEIGHT       8
#define FONT_GET_DATA(ch)    (  &x5x7_bits[(ch-FONT_START_CHAR)*FONT_GET_HEIGHT])
#define FONT_GET_WIDTH(ch)   (8-(x5x7_bits[(ch-FONT_START_CHAR)*FONT_GET_HEIGHT]&7))

#define bFONT_START_CHAR   0x16
#define bFONT_MAX_WIDTH       8
#define bFONT_WIDTH           7
#define bFONT_GET_HEIGHT     11
#define bFONT_STR_HEIGHT     11
#define bFONT_GET_DATA(ch)   (  &x7x11b_bits[(ch-bFONT_START_CHAR)*bFONT_GET_HEIGHT])
#define bFONT_GET_WIDTH(ch)  (8-(x7x11b_bits[(ch-bFONT_START_CHAR)*bFONT_GET_HEIGHT]&7))

#ifdef __NICE_BIG_FONT__
#define wFONT_START_CHAR   0x16
#define wFONT_MAX_WIDTH      12
#define wFONT_GET_HEIGHT     14
#define wFONT_STR_HEIGHT     16
#define wFONT_GET_DATA(ch)   (   &x10x14_bits[(ch-wFONT_START_CHAR)*2*wFONT_GET_HEIGHT  ])
#define wFONT_GET_WIDTH(ch)  (14-(x10x14_bits[(ch-wFONT_START_CHAR)*2*wFONT_GET_HEIGHT+1]&0x7))
#else
#define wFONT_MAX_WIDTH      12
#define wFONT_GET_HEIGHT     14
#endif

#define NUM_FONT_GET_WIDTH      16
#define NUM_FONT_GET_HEIGHT     22
#define NUM_FONT_GET_DATA(ch)   (&numfont16x22[ch*2*NUM_FONT_GET_HEIGHT])

#define KP_WIDTH                  (LCD_WIDTH / 4)                                  // numeric keypad button width
#define KP_HEIGHT                 ((LCD_HEIGHT - NUM_INPUT_HEIGHT) / 4)            // numeric keypad button height
#define KP_X_OFFSET               0                                                // numeric keypad X offset
#define KP_Y_OFFSET               0                                                // numeric keypad Y offset
#define KPF_WIDTH                 (LCD_WIDTH / 10)                                 // text keypad button width
#define KPF_HEIGHT                KPF_WIDTH                                        // text keypad button height
#define KPF_X_OFFSET              0                                                // text keypad X offset
#define KPF_Y_OFFSET              (LCD_HEIGHT - NUM_INPUT_HEIGHT - 4 * KPF_HEIGHT) // text keypad Y offset

#define S_ENTER    "\026"  // 0x16
#define S_DELTA    "\027"  // 0x17
#define S_SARROW   "\030"  // 0x18
#define S_INFINITY "\031"  // 0x19
#define S_LARROW   "\032"  // 0x1A
#define S_RARROW   "\033"  // 0x1B
#define S_PI       "\034"  // 0x1C
#define S_MICRO    "\035"  // 0x1D
#define S_OHM      "\036"  // 0x1E
#define S_DEGREE   "\037"  // 0x1F

#define C_ENTER     0x16   // 0x16
#define C_LARROW    0x1A   // 0x1A
#define C_RARROW    0x1B   // 0x1B

// String prefix for select font size (use not printable chars)
#define  FONT_s     "\001"
#define _FONT_s     1
// bold as default
#define  FONT_b     ""
#define _FONT_b     2
#define  FONT_w     "\003"
#define _FONT_w     3

// Max palette indexes in config
#define MAX_PALETTE     32

// trace 
#define MAX_TRACE_TYPE 12
enum trace_type {
  TRC_LOGMAG=0, TRC_PHASE, TRC_DELAY, TRC_SMITH, TRC_POLAR, TRC_LINEAR, TRC_SWR, TRC_REAL, TRC_IMAG, TRC_R, TRC_X, TRC_OFF
};
// Mask for define rectangular plot
#define RECTANGULAR_GRID_MASK ((1<<TRC_LOGMAG)|(1<<TRC_PHASE)|(1<<TRC_DELAY)|(1<<TRC_LINEAR)|(1<<TRC_SWR)|(1<<TRC_REAL)|(1<<TRC_IMAG)|(1<<TRC_R)|(1<<TRC_X))

// LOGMAG: SCALE, REFPOS, REFVAL
// PHASE: SCALE, REFPOS, REFVAL
// DELAY: SCALE, REFPOS, REFVAL
// SMITH: SCALE, <REFPOS>, <REFVAL>
// LINMAG: SCALE, REFPOS, REFVAL
// SWR: SCALE, REFPOS, REFVAL

// Electrical Delay
// Phase

#define MAX_UNIT_TYPE 7     // Index of U_DBC
enum unit_type {
  U_DBM=0, U_DBMV, U_DBUV, U_RAW, U_VOLT, U_VPP, U_WATT, U_DBC //  dBc only for displaying delta marker info
};

#define UNIT_IS_LINEAR(T) ( T >= U_VOLT ? true : false)
#define UNIT_IS_LOG(T) ( T >= U_VOLT ? false : true)

float value(float);
float index_to_value(const int i);
float marker_to_value(const int i);

#define FREQ_MODE_START_STOP    0x0
#define FREQ_MODE_CENTER_SPAN   0x1
//#define FREQ_MODE_DOTTED_GRID   0x2

// Connection flag
#define _MODE_CONNECTION_MASK  0x04
#define _MODE_SERIAL           0x04
#define _MODE_USB              0x00
// don't save state
#define _MODE_DONT_SAVE_STATE   0x08
// auto name
#define _MODE_AUTO_FILENAME    0x10
#define _MODE_MHZ_CSV          0x20

#pragma pack(push, 4)
typedef struct config {
  int32_t magic;
  uint32_t deviceid;
  uint16_t lcd_palette[MAX_PALETTE];
  int16_t  touch_cal[4];
  uint32_t _serial_speed;
  uint16_t dac_value;
  uint16_t vbat_offset;
  int16_t    cor_am;
  float low_level_offset;
  float high_level_offset;
  float low_level_output_offset;
  float high_level_output_offset;
  float receive_switch_offset;
#ifdef TINYSA4
  float out_switch_offset;
  float lna_level_offset;
  float harmonic_level_offset;
  float harmonic_lna_level_offset;
  float shift_level_offset;
  float shift1_level_offset;
  float shift2_level_offset;
  float shift3_level_offset;
  float drive1_level_offset;
  float drive2_level_offset;
  float drive3_level_offset;
  float direct_level_offset;
  float ultra_level_offset;
  float direct_lna_level_offset;
  float ultra_lna_level_offset;
  float adf_level_offset;
  float direct_level_output_offset;
#endif
#ifdef __NOISE_FIGURE__
  float noise_figure;
#endif
  float  correction_value[CORRECTION_SIZE][CORRECTION_POINTS];
  freq_t correction_frequency[CORRECTION_SIZE][CORRECTION_POINTS];
#ifdef TINYSA4
  freq_t  setting_frequency_30mhz;
#else
  freq_t  setting_frequency_10mhz;
#endif

  uint16_t gridlines;
  uint16_t hambands;
#ifdef TINYSA4
  freq_t frequency_IF1;
  freq_t frequency_IF2;
#endif
#ifdef __ULTRA__
  freq_t ultra_start;
  freq_t harmonic_start;
  freq_t direct_start;
  freq_t direct_stop;
  freq_t overclock;
  int8_t    ultra;
#endif
  uint8_t   input_is_calibrated;
  uint8_t   output_is_calibrated;
  uint8_t   _mode;
  int8_t    cor_wfm;
  int8_t    cor_nfm;
#ifdef TINYSA4
  int8_t    cor_nfm2;
  int8_t    cor_nfm3;
#endif
  uint8_t  _brightness;
#ifndef __NEW_SWITCHES__
  uint8_t high_out_adf4350;
#endif
  uint8_t flip;
#ifdef __ULTRA__
  uint8_t    direct;
#endif
#ifdef TINYSA4
  uint8_t hide_21MHz;
#endif
  float sweep_voltage;
  float switch_offset;
  int16_t   ext_zero_level;
  uint32_t    dummy;
//  uint8_t _reserved[22];
  freq_t checksum;
} config_t;
#pragma pack(pop)

extern config_t config;
//#define settingLevelOffset config.level_offset
float get_level_offset(void);

extern uint8_t in_selftest;
extern int display_test(void);
extern void clear_marker_cache(void);

//
// Shell config functions and macros
// Serial connect definitions not used if Serial mode disabled
void shell_update_speed(void);
void shell_reset_console(void);
int  shell_serial_printf(const char *fmt, ...);
void shell_executeCMDLine(char *line);

// marker
enum {
  M_NORMAL=0,M_REFERENCE=1, M_DELTA=2, M_NOISE=4, M_STORED=8, M_AVER=16, M_TRACKING=32, M_DELETE=64 // Tracking must be last.
};

enum {
  M_DISABLED = 0, M_ENABLED = 1
};


// Flags/macros for enable/disable traces
#define TRACE_ACTUAL_FLAG (1<<(TRACE_ACTUAL))
#define TRACE_STORED_FLAG (1<<(TRACE_STORED))
#define TRACE_TEMP_FLAG   (1<<(TRACE_TEMP))
#ifdef TINYSA4
#define TRACE_STORED2_FLAG (1<<(TRACE_STORED2))
#endif
#define TRACE_ENABLE(t_mask)  {setting._traces|= (t_mask);}
#define TRACE_DISABLE(t_mask) {setting._traces&=~(t_mask);}

#define IS_TRACES_ENABLED(t_mask) (setting._traces&(t_mask))
#define IS_TRACE_ENABLE(t)        (setting._traces&(1<<(t)))
#define IS_TRACE_DISABLE(t)      !(setting._traces&(1<<(t)))

// Enable trace for show only after sweep complete (disable it at call)
void enableTracesAtComplete(uint8_t mask);

typedef struct {
  uint8_t mtype;
  uint8_t enabled;
  uint8_t ref;
  uint8_t trace;
  int16_t index;
  freq_t frequency;
} marker_t;

#ifdef __LIMITS__
#ifdef TINYSA4
#define LIMITS_MAX  8
#else
#define LIMITS_MAX  6
#endif
#define REFERENCE_MAX TRACES_MAX
typedef struct {
  uint8_t enabled;
  float level;
  freq_t frequency;
  int16_t index;
} limit_t;
extern uint8_t active_limit;
extern void limits_update(void);
#endif

#define MARKERS_MAX MARKER_COUNT
#define MARKER_INVALID -1

extern int8_t previous_marker;
extern int8_t marker_tracking;

void plot_init(void);
void update_grid(void);
void request_to_redraw_grid(void);
void redraw_frame(void);
//void redraw_all(void);
void request_to_draw_cells_behind_menu(void);
void request_to_draw_cells_behind_numeric_input(void);
void redraw_marker(int marker);
void redraw_trigger(int band);
void markmap_all_markers(void);
void plot_into_index(measurement_t measured);
void draw_frequencies(void);
void draw_all(bool flush);

void draw_cal_status(void);

//void markmap_all_markers(void);

int distance_to_index(int8_t t, uint16_t idx, int16_t x, int16_t y);
int search_nearest_index(int x, int y, int t);
int search_nearest_trigger(int x, int y);

float pos_to_trigger_value(int y);
bool is_multiband_trigger(void);

int marker_search_max(int m);
int marker_search_left_max(int m);
int marker_search_right_max(int m);
int marker_search_left_min(int m);
int marker_search_right_min(int m);
void markers_reset(void);

// _request flag for update screen
#define REDRAW_CELLS      (1<<0)
#define REDRAW_FREQUENCY  (1<<1)
#define REDRAW_CAL_STATUS (1<<2)
#define REDRAW_MARKER     (1<<3)
#define REDRAW_BATTERY    (1<<4)
#define REDRAW_AREA       (1<<5)
#define REDRAW_TRIGGER    (1<<6)
#define REDRAW_INBETWEEN  (1<<7)
extern  uint16_t redraw_request;

/*
 * ili9341.c
 */
// Set display buffers count for cell render (if use 2 and DMA, possible send data and prepare new in some time)

#ifdef __USE_DISPLAY_DMA__
// Cell size = sizeof(spi_buffer), but need wait while cell data send to LCD
//#define DISPLAY_CELL_BUFFER_COUNT     1
// Cell size = sizeof(spi_buffer)/2, while one cell send to LCD by DMA, CPU render to next cell
#define DISPLAY_CELL_BUFFER_COUNT     2
#else
// Always one if no DMA mode
#define DISPLAY_CELL_BUFFER_COUNT     1
#endif

// One pixel size
typedef uint16_t pixel_t;

#define CELLWIDTH  (64/DISPLAY_CELL_BUFFER_COUNT)
#define CELLHEIGHT (32)

// Define size of screen buffer in pixels (one pixel 16bit size)
#define SPI_BUFFER_SIZE     (CELLWIDTH * CELLHEIGHT * DISPLAY_CELL_BUFFER_COUNT)

// SPI bus revert byte order
// 16-bit gggBBBbb RRRrrGGG
#define RGB565(r,g,b)  ( (((g)&0x1c)<<11) | (((b)&0xf8)<<5) | ((r)&0xf8) | (((g)&0xe0)>>5) )
#define RGBHEX(hex) ( (((hex)&0x001c00)<<3) | (((hex)&0x0000f8)<<5) | (((hex)&0xf80000)>>16) | (((hex)&0x00e000)>>13) )
#define HEXRGB(hex) ( (((hex)>>3)&0x001c00) | (((hex)>>5)&0x0000f8) | (((hex)<<16)&0xf80000) | (((hex)<<13)&0x00e000) )

// Define LCD display driver and screen size
#ifdef TINYSA4
#define LCD_DRIVER_ST7796S
#define LCD_WIDTH                   480
#define LCD_HEIGHT                  320
#else
#define LCD_DRIVER_ILI9341
#define LCD_WIDTH                   320
#define LCD_HEIGHT                  240
#endif

// Default LCD brightness if display support it
#define DEFAULT_BRIGHTNESS       70

#define LCD_BG_COLOR             0
#define LCD_FG_COLOR             1
#define LCD_GRID_COLOR           2
#define LCD_MENU_COLOR           3
#define LCD_MENU_TEXT_COLOR      4
#define LCD_MENU_ACTIVE_COLOR    5
#define LCD_TRACE_1_COLOR        6
#define LCD_TRACE_2_COLOR        7
#define LCD_TRACE_3_COLOR        8
#define LCD_TRACE_4_COLOR        9
#define LCD_NORMAL_BAT_COLOR    10
#define LCD_LOW_BAT_COLOR       11
#define LCD_TRIGGER_COLOR       12
#define LCD_RISE_EDGE_COLOR     13
#define LCD_FALLEN_EDGE_COLOR   14
#define LCD_SWEEP_LINE_COLOR    15
#define LCD_BW_TEXT_COLOR       16
#define LCD_INPUT_TEXT_COLOR    17
#define LCD_INPUT_BG_COLOR      18
#define LCD_BRIGHT_COLOR_BLUE   19
#define LCD_BRIGHT_COLOR_RED    20
#define LCD_BRIGHT_COLOR_GREEN  21
#define LCD_DARK_GREY           22
#define LCD_LIGHT_GREY          23
#define LCD_HAM_COLOR           24
#define LCD_GRID_VALUE_COLOR    25
#define LCD_M_REFERENCE         26
#define LCD_M_DELTA             27
#define LCD_M_NOISE             28
#define LCD_M_DEFAULT           29

#if TRACES_MAX == 3
#define LCD_DEFAULT_PALETTE {\
[LCD_BG_COLOR         ] = RGB565(  0,  0,  0), \
[LCD_FG_COLOR         ] = RGB565(255,255,255), \
[LCD_GRID_COLOR       ] = RGB565(128,128,128), \
[LCD_MENU_COLOR       ] = RGB565(230,230,230), \
[LCD_MENU_TEXT_COLOR  ] = RGB565(  0,  0,  0), \
[LCD_MENU_ACTIVE_COLOR] = RGB565(210,210,210), \
[LCD_TRACE_1_COLOR    ] = RGB565(255,255,  0), \
[LCD_TRACE_2_COLOR    ] = RGB565( 64,255, 64), \
[LCD_TRACE_3_COLOR    ] = RGB565(255, 64, 64), \
[LCD_TRACE_4_COLOR    ] = RGB565(255,  0,255), \
[LCD_NORMAL_BAT_COLOR ] = RGB565( 31,227,  0), \
[LCD_LOW_BAT_COLOR    ] = RGB565(255,  0,  0), \
[LCD_TRIGGER_COLOR    ] = RGB565(  0,  0,255), \
[LCD_RISE_EDGE_COLOR  ] = RGB565(255,255,255), \
[LCD_FALLEN_EDGE_COLOR] = RGB565(128,128,128), \
[LCD_SWEEP_LINE_COLOR ] = RGB565(  0,255,  0), \
[LCD_BW_TEXT_COLOR    ] = RGB565(128,128,128), \
[LCD_INPUT_TEXT_COLOR ] = RGB565(  0,  0,  0), \
[LCD_INPUT_BG_COLOR   ] = RGB565(255,255,255), \
[LCD_BRIGHT_COLOR_BLUE] = RGB565(  0,  0,255), \
[LCD_BRIGHT_COLOR_RED ] = RGB565(255,128,128), \
[LCD_BRIGHT_COLOR_GREEN]= RGB565(  0,255,  0), \
[LCD_DARK_GREY        ] = RGB565(140,140,140), \
[LCD_LIGHT_GREY       ] = RGB565(220,220,220), \
[LCD_HAM_COLOR        ] = RGB565( 80, 80, 80), \
[LCD_GRID_VALUE_COLOR ] = RGB565(196,196,196), \
[LCD_M_REFERENCE      ] = RGB565(255,255,255), \
[LCD_M_DELTA          ] = RGB565(  0,255,  0), \
[LCD_M_NOISE          ] = RGB565(  0,255,255), \
[LCD_M_DEFAULT        ] = RGB565(255,255,  0), \
}
#else
#define LCD_DEFAULT_PALETTE {\
[LCD_BG_COLOR         ] = RGB565(  0,  0,  0), \
[LCD_FG_COLOR         ] = RGB565(255,255,255), \
[LCD_GRID_COLOR       ] = RGB565(128,128,128), \
[LCD_MENU_COLOR       ] = RGB565(230,230,230), \
[LCD_MENU_TEXT_COLOR  ] = RGB565(  0,  0,  0), \
[LCD_MENU_ACTIVE_COLOR] = RGB565(210,210,210), \
[LCD_TRACE_1_COLOR    ] = RGB565(255,255,  0), \
[LCD_TRACE_2_COLOR    ] = RGB565( 64,255, 64), \
[LCD_TRACE_3_COLOR    ] = RGB565(255,  0,255), \
[LCD_TRACE_4_COLOR    ] = RGB565(255, 64, 64), \
[LCD_NORMAL_BAT_COLOR ] = RGB565( 31,227,  0), \
[LCD_LOW_BAT_COLOR    ] = RGB565(255,  0,  0), \
[LCD_TRIGGER_COLOR    ] = RGB565(  0,  0,255), \
[LCD_RISE_EDGE_COLOR  ] = RGB565(255,255,255), \
[LCD_FALLEN_EDGE_COLOR] = RGB565(128,128,128), \
[LCD_SWEEP_LINE_COLOR ] = RGB565(  0,255,  0), \
[LCD_BW_TEXT_COLOR    ] = RGB565(128,128,128), \
[LCD_INPUT_TEXT_COLOR ] = RGB565(  0,  0,  0), \
[LCD_INPUT_BG_COLOR   ] = RGB565(255,255,255), \
[LCD_BRIGHT_COLOR_BLUE] = RGB565(  0,  0,255), \
[LCD_BRIGHT_COLOR_RED ] = RGB565(255,128,128), \
[LCD_BRIGHT_COLOR_GREEN]= RGB565(  0,255,  0), \
[LCD_DARK_GREY        ] = RGB565(140,140,140), \
[LCD_LIGHT_GREY       ] = RGB565(220,220,220), \
[LCD_HAM_COLOR        ] = RGB565( 40, 40, 40), \
[LCD_GRID_VALUE_COLOR ] = RGB565(196,196,196), \
[LCD_M_REFERENCE      ] = RGB565(255,255,255), \
[LCD_M_DELTA          ] = RGB565(  0,255,  0), \
[LCD_M_NOISE          ] = RGB565(  0,255,255), \
[LCD_M_DEFAULT        ] = RGB565(255,255,  0), \
}
#endif

#define GET_PALTETTE_COLOR(idx)  config.lcd_palette[idx]

extern uint16_t foreground_color;
extern uint16_t background_color;

extern pixel_t spi_buffer[SPI_BUFFER_SIZE];

// Used for easy define big Bitmap as 0bXXXXXXXXX image
#define _BMP8(d)                                                        ((d)&0xFF)
#define _BMP16(d)                                      (((d)>>8)&0xFF), ((d)&0xFF)
#define _BMP24(d)                    (((d)>>16)&0xFF), (((d)>>8)&0xFF), ((d)&0xFF)
#define _BMP32(d)  (((d)>>24)&0xFF), (((d)>>16)&0xFF), (((d)>>8)&0xFF), ((d)&0xFF)

void ili9341_init(void);
void ili9341_test(int mode);
void ili9341_bulk(int x, int y, int w, int h);              // send data to display, in DMA mode use it, but wait DMA complete
void ili9341_fill(int x, int y, int w, int h);
void ili9341_flip(bool flip);

// Double buffer mode parser
#if DISPLAY_CELL_BUFFER_COUNT == 1
#define ili9341_get_cell_buffer()             spi_buffer
#define ili9341_bulk_continue                 ili9341_bulk
#define ili9341_bulk_finish()                 {}
#else
pixel_t *ili9341_get_cell_buffer(void);                     // get buffer for cell render
void ili9341_bulk_continue(int x, int y, int w, int h);     // send data to display, in DMA mode use it, no wait DMA complete
void ili9341_bulk_finish(void);                             // wait DMA complete (need call at end)
#endif

void ili9341_set_foreground(uint16_t fg_idx);
void ili9341_set_background(uint16_t bg_idx);

void ili9341_clear_screen(void);
void ili9341_blitBitmap(int x, int y, int width, int height, const uint8_t *bitmap);
void ili9341_drawchar(uint8_t ch, int x, int y);
void ili9341_drawstring(const char *str, int x, int y);
void ili9341_drawstring_7x13(const char *str, int x, int y);
void ili9341_drawstring_10x14(const char *str, int x, int y);
void lcd_set_font(int type);
int  lcd_printf(int16_t x, int16_t y, const char *fmt, ...);
void ili9341_drawstringV(const char *str, int x, int y);
int  ili9341_drawchar_size(uint8_t ch, int x, int y, uint8_t size, int x_max);
int  ili9341_drawstring_size(const char *str, int x, int y, uint8_t size, int x_max);
void ili9341_drawfont(uint8_t ch, int x, int y);
void ili9341_read_memory(int x, int y, int w, int h, uint16_t* out);
void ili9341_line(int x0, int y0, int x1, int y1);
void ili9341_define_scroll(uint16_t tfa, uint16_t bfa);
void ili9341_scroll(uint16_t start);
void show_version(void);
void lcd_setBrightness(uint16_t b);
void spi_init(void);

/*
 * flash.c
 */

#ifdef __BANDS__
#define BANDS_MAX   8
#define BAND_NAME_SIZE  9
typedef struct {
  char      name[BAND_NAME_SIZE];
  bool      enabled;
  freq_t    start;
  freq_t    end;
  float     level;
  int       start_index;
  int       stop_index;
} band_t;
#endif


typedef struct setting
{
  uint32_t magic;
  bool auto_reflevel;          // bool
  bool auto_attenuation;       // bool
  bool mirror_masking;         // bool
//  bool show_stored;            // bool
  bool tracking_output;        // bool
  bool mute;                   // bool
  bool auto_IF;                // bool
  bool sweep;                  // bool
  bool pulse;                  // bool
  bool stored[TRACES_MAX];     // enum
  bool normalized[TRACES_MAX];     // enum
#ifdef __BANDS__
  band_t bands[BANDS_MAX];
#endif

  uint8_t mode;                // enum
  uint8_t below_IF;            // enum
  uint8_t unit;                // enum
  uint8_t agc;                 // enum
  uint8_t lna;                 // enum
  uint8_t modulation;          // enum
  uint8_t trigger;             // enum
  uint8_t trigger_mode;        // enum
  uint8_t trigger_direction;   // enum
#ifdef __BEEP__
  uint8_t trigger_beep;
#endif
#ifdef TINYSA4
  uint8_t trigger_auto_save;
#endif
  uint8_t step_delay_mode;     // enum
  uint8_t waterfall;           // enum
#ifdef __LEVEL_METER__
  uint8_t level_meter;         // enum
#endif
  uint8_t average[TRACES_MAX]; // enum
  uint8_t subtract[TRACES_MAX];// index
  uint8_t measurement;         // enum
  uint8_t spur_removal;        // enum
  uint8_t disable_correction;
  int8_t normalized_trace;
  uint8_t listen;

  int8_t  tracking;            // -1...1 Can NOT convert to bool!!!!!!
  uint8_t atten_step;          //  0...1 !!! need convert to bool
  int8_t _active_marker;       // -1...MARKER_MAX
  uint8_t unit_scale_index;    // table index
  uint8_t noise;               // 2...50
  uint8_t lo_drive;            // 0-3 , 3dB steps
  uint8_t rx_drive;            // 0-15 , 7=+20dBm, 3dB steps
  uint8_t test;                // current test number
  uint8_t harmonic;            // used harmonic number 1...5
  uint8_t fast_speedup;        // 0 - 20
  uint8_t faster_speedup;      // 0 - 20
  uint8_t  _traces;            // enabled traces flags
  uint8_t   draw_line;         // uses the trigger level setting
#ifdef TINYSA4
  uint8_t   lock_display;
  uint8_t   jog_jump;
#endif
#ifdef __BANDS__
  uint8_t multi_band;
  uint8_t multi_trace;
#endif
#ifdef __TRIGGER_TRACE__
  uint8_t trigger_trace;
#endif
  uint16_t repeat;              // 1...100
  uint16_t linearity_step;     // range equal POINTS_COUNT
  uint16_t _sweep_points;
  int16_t attenuate_x2;        // 0...60 !!! in calculation can be < 0

  uint16_t step_delay;         // KM_SAMPLETIME   250...10000, 0=auto
  uint16_t offset_delay;       // KM_OFFSET_DELAY 250...10000, 0=auto

  uint16_t freq_mode;           //  0...1!!! need convert to bool or bit field
  int16_t  refer;               // -1 disabled

#ifdef TINYSA4
  uint16_t modulation_depth_x100;      // AM (30% - 100%) multiplied by 100
  uint16_t modulation_deviation_div100;  // FM (2.5kHz to 100kHz) divided by 100
#endif
  int decay;                      // KM_DECAY   < 1000000
  int attack;                     // KM_ATTACK  <   20000

  int32_t slider_position;
  freq_t  slider_span;

  uint32_t rbw_x10;
  uint32_t vbw_x100;
  uint32_t scan_after_dirty[TRACES_MAX];

  float modulation_frequency;  // 50...6000
  float reflevel;
  float scale;
  float external_gain;
  float trigger_level;
  float level;
  float level_sweep;

  float unit_scale;
  float normalize_level;     // Level to set normalize to, zero if not doing anything

  freq_t frequency_step;
  freq_t frequency0;
  freq_t frequency1;
  freq_t frequency_var;
  freq_t frequency_IF;
  freq_t frequency_offset;
#define FREQUENCY_SHIFT ((freq_t)100000000)   // 100MHz upconversion maximum
  float trace_scale;
  float trace_refpos;
  marker_t _markers[MARKERS_MAX];
#ifdef __LIMITS__
  limit_t limits[REFERENCE_MAX][LIMITS_MAX];
#endif
  systime_t sweep_time_us;
  systime_t measure_sweep_time_us;
  systime_t actual_sweep_time_us;
  systime_t additional_step_delay_us;

  uint32_t trigger_grid;

//  freq_t  *correction_frequency;
//  float   *correction_value;

#ifdef __ULTRA__
  uint8_t ultra;    // enum ??
#endif
#ifdef TINYSA4
  bool    extra_lna;
  int R;            // KM_R
  int32_t exp_aver;
  bool increased_R;
  bool mixer_output;
  uint32_t interval;
#define PRESET_NAME_LENGTH  10
  char preset_name[PRESET_NAME_LENGTH];
#endif
  int64_t test_argument;            // used for tests
  uint32_t checksum;            // must be last and at 4 byte boundary
}setting_t;

extern setting_t setting;

void reset_settings(int m);

void set_trace_scale(float scale);
void set_trace_refpos(float refpos);
#define get_trace_scale()  setting.trace_scale
#define get_trace_refpos() setting.trace_refpos

#define S_IS_AUTO(x) ((x)&2)
#define S_STATE(X) ((X)&1)
enum { S_OFF=0, S_ON=1, S_AUTO_OFF=2, S_AUTO_ON=3 };

enum { SD_NORMAL, SD_PRECISE, SD_FAST, SD_NOISE_SOURCE, SD_MANUAL };

enum {W_OFF, W_SMALL, W_BIG, W_SUPER};

#ifdef __FAST_SWEEP__
#define MINIMUM_SWEEP_TIME  1800U    // Minimum sweep time on zero span in uS
#else
#define MINIMUM_SWEEP_TIME  15000U   // Minimum sweep time on zero span in uS
#endif
#define MAXIMUM_SWEEP_TIME  600000000U // Maximum sweep time uS
#define ONE_SECOND_TIME     1000000U // One second uS
#define ONE_MS_TIME         1000U    // One ms uS

#define REPEAT_TIME         111         // Time per extra repeat in uS
#define MEASURE_TIME        127         // Time per single point measurement with vbwstep =1 without step delay in uS

extern const float unit_scale_value[];
extern const char  unit_scale_text[];
#ifdef TINYSA4
extern int debug_frequencies;
extern int linear_averaging;
#endif
#if 1   // Still sufficient flash
// Flash save area - flash7  : org = 0x0801B000, len = 20k in *.ld file
// 2k - for config save
// 9 * 2k for setting_t + stored trace
#ifdef TINYSA4
#define SAVEAREA_MAX 5
#else
#define SAVEAREA_MAX 5
#endif

// STM32 minimum page size for write
#define FLASH_PAGESIZE          0x800
// config save area (flash7 addr)
#ifdef TINYSA3
#define SAVE_CONFIG_ADDR        0x0801D000
#define SAVE_CONFIG_SIZE        FLASH_PAGESIZE
#define FLASH_END               0x08020000
#define FLASH_START_ADDRESS     0x08000000
#define FLASH_TOTAL_SIZE        (128*1024)
#endif

#ifdef TINYSA4
#define SAVE_CONFIG_ADDR        0x0803C000
#define SAVE_CONFIG_SIZE        FLASH_PAGESIZE*2
#define FLASH_END               0x08040000
#define FLASH_START_ADDRESS     0x08000000
#define FLASH_TOTAL_SIZE        (256*1024)
#endif

typedef char assert_config[sizeof(config_t)> SAVE_CONFIG_SIZE ? -1 : 1];        // Check config size

// setting_t save area (save area + config size)
#define SAVE_PROP_CONFIG_ADDR   (SAVE_CONFIG_ADDR + SAVE_CONFIG_SIZE)

#ifdef TINYSA4
#define SAVE_PROP_CONFIG_SIZE   0x00000800
#else
#define SAVE_PROP_CONFIG_SIZE   0x00000800
#endif

typedef char assert_setting[sizeof(setting_t)> SAVE_PROP_CONFIG_SIZE ? -1 : 1]; // Check setting size

// Should include all save slots
#define SAVE_CONFIG_AREA_SIZE   (SAVE_CONFIG_SIZE + SAVEAREA_MAX * SAVE_PROP_CONFIG_SIZE)

typedef char assert_flash[ SAVE_CONFIG_ADDR + SAVE_CONFIG_AREA_SIZE > FLASH_END ? -1 : 1];

#else
#define SAVEAREA_MAX 4
// Begin addr                   0x0801C000
#define SAVE_CONFIG_AREA_SIZE   0x00004000
// config save area
#define SAVE_CONFIG_ADDR        0x0801C000
// properties_t save area
#define SAVE_PROP_CONFIG_0_ADDR 0x0801C800
#define SAVE_PROP_CONFIG_1_ADDR 0x0801D000
#define SAVE_PROP_CONFIG_2_ADDR 0x0801D800
#define SAVE_PROP_CONFIG_3_ADDR 0x0801E000
#define SAVE_PROP_CONFIG_4_ADDR 0x0801e800
#endif

#if 0
typedef struct properties {
  uint32_t magic;
  preset_t setting;
//  freq_t _frequency0;
//  freq_t _frequency1;
  uint16_t _sweep_points;
#ifdef __VNA__
  uint16_t _cal_status;

#endif
#ifdef __SA__
//  freq_t _frequency_IF; //IF frequency
#endif
//  freq_t _frequencies[POINTS_COUNT];
#ifdef __VNA__
  float _cal_data[5][POINTS_COUNT][2];
  float _electrical_delay; // picoseconds
#endif
  trace_t _trace[TRACES_MAX];
  marker_t _markers[MARKERS_MAX];
  int8_t _active_marker;
#ifdef __VNA__
  float _velocity_factor; // %
  uint8_t _domain_mode; /* 0bxxxxxffm : where ff: TD_FUNC m: DOMAIN_MODE */
  uint8_t _marker_smith_format;
  uint8_t _bandwidth;
#endif  
  uint8_t _reserved[2];
  uint32_t checksum;
} properties_t;

#endif

//sizeof(properties_t) == 0x1200

#define CONFIG_MAGIC  0x434f4e69
#define SETTING_MAGIC 0x434f4e69

extern int16_t lastsaveid;
//extern properties_t *active_props;

//extern properties_t current_props;
#ifdef __USE_FREQ_TABLE__
extern freq_t frequencies[POINTS_COUNT];
#define getFrequency(idx) frequencies[idx]
#ifndef getFrequency
freq_t getFrequency(uint16_t idx);
#endif
#else
freq_t getFrequency(uint16_t idx);
#endif
#ifdef __BANDS__
int getBand(uint16_t idx);
#endif

//#define frequency0 current_props._frequency0
//#define frequency1 current_props._frequency1
#define sweep_points setting._sweep_points
#ifdef __VNA__
#define cal_status current_props._cal_status
#endif
#ifdef __SA__
//#define frequency_IF current_props._frequency_IF
#endif
//#define frequencies current_props._frequencies
#ifdef __VNA__
#define cal_data active_props->_cal_data
#define electrical_delay current_props._electrical_delay
#endif
#define markers setting._markers
#define active_marker setting._active_marker
#ifdef __VNA__
#define domain_mode current_props._domain_mode
#define velocity_factor current_props._velocity_factor
#define marker_smith_format current_props._marker_smith_format
#define bandwidth current_props._bandwidth
#endif

#define FREQ_IS_STARTSTOP() (!(setting.freq_mode&FREQ_MODE_CENTER_SPAN))
#define FREQ_IS_CENTERSPAN() (setting.freq_mode&FREQ_MODE_CENTER_SPAN)
#define FREQ_IS_CW() (setting.frequency0 == setting.frequency1)
int caldata_recall(uint16_t id);
int caldata_save(uint16_t id);
//const properties_t *caldata_ref(int id);
int config_save(void);
int config_recall(void);
setting_t * caldata_pointer(uint16_t id);
uint32_t checksum(const void *start, size_t len);

void clear_all_config_prop_data(void);

/*
 * ui.c
 */

// Set structure align as WORD (save flash memory)
#pragma pack(push, 2)
typedef struct {
  const uint8_t type;
  const uint8_t data;
  const char *label;
  const void *reference;
} menuitem_t;
#pragma pack(pop)


extern void ui_init(void);
extern void ui_process(void);
int current_menu_is_form(void);
extern float nf_gain;
extern const char * const averageText[];
extern uint8_t menu_current_level;
void menu_invoke(int item);
extern char    kp_buf[];
void set_numeric_value(void);

void ui_mode_normal(void);
void ui_mode_menu(void);
void menu_push_lowoutput(void);
void menu_push_highoutput(void);
void menu_move_top(void);
void draw_menu(void);
void draw_menu_mask(uint32_t mask);
void refres_sweep_menu(void);
int check_touched(void);
void touch_set(int16_t x, int16_t y);
int invoke_quick_menu(int);
bool ui_process_listen_lever(void);
void refresh_sweep_menu(int i);
void save_to_sd(int mask);
void drawMessageBox(const char *header, char *text, uint32_t delay);
bool isFullScreenMode(void);
int btn_side(void);
extern int si5351_available;

// Irq operation process set
#define OP_NONE       0x00
#define OP_LEVER      0x01
#define OP_TOUCH      0x02
#define OP_CONSOLE    0x04
//#define OP_FREQCHANGE 0x04
extern volatile uint8_t operation_requested;
extern volatile uint8_t break_execute;
extern volatile uint8_t abort_enabled;


// lever_mode
enum lever_mode {
  LM_MARKER, LM_SEARCH, LM_CENTER, LM_SPAN, LM_TRIGGER
};

// marker smith value format
enum marker_smithvalue {
  MS_LIN, MS_LOG, MS_REIM, MS_RX, MS_RLC
};

typedef struct uistat {
  float  value; // for editing at numeric input area
  freq_t freq_value; // for editing frequencies that do not fit in float;
  int8_t current_trace; /* 0..3 */
  uint8_t lever_mode;
  uint8_t marker_delta;
  uint8_t marker_noise;
  uint8_t marker_tracking;
  uint8_t auto_center_marker;
  char text[28];
} uistat_t;

typedef struct ui_button {
  uint16_t fg;
  uint16_t bg;
  uint8_t  border;
  int8_t   icon;
  union {
    int32_t  i;
    uint32_t u;
    float    f;
    const char *text;
  } param_1;    // void data for label printf
  char text[32];
} ui_button_t;

typedef struct ui_slider {
  uint8_t keypad;
  uint8_t has_steps;
  uint16_t slider_position;
  uint16_t slider_step;
  float min_value;
  float max_value;
} ui_slider_t;

extern uistat_t uistat;
void ui_init(void);
void ui_show(void);
void ui_hide(void);

void touch_position(int *x, int *y);
void handle_touch_interrupt(void);

#define TOUCH_THRESHOLD 2000

void touch_cal_exec(void);
void touch_draw_test(void);
void enter_dfu(void);

#ifdef TINYSA4
extern char range_text[20];
#endif

/*
 * adc.c
 */
#ifdef TINYSA4
#define rccEnableWWDG(lp) rccEnableAPB1(RCC_APB1ENR_WWDGEN, lp)
#define ADC_TOUCH_X  ADC_CHANNEL_IN3
#define ADC_TOUCH_Y  ADC_CHANNEL_IN4
uint16_t adc1_single_read(uint32_t chsel);
#else
#define ADC_TOUCH_X  ADC_CHSELR_CHSEL6
#define ADC_TOUCH_Y  ADC_CHSELR_CHSEL7
#endif

void adc_init(void);
uint16_t adc_single_read(uint32_t chsel);
void adc_start_analog_watchdog(void);
void adc_stop_analog_watchdog(void);
int16_t adc_vbat_read(void);

/*
 * rtc.c
 */
#ifdef __USE_RTC__
#define RTC_START_YEAR          2000

#define RTC_DR_YEAR(dr)         (((dr)>>16)&0xFF)
#define RTC_DR_MONTH(dr)        (((dr)>> 8)&0xFF)
#define RTC_DR_DAY(dr)          (((dr)>> 0)&0xFF)

#define RTC_TR_HOUR(dr)         (((tr)>>16)&0xFF)
#define RTC_TR_MIN(dr)          (((tr)>> 8)&0xFF)
#define RTC_TR_SEC(dr)          (((tr)>> 0)&0xFF)

// Init RTC
void rtc_init(void);
// Then read time and date TR should read first, after DR !!!
// Get RTC time as bcd structure in 0x00HHMMSS
#define rtc_get_tr_bcd() (RTC->TR & 0x007F7F7F)
// Get RTC date as bcd structure in 0x00YYMMDD (remove day of week information!!!!)
#define rtc_get_dr_bcd() (RTC->DR & 0x00FF1F3F)
// read TR as 0x00HHMMSS in bin (TR should be read first for sync)
uint32_t rtc_get_tr_bin(void);
// read DR as 0x00YYMMDD in bin (DR should be read second)
uint32_t rtc_get_dr_bin(void);
// Read time in FAT filesystem format
uint32_t rtc_get_FAT(void);
// Write date and time (need in bcd format!!!)
void rtc_set_time(uint32_t dr, uint32_t tr);
#endif

// SD Card support, discio functions for FatFS lib implemented in ili9341.c
#ifdef  __USE_SD_CARD__
#include "../FatFs/ff.h"
#include "../FatFs/diskio.h"
extern uint16_t sd_card_inserted_at_boot;
bool SD_Inserted(void);
void SD_PowerOff(void);
// Buffers for SD card use spi_buffer
#if SPI_BUFFER_SIZE < 2048
#error "SPI_BUFFER_SIZE for SD card support need size >= 2048"
#else
// Fat file system work area (at the end of spi_buffer)
#define fs_volume    (FATFS *)(((uint8_t*)(&spi_buffer[SPI_BUFFER_SIZE])) - sizeof(FATFS))
// FatFS file object (at the end of spi_buffer)
#define fs_file      (   FIL*)(((uint8_t*)(&spi_buffer[SPI_BUFFER_SIZE])) - sizeof(FATFS) - sizeof(FIL))
#endif
void testLog(void);        // debug log
void sd_card_load_config(char *filename);
extern systime_t last_auto_save;
void save_csv(uint8_t mask);
#endif

/*
 * Backup
 */
#pragma pack(push)
#pragma pack(1)

#ifdef TINYSA4
extern uint8_t SI4463_rbw_selected;
#else
extern uint8_t SI4432_rbw_selected;
#endif
extern const menuitem_t  menu_lowoutputmode[];
extern const menuitem_t  menu_highoutputmode[];
extern const menuitem_t  menu_mode[];
extern void menu_push_submenu(const menuitem_t *submenu);

#ifdef TINYSA4
#define MAX_BACKUP_SIZE 16
#define USED_BACKUP_SIZE 6
#else
#define MAX_BACKUP_SIZE 5
#define USED_BACKUP_SIZE 4      // must be equal to sizeof(backup_t)
#endif


typedef struct {
  union {
    uint32_t raw[USED_BACKUP_SIZE];     // checksum must be last byte
    struct {
      freq_t  frequency0, frequency1;
      uint8_t attenuation;
      uint8_t reflevel;
      uint8_t RBW;
      uint8_t mode;
      int8_t external_gain;
      uint8_t harmonic,dummy2;
      uint8_t checksum;
    } data;
  };
} backup_t;
#pragma pack(pop)

#define backup (*(uint32_t *)0x40002850)   // backup registers 5 * 32 bits


#if USED_BACKUP_SIZE > MAX_BACKUP_SIZE
#error "backup_t too large"
#endif
//#if sizeof(backup_t) != USED_BACKUP_SIZE  // does not work
//#error "backup_t size incorrect"
//#endif
/*
 * misclinous
 */
int parse_line(char *line, char* args[], int max_cnt);
int plot_printf(char *str, int, const char *fmt, ...);
#define PULSE do { palClearPad(GPIOC, GPIOC_LED); palSetPad(GPIOC, GPIOC_LED);} while(0)
//extern int setting_attenuate;
//extern int settingPowerCal;
//extern int setting_step_delay;
//extern int actualStepDelay;
//extern int setting_mode;

#define ARRAY_COUNT(a)    (sizeof(a)/sizeof(*(a)))
// Speed profile definition
#define START_PROFILE   systime_t time = chVTGetSystemTimeX();
#define RESTART_PROFILE   time = chVTGetSystemTimeX();
#define STOP_PROFILE    {char string_buf[12];plot_printf(string_buf, sizeof string_buf, "%06d", chVTGetSystemTimeX() - time);ili9341_set_foreground(LCD_FG_COLOR);ili9341_drawstring(string_buf, 0, FREQUENCIES_YPOS);}
#define DELTA_TIME (time = chVTGetSystemTimeX() - time)
// Macros for convert define value to string
#define STR1(x)  #x
#define define_to_STR(x)  STR1(x)

// sa_core.c

typedef uint8_t  deviceRSSI_t;
typedef int16_t  pureRSSI_t;
extern int current_index;
// RSSI values conversion macro

#define DEVICE_TO_PURE_RSSI(rssi) ((rssi)<<4)
#define PURE_TO_DEVICE_RSSI(rssi) ((rssi)>>4)
#define float_TO_PURE_RSSI(rssi)  ((rssi)*32)
#define PURE_TO_float(rssi)       ((rssi)/32.0)

extern uint16_t actual_rbw_x10;

void toggle_tracking(void);
void toggle_hambands(void);
void reset_calibration(void);
void set_reflevel(float);
void set_external_gain(float);
void set_unit(int);
void set_switches(int);
void set_gridlines(int);
void set_trigger_level(float);
void set_trigger(int);
void update_rbw(void);
void set_fast_speedup(int);
void set_faster_speedup(int);
//extern int setting_measurement;
void self_test(int);
//extern int setting_test;
void wait_user(void);
void calibrate(void);
void calibrate_harmonic(void);
float to_dBm(float);
float dBm_to_Watt(float);
uint32_t calc_min_sweep_time_us(void);
pureRSSI_t perform(bool b, int i, freq_t f, int e);
extern pureRSSI_t get_frequency_correction(freq_t f);

void interpolate_maximum(int m);
void calibrate_modulation(int modulation, int8_t *correction);

enum {
  M_OFF, M_IMD, M_OIP3, M_PHASE_NOISE, M_SNR, M_PASS_BAND, M_LINEARITY, M_AM, M_FM, M_THD, M_CP, M_NF_TINYSA, M_NF_STORE, M_NF_VALIDATE, M_NF_AMPLIFIER, M_DECONV,M_MAX
};
#define MEASUREMENT_TEXT "OFF","HARM","OIP3","PN","SNR","PASS","LIN","AM","FM","THD","CP","NF T","NF S","NF V","NF A", "DECONF"

enum {
  T_AUTO, T_NORMAL, T_SINGLE, T_DONE, T_UP, T_DOWN, T_MODE, T_PRE, T_POST, T_MID, T_BEEP, T_AUTO_SAVE,
};

//!!! Warning can show not correct results on CH_CFG_ST_FREQUENCY not round by 1000 or > 1000000UL
#define sa_ST2US(n) ((n)*(1000000UL/(CH_CFG_ST_FREQUENCY)))

extern const uint8_t SI4432_RBW_count;
extern void SI4432_Listen(int s);

#ifdef TINYSA4
// si4432.c


enum {PATH_OFF, PATH_LOW, PATH_DIRECT, PATH_LEAKAGE, PATH_ULTRA, PATH_HARM, PATH_HIGH};  // must be same order as correction tables!!!!
#define PATH_TEXT {"OFF", "LOW", "DIRECT", "ADF", "ULTRA", "HARM", "High"}
extern const char * const path_text[];
extern int signal_path;
extern int test_path;
extern int force_signal_path;

extern uint16_t R;
extern uint8_t rfPower;
extern void ADF4351_mux(int R);
extern void ADF4351_force_refresh(void);
extern void ADF4351_CP(int p);
extern uint16_t ADF4351_get_CP(void);
extern void ADF4351_modulo(int m);
extern uint16_t ADF4351_get_modulo(void);
extern void ADF4351_csr(int c);
extern void ADF4351_fastlock(int c);
extern void ADF4351_recalculate_PFDRFout(void);
extern int SI4463_R;
extern int64_t ADF4350_modulo;
extern void SI446x_set_AGC_LNA(uint8_t v);
extern void SI4463_init_rx(void);
extern void SI4463_init_tx(void);
extern void SI4463_start_tx(uint8_t CHANNEL);
extern void SI4463_set_output_level(int t);
extern freq_t SI4463_set_freq(freq_t freq);
extern uint16_t set_rbw(uint16_t rbw_x10);
extern uint16_t force_rbw(int f);
extern int SI4463_do_api(void* data, uint8_t len, void* out, uint8_t outLen);
extern void SI4463_set_gpio(int i, int s);
extern void si_set_offset(int16_t offset);
extern void si_fm_offset(int16_t offset);
extern bool ADF4351_frequency_changed;
extern bool SI4463_frequency_changed;
extern bool SI4463_offset_changed;
extern int SI4463_is_in_tx_mode(void);
extern int16_t SI4463_noise_correction_x10;
void switch_SI4463_RSSI_correction(bool);
extern int old_R;
extern float Si446x_get_temp(void);
#define ENBW_Hz    SI4463_ENBW_Hz
#endif
#ifdef TINYSA3
#define ENBW_Hz    actual_rbw_x10*100;
#endif
/*EOF*/
