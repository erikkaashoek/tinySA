/* All rights reserved.
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

#include <math.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "nanovna.h"

#pragma GCC push_options
#pragma GCC optimize ("Os")      // Makes the code just a bit faster, disable during debugging.

#ifdef __SCROLL__
uint16_t _grid_y = (CHART_BOTTOM / NGRIDY);
uint16_t graph_bottom = CHART_BOTTOM;
#endif
static void cell_draw_marker_info(int x0, int y0);
static void cell_grid_line_info(int x0, int y0);
static void cell_blit_bitmap(int x, int y, uint16_t w, uint16_t h, const uint8_t *bitmap);
static void draw_battery_status(void);
static void update_waterfall(void);
#ifdef __LEVEL_METER__
static void update_level_meter(void);
char level_text[20];
char freq_text[20];
#endif
void cell_draw_test_info(int x0, int y0);
int cell_printf(int16_t x, int16_t y, const char *fmt, ...);
#ifndef wFONT_GET_DATA
static void cell_drawstring_size(char *str, int x, int y, int size);
#endif
static int16_t grid_offset;
static int16_t grid_width;
static freq_t grid_span;

uint16_t area_width  = AREA_WIDTH_NORMAL;
uint16_t area_height; // initialized in main()  = AREA_HEIGHT_NORMAL;

// Cell render use spi buffer
pixel_t *cell_buffer = (pixel_t *)spi_buffer;

// Check buffer size
#if CELLWIDTH*CELLHEIGHT > SPI_BUFFER_SIZE
#error "Too small spi_buffer size SPI_BUFFER_SIZE < CELLWIDTH*CELLHEIGH"
#endif

// indicate dirty cells (not redraw if cell data not changed)
#define MAX_MARKMAP_X    ((LCD_WIDTH+CELLWIDTH-1)/CELLWIDTH)
#define MAX_MARKMAP_Y    ((LCD_HEIGHT+CELLHEIGHT-1)/CELLHEIGHT)
// Define markmap mask size
#if MAX_MARKMAP_X <= 8
typedef uint8_t map_t;
#elif MAX_MARKMAP_X <= 16
typedef uint16_t map_t;
#elif MAX_MARKMAP_X <= 32
typedef uint32_t map_t;
#endif

static map_t   markmap[2][MAX_MARKMAP_Y+1];
static uint8_t current_mappage = 0;

// Trace data cache, for faster redraw cells
// Set data size types depend from screen resolution
// Set type for x resolution
#if LCD_WIDTH < 256
typedef uint8_t   index_x_t;
#else
typedef uint16_t  index_x_t;
#endif
// Set type for y resolution
#if LCD_HEIGHT < 256
typedef uint8_t   index_y_t;
#else
typedef uint16_t  index_y_t;
#endif

static index_x_t trace_index_x[POINTS_COUNT];
static index_y_t trace_index_y[TRACES_MAX][POINTS_COUNT];

uint16_t marker_color(int m_i)
{
  return LCD_TRACE_1_COLOR + markers[m_i].trace;
}

#if 1
#define float2int(v) ((int)(v))
#else
static int 
float2int(float v) 
{
  if (v < 0) return v - 0.5;
  if (v > 0) return v + 0.5;
  return 0;
}
#endif

void update_grid(void)
{
  freq_t gdigit = 1000000000;
  freq_t fstart = get_sweep_frequency(ST_START) + (setting.frequency_offset - FREQUENCY_SHIFT);
  freq_t fspan  = get_sweep_frequency(ST_SPAN);
  freq_t grid;

  if (fspan == 0) {
    fspan = setting.actual_sweep_time_us; // Time in uS
    fstart = 0;
  }
  if (config.gridlines == 0) {
    grid = fspan/10;
  } else {
  if (config.gridlines < 3)
    config.gridlines = 6;
  while (gdigit > 1) {
    grid = 5 * gdigit;
    if (fspan / grid >= config.gridlines)
      break;
    grid = 2 * gdigit;
    if (fspan / grid >= config.gridlines)
      break;
    grid = gdigit;
    if (fspan / grid >= config.gridlines)
      break;
    gdigit /= 10;
  }
  }
  grid_span = grid;
  if (grid > 1000) {
    grid_offset = (WIDTH) * ((fstart % grid) / 100) / (fspan / 100);
    grid_width = (WIDTH) * (grid / 100) / (fspan / 1000);
  } else {
    grid_offset = (WIDTH) * ((fstart % grid)) / (fspan);
    grid_width = (WIDTH) * (grid) / (fspan/10);
  }
  if (config.gridlines == 0)
    grid_offset = 0;
//  if (setting.waterfall)
  set_level_meter_or_waterfall();
  redraw_request |= REDRAW_FREQUENCY | REDRAW_AREA;
}

#if 0
static int
rectangular_grid(int x, int y)
{
  //#define FREQ(x) (((x) * (fspan / 1000) / (WIDTH-1)) * 1000 + fstart)
  //int32_t n = FREQ(x-1) / fgrid;
  //int32_t m = FREQ(x) / fgrid;
  //if ((m - n) > 0)
  //if (((x * 6) % (WIDTH-1)) < 6)
  //if (((x - grid_offset) % grid_width) == 0)
  if (x == 0 || x == WIDTH-1)
    return 1;
  if ((y % GRIDY) == 0)
    return 1;
  if ((((x + grid_offset) * 10) % grid_width) < 10)
    return 1;
  return 0;
}
#endif

#ifdef __HAM_BAND__
typedef const struct {
  freq_t start;
  freq_t stop;
} ham_bands_t;

const ham_bands_t ham_bands[] =
{
  {135700, 137800},
  {472000, 479000},
  {1800000, 2000000},
  {3500000, 3800000},
  {5250000, 5450000},
  {7000000, 7200000},
  {10100000, 10150000},
  {14000000, 14350000},
  {18068000, 18168000},
  {21000000, 21450000},
  {24890000, 24990000},
  {28000000, 29700000},
  {50000000, 52000000},
  {70000000, 70500000},
  {144000000, 148000000},
  {222000000, 225000000},
  {420000000, 450000000},
  {902000000, 928000000},
#ifdef TINYSA4
  {1240000000,1300000000},
  {2300000000, 2310000000},
  {2390000000, 2450000000},
  {3300000000, 3500000000},
  {5650000000, 5925000000},
  {10000000000, 10500000000}
#endif
};
#ifdef TINYSA4
const int ham_band_max = 24;
#else
const int ham_band_max = 18;
#endif

int ham_band(int x)      // Search which index in the frequency tabled matches with frequency  f using actual_rbw
{
  freq_t f = getFrequency(x)  + (setting.frequency_offset - FREQUENCY_SHIFT);
  int L = 0;
  int R =  ham_band_max - 1;
  while (L <= R) {
    int m = (L + R) / 2;
    if (ham_bands[m].stop < f)
      L = m + 1;
    else if (ham_bands[m].start > f)
      R = m - 1;
    else
       return true; // index is m
  }
  return false;
}
#endif

static int
rectangular_grid_x(int x)
{
  x -= CELLOFFSETX;
  if (x < 0) return 0;
  if (x == 0 || x == WIDTH)
    return 1;
  if ((((x + grid_offset) * 10) % grid_width) < 10)
    return 1;
  return 0;
}

static int
rectangular_grid_y(int y)
{
  if (y < 0)
    return 0;
  if ((y % GRIDY) == 0)
    return 1;
  return 0;
}

#if 0
int
set_strut_grid(int x)
{
  uint16_t *buf = spi_buffer;
  int y;

  for (y = 0; y < HEIGHT; y++) {
    int c = rectangular_grid(x, y);
    c |= smith_grid(x, y);
    *buf++ = c;
  }
  return y;
}

void
draw_on_strut(int v0, int d, int color)
{
  int v;
  int v1 = v0 + d;
  if (v0 < 0) v0 = 0;
  if (v1 < 0) v1 = 0;
  if (v0 >= HEIGHT) v0 = HEIGHT-1;
  if (v1 >= HEIGHT) v1 = HEIGHT-1;
  if (v0 == v1) {
    v = v0; d = 2;
  } else if (v0 < v1) {
    v = v0; d = v1 - v0 + 1;
  } else {
    v = v1; d = v0 - v1 + 1;
  }
  while (d-- > 0)
    spi_buffer[v++] |= color;
}
#endif

/*
 * calculate log10f(abs(gamma))
 */ 
#if 0
float
index_to_value(const int i)
{
  return(value(actual_t[i]));
}
#endif

#ifdef __MARKER_CACHE__
float marker_cache[MARKERS_MAX];
bool marker_cache_valid[MARKERS_MAX];
int32_t marker_cache_index[MARKERS_MAX];

void
clear_marker_cache(void)
{
  for (int i = 0; i<MARKERS_MAX; i++) {
    marker_cache_valid[i] = false;
  }
}
#endif

float
marker_to_value(const int i)
{
#ifdef __MARKER_CACHE__
  if (markers[i].mtype & M_AVER && linear_averaging && marker_cache_valid[i] && marker_cache_index[i] == markers[i].index)
    return marker_cache[i];
#endif
  float *ref_marker_levels;
  ref_marker_levels = measured[markers[i].trace];
  float v = value(ref_marker_levels[markers[i].index]);
  if (markers[i].mtype & M_AVER) {
    int old_unit = setting.unit;
    if (markers[i].mtype & M_NOISE
#ifdef TINYSA4
        && linear_averaging
#endif
    )
      setting.unit = U_WATT;            // Noise averaging should always be done in Watts
    v = 0;
    for (int i=0; i<sweep_points; i++)
      v += value(ref_marker_levels[i]); // TODO this should be power averaging for noise markers
    v /= sweep_points;
    v = to_dBm(v);
    setting.unit = old_unit;
    v = value(v);
  }
  if (markers[i].mtype & M_NOISE){
    v = v - logf(actual_rbw_x10*100.0) * (10.0/logf(10.0))
#ifdef TINYSA4
    + ((float)SI4463_noise_correction_x10)/10.0 + (linear_averaging ? 0.0 : log_averaging_correction);
#endif
    ;
  }
#ifdef __MARKER_CACHE__
  marker_cache_valid[i] = true;
  marker_cache_index[i] = markers[i].index;
  marker_cache[i] = v;
#endif
  return(v);
}

#ifdef TINYSA3
float sa_sqrtf(const float x)
{
  union
  {
    int i;
    float x;
  } u;
  u.x = x;
  u.i = (1<<29) + (u.i >> 1) - (1<<22);

  // Two Babylonian Steps (simplified from:)
  // u.x = 0.5f * (u.x + x/u.x);
  // u.x = 0.5f * (u.x + x/u.x);
  u.x =       u.x + x/u.x;
  u.x = 0.25f*u.x + x/u.x;

  return u.x;
}
#else
//#define sa_sqrtf(x) sqrtf(x)
__attribute__((always_inline)) __STATIC_INLINE float sa_sqrtf(float x){__asm__ ("vsqrt.f32 %0, %1" : "=t"(x) : "t"(x)); return x;}
#endif

// Function for convert to different type of values from dBm
// Replaced some equal functions and use recalculated constants:
// powf(10,x) =  expf(x * logf(10))
// log10f(x)  =  logf(x)/logf(10)
float
value(const float v)
{
  switch(setting.unit)
  {
  case U_DBMV:
    return v + (30.0 + 20.0*log10f(sqrtf(50.0)));
  case U_DBUV:
    return v + (90.0 + 20.0*log10f(sqrtf(50.0)));
  case U_VOLT:
//  return powf(10.0, (v-30.0)/20.0) * sqrtf(50.0);                     // powf(10.0, v           /20.0) * powf(10, -30.0/20.0) * sqrtf(50)
    return expf(v*(logf(10.0)/20.0)) * (powf(10, -30.0/20.0)*sqrtf(50));//       expf(v*logf(10.0)/20.0) * powf(10, -30.0/20.0) * sqrtf(50)
  case U_VPP:
//  return powf(10.0, (v-30.0)/20.0) * sqrtf(50.0);                     // powf(10.0, v           /20.0) * powf(10, -30.0/20.0) * sqrtf(50)
    return 2.828 * expf(v*(logf(10.0)/20.0)) * (powf(10, -30.0/20.0)*sqrtf(50));//       expf(v*logf(10.0)/20.0) * powf(10, -30.0/20.0) * sqrtf(50)
  case U_WATT:
//    return powf(10.0, v/10.0)/1000.0;               // powf(10.0, v           /10.0) / 1000.0
    return expf(v*(logf(10.0)/10.0)) / 1000.0;        //       expf(v*logf(10.0)/10.0) / 1000.0
  }
  //  case U_DBM:
  //  case U_RAW:
    return v;  // raw data is in logmag*10 format
}

float
to_dBm(const float v)
{
  switch(setting.unit)
  {
  case U_DBMV:
    return v - (30.0 + 20.0*log10f(sqrtf(50.0)));
  case U_DBUV:
    return v - (90.0 + 20.0*log10f(sqrtf(50.0)));
  case U_VOLT:
//  return log10f(v/(sqrtf(50.0)))* 20.0             + 30.0;
    return   logf(v/(sqrtf(50.0)))*(20.0/logf(10.0)) + 30.0;
  case U_VPP:
//  return log10f(v/(sqrtf(50.0)))* 20.0             + 30.0;
    return   logf(v/(2.828*sqrtf(50.0)))*(20.0/logf(10.0)) + 30.0;
  case U_WATT:
//  return log10f(v*1000.0)* 10.0;
    return   logf(v*1000.0)*(10.0/logf(10.0));
  }
//  case U_DBM:
    return v;  // raw data is in logmag*10 format
}

float
dBm_to_Watt(const float v)
{
  return   logf(v*1000.0)*(10.0/logf(10.0));
}

static float fast_expf(float x)
{
  union { float f; int32_t i; } v;
  v.i = (int32_t)(12102203.0f*x) + 0x3F800000;
  int32_t m = (v.i >> 7) & 0xFFFF;  // copy mantissa
#if 1
  // cubic spline approximation
  // empirical values for small maximum relative error (8.34e-5):
  v.i += ((((((((1277*m) >> 14) + 14825)*m) >> 14) - 79749)*m) >> 11) - 626;
#else
  // quartic spline approximation
  // empirical values for small maximum relative error (1.21e-5):
  v.i += (((((((((((3537*m) >> 16) + 13668)*m) >> 18) + 15817)*m) >> 14) - 80470)*m) >> 11);
#endif
  return v.f;
}

static void
trace_into_index_x_array(index_x_t *x, uint16_t points){
  // Not need update if index calculated for this points count
  static uint16_t old_points = 0;
  if (old_points == points) return;
  old_points = points;
  points-=1;
  for (int i=0; i<= points;i++)
    x[i] = (i * WIDTH + (points>>1)) / points + CELLOFFSETX;
}

//
// Optimized by speed/size array processing of value(const float v) function
// on screen need calculate as:
// y = (ref-v)/scale
// and align by top/bottom
static void
trace_into_index_y_array(index_y_t *y, float *array, int points)
{
  float scale = GRIDY / get_trace_scale();
  float ref   = get_trace_refpos();
  float mult = 0, vmult = 1.0;
  float ref_shift = 0;
  switch (setting.unit){
  case U_DBM: break;
#ifdef TINYSA4
  case U_RAW: break;
#endif
    case U_DBMV: ref_shift = 30.0 + 20.0*log10f(sqrtf(50.0));break;
    case U_DBUV: ref_shift = 90.0 + 20.0*log10f(sqrtf(50.0));break;
    case U_VOLT: vmult = powf(10, -30.0/20.0) * sqrtf(50.0); mult = logf(10.0)/20.0;break;
    case U_VPP:  vmult = 2.828* powf(10, -30.0/20.0) * sqrtf(50.0); mult = logf(10.0)/20.0;break;
    case U_WATT: vmult = 1.0/1000.0;                         mult = logf(10.0)/10.0;break;
    default:
    return;
  }
  // Universal formula look like this:
  // v = (refpos - (mult ? expf(value*mult) : value) - ref_shift) * vmult) * scale;
  // v = ((refpos - ref_shift) * scale) - (mult ? expf(value*mult) : value) * (vmult * scale)
  // Made precalculated constants:
  ref = (ref - ref_shift) * scale + 0.5; // add 0.5 for floor on int convert
  scale  = scale * vmult;
  int max = NGRIDY * GRIDY, i;
  for (i=0;i<points;i++){
    float value = array[i];
    if (mult) value = fast_expf(value*mult);
    value = ref - value * scale;
    int v = value;
         if (v <    0) v = 0;
    else if (v >= max) v = max-1;
    y[i] = v;
  }
  return;
}

static index_y_t trace_into_index_y(float level) {
  index_y_t tp;
  trace_into_index_y_array(&tp, &level, 1);
  return tp;
}

static inline void
swap_markmap(void)
{
  current_mappage^= 1;
}

static void
clear_markmap(void)
{
  memset(markmap[current_mappage], 0, sizeof markmap[current_mappage]);
}

static void
force_set_markmap(void)
{
  memset(markmap[current_mappage], 0xff, sizeof markmap[current_mappage]);
}

/*
 * Force region of screen update
 */
static void
invalidate_rect_func(int x0, int y0, int x1, int y1)
{
  int x, y;
  map_t *map = &markmap[current_mappage][0];
  for (y = y0; y <= y1; y++)
    if ((uint32_t)y < MAX_MARKMAP_Y)
      for (x = x0; x <= x1; x++)
        map[y]|= 1 << x;
}
#define invalidate_rect(x0, y0, x1, y1) invalidate_rect_func((x0)/CELLWIDTH, (y0)/CELLHEIGHT, (x1)/CELLWIDTH, (y1)/CELLHEIGHT)

#define SWAP(x,y) {int t=x;x=y;y=t;}
static void
mark_cells_from_index(void)
{
  int t, i, j;
  /* mark cells between each neighber points */
  map_t *map = &markmap[current_mappage][0];
  index_x_t *index_x = trace_index_x;
  for (t = 0; t < TRACES_MAX; t++) {
    if (IS_TRACE_DISABLE(t))
      continue;
    index_y_t *index_y = trace_index_y[t];
    int m0 = index_x[0] / CELLWIDTH;
    int n0 = index_y[0] / CELLHEIGHT;
    map[n0] |= 1 << m0;
    for (i = 1; i < sweep_points; i++) {
      int m1 = index_x[i] / CELLWIDTH;
      int n1 = index_y[i] / CELLHEIGHT;
      if (m0 == m1 && n0 == n1)
        continue;
      int x0 = m0; int x1 = m1; if (x0>x1) SWAP(x0, x1); m0 = m1;
      int y0 = n0; int y1 = n1; if (y0>y1) SWAP(y0, y1); n0 = n1;
      for (; y0 <= y1; y0++)
        for (j = x0; j <= x1; j++)
          map[y0] |= 1 << j;
    }
  }
}

static inline void
markmap_upperarea(void)
{
  // Hardcoded, Text info from upper area
#if MARKER_COUNT == 4
  invalidate_rect(0, 0, AREA_WIDTH_NORMAL, 31);
#else
  invalidate_rect(0, 0, AREA_WIDTH_NORMAL, 63);
#endif
}

#ifdef __BANDS__
// Triggers draw in multiband mode
bool is_multiband_trigger(void) {
  if (setting.multi_band && !setting.multi_trace
        #ifdef __DRAW_LINE__
          && !setting.draw_line
        #endif
     ) return true;
  return false;
}

// Mark band trigger area for redraw
static void markmap_band_trigger(int band) {
  band_t *b = &setting.bands[band];
  if (!b->enabled) return;
  int tp = trace_into_index_y(b->level);
  int x1 = trace_index_x[b->start_index == 0 ? b->start_index : b->start_index - 1];
  int x2 = trace_index_x[b->stop_index];
  invalidate_rect(x1, tp, x2, tp);
}
#endif

// Mark trigger area or line for redraw
static void markmap_trigger(void) {
  int tp = trace_into_index_y(setting.trigger_level);
  markmap[current_mappage][tp/CELLWIDTH] = (map_t)0xFFFFFFFF;
}

// Mark all trigger lines area for redraw
static void markmap_trigger_area(void) {
#ifdef __BANDS__
  if (is_multiband_trigger()) {
    for (int band = 0; band < BANDS_MAX; band++)
      markmap_band_trigger(band);
  } else
#endif
    markmap_trigger();
}

//
// in most cases _compute_outcode clip calculation not give render line speedup
//
static inline void
cell_drawline(int x0, int y0, int x1, int y1, pixel_t c)
{
  if (x0 < 0 && x1 < 0) return;
  if (y0 < 0 && y1 < 0) return;
  if (x0 >= CELLWIDTH && x1 >= CELLWIDTH) return;
  if (y0 >= CELLHEIGHT && y1 >= CELLHEIGHT) return;

  // modifed Bresenham's line algorithm, see https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
  // Draw from top to bottom (most graph contain vertical lines)
  if (y1 < y0) { SWAP(x0, x1); SWAP(y0, y1); }
  int dx =-(x1 - x0), sx = 1; if (dx > 0) { dx = -dx; sx = -sx; }
  int dy = (y1 - y0);
  int err = ((dy + dx) < 0 ? -dx : -dy) / 2;
  // Fast skip points while y0 < 0
  if (y0 < 0){
    while(1){
      int e2 = err;
      if (e2 > dx) { err-= dy; x0+=sx;}
      if (e2 < dy) { err-= dx; y0++; if (y0 == 0) break;}
    }
  }
  // align y by CELLWIDTH for faster calculations
  y0*=CELLWIDTH;
  y1*=CELLWIDTH;
  while (1) {
    if ((uint32_t)x0 < CELLWIDTH)
      cell_buffer[y0 + x0]|= c;
    if (x0 == x1 && y0 == y1)
      return;
    int e2 = err;
    if (e2 > dx) { err-= dy; x0+=sx;}
    if (e2 < dy) { err-= dx; y0+=CELLWIDTH; if (y0>=CELLHEIGHT*CELLWIDTH) return;} // stop after cell bottom
  }
}

static void
cell_draw_hline(int x0, int x1, int y, pixel_t c) {
  if ((uint32_t)y >= CELLHEIGHT) return;
//  if (x1 < x0) SWAP(x0, x1);
  if (x0 <         0) x0 = 0;
  if (x1 > CELLWIDTH) x1 = CELLWIDTH;
  for (;x0 < x1; x0++) cell_buffer[y * CELLWIDTH + x0] = c;
}

static void
cell_draw_vline(int y0, int y1, int x, pixel_t c) {
  if ((uint32_t)x >= CELLWIDTH) return;
//  if (y1 < y0) SWAP(y0, y1);
  if (y0 <          0) y0 = 0;
  if (y1 > CELLHEIGHT) y1 = CELLHEIGHT;
  for (;y0 < y1; y0++) cell_buffer[y0 * CELLWIDTH + x] = c;
}

// Give a little speedup then draw rectangular plot (50 systick on all calls, all render req 700 systick)
// Write more difficult algoritm for seach indexes not give speedup
static int
search_index_range_x(int x1, int x2, int *i0, int *i1)
{
  int i, j;
  int head = 0;
  int tail = sweep_points;
  int idx_x;
  index_x_t *index_x = trace_index_x;
  // Search index point in cell
  while (1) {
    i = (head + tail) / 2;
    idx_x = index_x[i];
    if (idx_x >= x2) { // index after cell
      if (tail == i)
        return false;
      tail = i;
    }
    else if (idx_x < x1) {    // index before cell
      if (head == i)
        return false;
      head = i;
    }
    else  // index in cell (x =< idx_x < cell_end)
      break;
  }
  j = i;
  // Search index left from point
  do {
    j--;
  } while (j > 0 && x1 <= index_x[j]);
  *i0 = j;
  // Search index right from point
  do {
    i++;
  } while (i < sweep_points-1 && index_x[i] < x2);
  *i1 = i;

  return TRUE;
}
#if 0       // Not used as refpos is always at top of screen
#define REFERENCE_WIDTH    6
#define REFERENCE_HEIGHT   5
#define REFERENCE_X_OFFSET 5
#define REFERENCE_Y_OFFSET 2

// Reference bitmap
static const uint8_t reference_bitmap[]={
  _BMP8(0b11000000),
  _BMP8(0b11110000),
  _BMP8(0b11111100),
  _BMP8(0b11110000),
  _BMP8(0b11000000),
#endif

#if _MARKER_SIZE_ == 0
#define MARKER_WIDTH       7
#define MARKER_HEIGHT     10
#define X_MARKER_OFFSET    3
#define Y_MARKER_OFFSET   10
#define MARKER_BITMAP(i)  (&marker_bitmap[(i)*MARKER_HEIGHT])
static const uint8_t marker_bitmap[]={
// Marker Back plate
  _BMP8(0b11111110),
  _BMP8(0b11111110),
  _BMP8(0b11111110),
  _BMP8(0b11111110),
  _BMP8(0b11111110),
  _BMP8(0b11111110),
  _BMP8(0b11111110),
  _BMP8(0b01111100),
  _BMP8(0b00111000),
  _BMP8(0b00010000),
  // Marker 1
  _BMP8(0b00000000),
  _BMP8(0b00010000),
  _BMP8(0b00110000),
  _BMP8(0b00010000),
  _BMP8(0b00010000),
  _BMP8(0b00010000),
  _BMP8(0b00111000),
  _BMP8(0b00000000),
  _BMP8(0b00000000),
  _BMP8(0b00000000),
#if MARKERS_MAX >=2
  // Marker 2
  _BMP8(0b00000000),
  _BMP8(0b00111000),
  _BMP8(0b01000100),
  _BMP8(0b00000100),
  _BMP8(0b00111000),
  _BMP8(0b01000000),
  _BMP8(0b01111100),
  _BMP8(0b00000000),
  _BMP8(0b00000000),
  _BMP8(0b00000000),
#endif
#if MARKERS_MAX >=3
  // Marker 3
  _BMP8(0b00000000),
  _BMP8(0b00111000),
  _BMP8(0b01000100),
  _BMP8(0b00011000),
  _BMP8(0b00000100),
  _BMP8(0b01000100),
  _BMP8(0b00111000),
  _BMP8(0b00000000),
  _BMP8(0b00000000),
  _BMP8(0b00000000),
#endif
#if MARKERS_MAX >=4
  // Marker 4
  _BMP8(0b00000000),
  _BMP8(0b00001000),
  _BMP8(0b00011000),
  _BMP8(0b00101000),
  _BMP8(0b01001000),
  _BMP8(0b01001000),
  _BMP8(0b01111100),
  _BMP8(0b00001000),
  _BMP8(0b00000000),
  _BMP8(0b00000000),
#endif
#if MARKERS_MAX >=5
  // Marker 5
  _BMP8(0b00000000),
  _BMP8(0b01111100),
  _BMP8(0b01000000),
  _BMP8(0b01111000),
  _BMP8(0b00000100),
  _BMP8(0b01000100),
  _BMP8(0b00111000),
  _BMP8(0b00000000),
  _BMP8(0b00000000),
  _BMP8(0b00000000),
#endif
#if MARKERS_MAX >=6
  // Marker 6
  _BMP8(0b00000000),
  _BMP8(0b00111100),
  _BMP8(0b01000000),
  _BMP8(0b01111000),
  _BMP8(0b01000100),
  _BMP8(0b01000100),
  _BMP8(0b00111000),
  _BMP8(0b00000000),
  _BMP8(0b00000000),
  _BMP8(0b00000000),
#endif
#if MARKERS_MAX >=7
  // Marker 7
  _BMP8(0b00000000),
  _BMP8(0b01111100),
  _BMP8(0b01000100),
  _BMP8(0b00000100),
  _BMP8(0b00001000),
  _BMP8(0b00010000),
  _BMP8(0b00010000),
  _BMP8(0b00010000),
  _BMP8(0b00000000),
  _BMP8(0b00000000),
#endif
#if MARKERS_MAX >=8
  // Marker 8
  _BMP8(0b00000000),
  _BMP8(0b00111000),
  _BMP8(0b01000100),
  _BMP8(0b00111000),
  _BMP8(0b01000100),
  _BMP8(0b01000100),
  _BMP8(0b00111000),
  _BMP8(0b00000000),
  _BMP8(0b00000000),
  _BMP8(0b00000000),
#endif
};

#elif _MARKER_SIZE_ == 1
#define MARKER_WIDTH       11
#define MARKER_HEIGHT      14
#define X_MARKER_OFFSET     5
#define Y_MARKER_OFFSET    14
#define MARKER_BITMAP(i)   (&marker_bitmap[(i)*2*MARKER_HEIGHT])
static const uint8_t marker_bitmap[]={
  // Marker Back plate
  _BMP16(0b0000000000000000),
  _BMP16(0b0111111111000000),
  _BMP16(0b0111111111000000),
  _BMP16(0b0111111111000000),
  _BMP16(0b0111111111000000),
  _BMP16(0b0111111111000000),
  _BMP16(0b0111111111000000),
  _BMP16(0b0111111111000000),
  _BMP16(0b0111111111000000),
  _BMP16(0b0111111111000000),
  _BMP16(0b0011111110000000),
  _BMP16(0b0001111100000000),
  _BMP16(0b0000111000000000),
  _BMP16(0b0000010000000000),
  // Marker 1
  _BMP16(0b1111111111100000),
  _BMP16(0b1000000000100000),
  _BMP16(0b1000011000100000),
  _BMP16(0b1000111000100000),
  _BMP16(0b1001011000100000),
  _BMP16(0b1000011000100000),
  _BMP16(0b1000011000100000),
  _BMP16(0b1000011000100000),
  _BMP16(0b1000011000100000),
  _BMP16(0b1000011000100000),
  _BMP16(0b0100111101000000),
  _BMP16(0b0010000010000000),
  _BMP16(0b0001000100000000),
  _BMP16(0b0000101000000000),
#if MARKERS_MAX >=2
  // Marker 2
  _BMP16(0b1111111111100000),
  _BMP16(0b1000000000100000),
  _BMP16(0b1000111100100000),
  _BMP16(0b1001100110100000),
  _BMP16(0b1001100110100000),
  _BMP16(0b1000000110100000),
  _BMP16(0b1000001100100000),
  _BMP16(0b1000111000100000),
  _BMP16(0b1001100000100000),
  _BMP16(0b1001100000100000),
  _BMP16(0b0101111101000000),
  _BMP16(0b0010000010000000),
  _BMP16(0b0001000100000000),
  _BMP16(0b0000101000000000),
#endif
#if MARKERS_MAX >=3
  // Marker 3
  _BMP16(0b1111111111100000),
  _BMP16(0b1000000000100000),
  _BMP16(0b1001111100100000),
  _BMP16(0b1011000110100000),
  _BMP16(0b1011000110100000),
  _BMP16(0b1000000110100000),
  _BMP16(0b1000011100100000),
  _BMP16(0b1000000110100000),
  _BMP16(0b1011000110100000),
  _BMP16(0b1011000110100000),
  _BMP16(0b0101111101000000),
  _BMP16(0b0010000010000000),
  _BMP16(0b0001000100000000),
  _BMP16(0b0000101000000000),
#endif
#if MARKERS_MAX >=4
  // Marker 4
  _BMP16(0b1111111111100000),
  _BMP16(0b1000000000100000),
  _BMP16(0b1000001100100000),
  _BMP16(0b1000011100100000),
  _BMP16(0b1000111100100000),
  _BMP16(0b1001101100100000),
  _BMP16(0b1011001100100000),
  _BMP16(0b1011001100100000),
  _BMP16(0b1011111110100000),
  _BMP16(0b1000001100100000),
  _BMP16(0b0100001101000000),
  _BMP16(0b0010000010000000),
  _BMP16(0b0001000100000000),
  _BMP16(0b0000101000000000),
#endif
#if MARKERS_MAX >=5
  // Marker 5
  _BMP16(0b1111111111100000),
  _BMP16(0b1000000000100000),
  _BMP16(0b1011111110100000),
  _BMP16(0b1011000000100000),
  _BMP16(0b1011000000100000),
  _BMP16(0b1011111100100000),
  _BMP16(0b1011000110100000),
  _BMP16(0b1000000110100000),
  _BMP16(0b1000000110100000),
  _BMP16(0b1011000110100000),
  _BMP16(0b0101111101000000),
  _BMP16(0b0010000010000000),
  _BMP16(0b0001000100000000),
  _BMP16(0b0000101000000000),
#endif
#if MARKERS_MAX >=6
  // Marker 6
  _BMP16(0b1111111111100000),
  _BMP16(0b1000000000100000),
  _BMP16(0b1001111100100000),
  _BMP16(0b1011000110100000),
  _BMP16(0b1011000000100000),
  _BMP16(0b1011011100100000),
  _BMP16(0b1011100110100000),
  _BMP16(0b1011000110100000),
  _BMP16(0b1011000110100000),
  _BMP16(0b1011000110100000),
  _BMP16(0b0101111101000000),
  _BMP16(0b0010000010000000),
  _BMP16(0b0001000100000000),
  _BMP16(0b0000101000000000),
#endif
#if MARKERS_MAX >=7
  // Marker 7
  _BMP16(0b1111111111100000),
  _BMP16(0b1000000000100000),
  _BMP16(0b1011111110100000),
  _BMP16(0b1011000110100000),
  _BMP16(0b1000000110100000),
  _BMP16(0b1000001100100000),
  _BMP16(0b1000011000100000),
  _BMP16(0b1000110000100000),
  _BMP16(0b1000110000100000),
  _BMP16(0b1000110000100000),
  _BMP16(0b0100110001000000),
  _BMP16(0b0010000010000000),
  _BMP16(0b0001000100000000),
  _BMP16(0b0000101000000000),
#endif
#if MARKERS_MAX >=8
  // Marker 8
  _BMP16(0b1111111111100000),
  _BMP16(0b1000000000100000),
  _BMP16(0b1001111100100000),
  _BMP16(0b1011000110100000),
  _BMP16(0b1011000110100000),
  _BMP16(0b1001111100100000),
  _BMP16(0b1011000110100000),
  _BMP16(0b1011000110100000),
  _BMP16(0b1011000110100000),
  _BMP16(0b1011000110100000),
  _BMP16(0b0101111101000000),
  _BMP16(0b0010000010000000),
  _BMP16(0b0001000100000000),
  _BMP16(0b0000101000000000),
#endif
};
#endif

static void
markmap_marker(int marker)
{
  static index_x_t prev_index_x[MARKERS_MAX];
  static index_y_t prev_index_y[MARKERS_MAX];

  if (!markers[marker].enabled)
    return;
  if (IS_TRACE_DISABLE(markers[marker].trace))
    return;
#ifdef __MARKER_CACHE__
  marker_cache_valid[marker] = false;  // force recalculation
#endif
  int idx = markers[marker].index;
  int x = trace_index_x[idx] - X_MARKER_OFFSET;
  int y = trace_index_y[markers[marker].trace][idx] - Y_MARKER_OFFSET;
  invalidate_rect(x, y, x+MARKER_WIDTH-1, y+MARKER_HEIGHT-1);
  invalidate_rect(prev_index_x[marker],
                  prev_index_y[marker],
                  prev_index_x[marker]+MARKER_WIDTH-1,
                  prev_index_y[marker]+MARKER_HEIGHT-1);
  prev_index_x[marker] = x;
  prev_index_y[marker] = y;
}

void
markmap_all_markers(void)
{
  int i;
  for (i = 0; i < MARKERS_MAX; i++) {
    if (!markers[i].enabled)
      continue;
    markmap_marker(i);
  }
  markmap_upperarea();
}

int
distance_to_index(int8_t t, uint16_t idx, int16_t x, int16_t y)
{
  x-= trace_index_x[idx];
  y-= trace_index_y[t][idx];
  return x*x + y*y;
}

int
search_nearest_index(int x, int y, int t)
{
  int min_i = -1;
  int min_d = MARKER_PICKUP_DISTANCE * MARKER_PICKUP_DISTANCE;
  int i;
  for (i = 0; i < sweep_points; i++) {
    int d = distance_to_index(t, i, x , y);
    if (d < min_d) {
      min_d = d;
      min_i = i;
    }
  }
  return min_i;
}

void
plot_into_index(measurement_t measured)
{
  int t;
//  START_PROFILE
  trace_into_index_x_array(trace_index_x, sweep_points);
  for (t = 0; t < TRACES_MAX; t++) {
    if (IS_TRACE_DISABLE(t))
      continue;
    trace_into_index_y_array(trace_index_y[t], measured[t], sweep_points);
  }
//  STOP_PROFILE
//  START_PROFILE
  mark_cells_from_index();
  markmap_all_markers();
//  STOP_PROFILE
}

static void
draw_cell(int m, int n)
{
  int x0 = m * CELLWIDTH;
  int y0 = n * CELLHEIGHT;
  int w = CELLWIDTH;
  int h = CELLHEIGHT;
  int x, y;
  int i;
  int t;
  uint16_t c;
  // Clip cell by area
  if (w > area_width - x0)
    w = area_width - x0;
  if (h > area_height - y0)
    h = area_height - y0;
  if (w <= 0 || h <= 0)
    return;
//  PULSE;
  cell_buffer = ili9341_get_cell_buffer();
  // Clear buffer ("0 : height" lines)
#if 0
  // use memset 350 system ticks for all screen calls
  // as understand it use 8 bit set, slow down on 32 bit systems
  memset(spi_buffer, LCD_BG_COLOR, (h*CELLWIDTH)*sizeof(uint16_t));
#else
  // use direct set  35 system ticks for all screen calls
#if CELLWIDTH%8 != 0
#error "CELLWIDTH % 8 should be == 0 for speed, or need rewrite cell cleanup"
#endif
  // Set LCD_BG_COLOR for 8 pixels in one cycle
  int count = h*CELLWIDTH / 8;
  uint32_t *p = (uint32_t *)cell_buffer;
  uint32_t clr = GET_PALTETTE_COLOR(LCD_BG_COLOR) | (GET_PALTETTE_COLOR(LCD_BG_COLOR) << 16);
  while (count--) {
    p[0] = clr;
    p[1] = clr;
    p[2] = clr;
    p[3] = clr;
    p += 4;
  }
#endif

// Draw grid
#if 1
  // Generate grid type list
  uint32_t trace_type = 0;

  if (IS_TRACES_ENABLED(TRACE_ACTUAL_FLAG|TRACE_STORED_FLAG|TRACE_TEMP_FLAG))
    trace_type |= RECTANGULAR_GRID_MASK;

#ifdef __HAM_BAND__
  if (config.hambands){
    c = GET_PALTETTE_COLOR(LCD_HAM_COLOR);
    for (x = 0; x < w; x++)
      if (ham_band(x+x0))
        for (y = 0; y < h; y++) cell_buffer[y * CELLWIDTH + x] = c;
  }
#endif

  c = GET_PALTETTE_COLOR(LCD_GRID_COLOR);
  // Draw rectangular plot (40 system ticks for all screen calls)
  const int step = 2;
  if (trace_type & RECTANGULAR_GRID_MASK) {
    for (x = 0; x < w; x++) {
      if (rectangular_grid_x(x + x0)) {
        for (y = 0; y < h; y+=step) cell_buffer[y * CELLWIDTH + x] = c;
      }
    }
    for (y = 0; y < h; y++) {
      if (rectangular_grid_y(y + y0)) {
        for (x = 0; x < w; x+=step)
          if ((uint32_t)(x + x0 - CELLOFFSETX) <= WIDTH)
            cell_buffer[y * CELLWIDTH + x] = c;
      }
    }
  }
#ifdef __CHANNEL_POWER__
  if (setting.measurement == M_CP||setting.measurement == M_SNR) {
    c = GET_PALTETTE_COLOR(LCD_TRIGGER_COLOR);
    cell_draw_vline(0, h, 1*WIDTH/3 - x0, c);
    cell_draw_vline(0, h, 2*WIDTH/3 - x0, c);
  }
#endif
//  PULSE;
#endif

  // Draw trigger and band splitter lines
  bool draw_trigger = (setting.trigger != T_AUTO
#ifdef __TRIGGER_TRACE__
       && setting.trigger_trace == 255
#endif
      );
  c = GET_PALTETTE_COLOR(LCD_TRIGGER_COLOR);
#ifdef __BANDS__
  if (setting.multi_band && !setting.multi_trace) {
    for (int band = 0; band < BANDS_MAX; band++) {
      band_t *b = &setting.bands[band];
      if (!b->enabled) continue;
      // Get start and stop band display position
      int x1 = trace_index_x[b->start_index == 0 ? 0 : b->start_index - 1] - x0;
      int x2 = trace_index_x[b->stop_index ] - x0;
      cell_draw_vline(0, h, x2, c); // Draw vertical band splitter
      if (draw_trigger)             // Draw trigger line
        cell_draw_hline(x1, x2, trace_into_index_y(b->level) - y0, c);
    }
    draw_trigger = false; // triggers draw in multi-band mode
  }
#endif
  // Draw line or single trigger line
  if (draw_trigger
#ifdef __DRAW_LINE__
      || setting.draw_line
#endif
     ) cell_draw_hline(0, w, trace_into_index_y(setting.trigger_level) - y0, c);

#if 1
  // Only right cells
  if (m >= (GRID_X_TEXT)/CELLWIDTH)
    cell_grid_line_info(x0, y0);
#endif

// Draw traces (50-600 system ticks for all screen calls, depend from lines
// count and size)
#if 1
  int i0 = 0;
  int i1 = 0;
  index_x_t *index_x = trace_index_x;
  search_index_range_x(x0, x0 + w, &i0, &i1);
  for (t = 0; t < TRACES_MAX; t++) {
    if (IS_TRACE_DISABLE(t))
      continue;

    c = GET_PALTETTE_COLOR(LCD_TRACE_1_COLOR + t);
    index_y_t *index_y = trace_index_y[t];
    for (i = i0; i < i1; i++) {
      int x1 = index_x[i  ] - x0;
      int x2 = index_x[i+1] - x0;
      int y1 = index_y[i  ] - y0;
      int y2 = index_y[i+1] - y0;
      cell_drawline(x1, y1, x2, y2, c);
    }
  }
#else
  for (x = 0; x < area_width; x += 6)
    cell_drawline(x - x0, 0 - y0, area_width - x - x0, area_height - y0,
                  config.trace_color[0]);
#endif
//  PULSE;
// draw marker symbols on each trace (<10 system ticks for all screen calls)
#if 1
  for (int t = TRACE_ACTUAL; t < TRACES_MAX; t++ ) {
  if (IS_TRACE_ENABLE(t)) {
    for (i = 0; i < MARKERS_MAX; i++) {
      if (!markers[i].enabled)
        continue;
      if (markers[i].trace != t) {
        continue;
      }
      int idx = markers[i].index;
      int x = trace_index_x[idx] - x0 - X_MARKER_OFFSET;
      int y = trace_index_y[t][idx] - y0 - Y_MARKER_OFFSET;
      // Check marker icon on cell
      if ((uint32_t)(x+MARKER_WIDTH ) < (CELLWIDTH  + MARKER_WIDTH ) &&
          (uint32_t)(y+MARKER_HEIGHT) < (CELLHEIGHT + MARKER_HEIGHT)){
          // Draw marker plate
          ili9341_set_foreground(LCD_TRACE_1_COLOR + t);
          cell_blit_bitmap(x, y, MARKER_WIDTH, MARKER_HEIGHT, MARKER_BITMAP(0));
          // Draw marker number
          ili9341_set_foreground(LCD_BG_COLOR);
          cell_blit_bitmap(x, y, MARKER_WIDTH, MARKER_HEIGHT, MARKER_BITMAP(i+1));
      }
    }
  }
  }
#endif
// Draw trace and marker info on the top (50 system ticks for all screen calls)
#if 1
  if (n <= (MARKERS_MAX > 4 ? 1 : 0))
    cell_draw_marker_info(x0, y0);
#endif
#ifdef __SELFTEST__
  cell_draw_test_info(x0, y0);
#endif
  //  PULSE;
#if 0
// Draw reference position (<10 system ticks for all screen calls)
  for (t = 0; t < TRACES_MAX; t++) {
    if (!trace[t].enabled)
      continue;
    uint32_t trace_type = (1 << trace[t].type);
    if (trace_type & ((1 << TRC_SMITH) | (1 << TRC_POLAR)))
      continue;
    int x = 0 - x0 + CELLOFFSETX - REFERENCE_X_OFFSET;
    if (x + REFERENCE_WIDTH >= 0 && x - REFERENCE_WIDTH < CELLWIDTH) {
      int y = HEIGHT - float2int((get_trace_refpos(t) * GRIDY)) - y0 - REFERENCE_Y_OFFSET;
      if (y + REFERENCE_HEIGHT >= 0 && y - REFERENCE_HEIGHT < CELLHEIGHT){
        ili9341_set_foreground(GET_PALTETTE_COLOR(LCD_TRACE_1_COLOR + t));
        cell_blit_bitmap(x , y, REFERENCE_WIDTH, REFERENCE_HEIGHT, reference_bitmap);
      }
    }
  }
#endif
// Need right clip cell render (25 system ticks for all screen calls)
#if 1
  if (w < CELLWIDTH) {
    pixel_t *src = cell_buffer + CELLWIDTH;
    pixel_t *dst = cell_buffer + w;
    for (y = h; --y; src += CELLWIDTH - w)
      for (x = w; x--;)
        *dst++ = *src++;
  }
#endif
  // Draw cell (500 system ticks for all screen calls)
  ili9341_bulk_continue(OFFSETX + x0, OFFSETY + y0, w, h);
}

static void
draw_all_cells(bool flush_markmap)
{
  int m, n;
//  START_PROFILE
  for (n = 0; n < (area_height+CELLHEIGHT-1) / CELLHEIGHT; n++){
    map_t update_map = markmap[0][n] | markmap[1][n];
    if (update_map == 0) continue;
    for (m = 0; update_map; update_map>>=1, m++)
      if (update_map & 1)
        draw_cell(m, n);
  }
#if 0
  // Used for debug control cell update
  ili9341_bulk_finish();
  for (m = 0; m < (area_width+CELLWIDTH-1) / CELLWIDTH; m++)
    for (n = 0; n < (area_height+CELLHEIGHT-1) / CELLHEIGHT; n++) {
      ili9341_set_background(((markmap[0][n] | markmap[1][n]) & (1 << m)) ? LCD_LOW_BAT_COLOR : LCD_NORMAL_BAT_COLOR);
      ili9341_fill(m*CELLWIDTH+OFFSETX, n*CELLHEIGHT, 2, 2);
    }
#endif
  if (flush_markmap) {
    // keep current map for update
    swap_markmap();
    // clear map for next plotting
    clear_markmap();
  }
  // Flush LCD buffer, wait completion (need call after end use ili9341_bulk_continue mode)
  ili9341_bulk_finish();
//  STOP_PROFILE
}

void
draw_all(bool flush)
{
  if (isFullScreenMode()) return;
#ifdef __LEVEL_METER__
  level_text[0] = 0;    // Clear level text
#ifdef TINYSA4
  freq_text[0] = 0;    // Clear level text
#endif
#endif
  if (redraw_request & REDRAW_AREA)       // this set all area for update
    force_set_markmap();
  else {
    if (redraw_request & REDRAW_MARKER)   // update marker area
      markmap_upperarea();
    if (redraw_request & REDRAW_TRIGGER)  // update trigger area
      markmap_trigger_area();
  }
  if (redraw_request & (REDRAW_CELLS | REDRAW_MARKER | REDRAW_AREA | REDRAW_TRIGGER)){
    draw_all_cells(flush);
#ifdef __SCROLL__
  //  START_PROFILE
  if (setting.waterfall && !(redraw_request & REDRAW_INBETWEEN))
    update_waterfall();
  //  STOP_PROFILE
#endif

#ifdef __LEVEL_METER__
  if (setting.level_meter) {
    update_level_meter();
  }
#endif
  }
  if (redraw_request & REDRAW_CAL_STATUS)
    draw_cal_status();                      // calculates the actual sweep time, must be before draw_frequencies
  if (redraw_request & REDRAW_FREQUENCY)
    draw_frequencies();
  if (redraw_request & REDRAW_BATTERY)
    draw_battery_status();
  redraw_request = 0;
}

//
// Call this function then need fast draw trigger line
// Used in ui.c for leveler move trigger (not added yet), drag trigger and etc.
void redraw_trigger(int band) {
  (void)band;
#ifdef __BANDS__
  if (is_multiband_trigger())
    markmap_band_trigger(band);
  else
#endif
    markmap_trigger();
  draw_all_cells(TRUE);
  redraw_request|= REDRAW_CAL_STATUS | REDRAW_AREA;  // Force redraw cal after
}

// Convert y display position to value in dBm
float pos_to_trigger_value(int y) {
  if (y <           0) y = 0;
  if (y > area_height) y = area_height;
  return to_dBm(get_trace_refpos() - get_trace_scale() * (float)(y * NGRIDY) / area_height);
}

// Search nearest trigger line on screen
int search_nearest_trigger(int x, int y) {
  if ((setting.trigger == T_AUTO       // Not in auto mode
      #ifdef __TRIGGER_TRACE__
       || setting.trigger_trace != 255 // Not in Trigger trace mode
      #endif
      )
      #ifdef __DRAW_LINE__
       && !setting.draw_line           // Allow in draw line mode
      #endif
  ) return -1;
#ifdef __BANDS__
  if (is_multiband_trigger()) {
    for (int band = 0; band < BANDS_MAX; band++) {
      band_t *b = &setting.bands[band];
      if (!b->enabled) continue;
      int x1 = trace_index_x[b->start_index];
      int x2 = trace_index_x[b->stop_index ];
      if (x < x1 || x > x2) continue;
      int tl = trace_into_index_y(b->level) + DRAG_TRIGGER_DISTANCE - y;
      if ((uint32_t)tl < 2 * DRAG_TRIGGER_DISTANCE) return band;
    }
    return -1;
  }
#endif
  int tl = trace_into_index_y(setting.trigger_level) + DRAG_TRIGGER_DISTANCE - y;
  if ((uint32_t)tl < 2 * DRAG_TRIGGER_DISTANCE) return 0;
  return -1;
}

//
// Call this function then need fast draw marker and marker info
// Used in ui.c for leveler move marker, drag marker and etc.
void
redraw_marker(int marker)
{
  if (marker < 0)
    return;
  // mark map on new position of marker
  markmap_marker(marker);

  // mark cells on marker info
  markmap_upperarea();

  draw_all_cells(TRUE);
  // Force redraw all area after (disable artifacts after fast marker update area)
  redraw_request|=REDRAW_AREA;
}

#if 0  // Not used
void
request_to_draw_cells_behind_menu(void)
{
  // Values Hardcoded from ui.c
  if (current_menu_is_form())
    invalidate_rect(0, 0, LCD_WIDTH-1, LCD_HEIGHT-1);
  else
    invalidate_rect(LCD_WIDTH-MENU_BUTTON_WIDTH-OFFSETX, 0, LCD_WIDTH-OFFSETX, LCD_HEIGHT-1);
  redraw_request |= REDRAW_CELLS;
}

void
request_to_draw_cells_behind_numeric_input(void)
{
  // Values Hardcoded from ui.c
  invalidate_rect(0, LCD_HEIGHT-NUM_INPUT_HEIGHT, LCD_WIDTH-1, LCD_HEIGHT-1);
  redraw_request |= REDRAW_CELLS;
}
#endif

static void
cell_blit_bitmap(int x, int y, uint16_t w, uint16_t h, const uint8_t *bmp)
{
  if (x <= -w)
    return;
  uint8_t bits = 0;
  int c = h+y, r;
  for (; y < c; y++) {
    for (r = 0; r < w; r++, bits<<=1) {
      if ((r&7)==0) bits = *bmp++;
      if ((0x80 & bits) == 0) continue;    // no pixel
      if ((uint32_t)(y+0) >= CELLHEIGHT) continue; // y   < 0 || y   >= CELLHEIGHT
      if ((uint32_t)(x+r) >= CELLWIDTH ) continue; // x+r < 0 || x+r >= CELLWIDTH
      cell_buffer[y*CELLWIDTH + x + r] = foreground_color;
    }
  }
}

#ifndef wFONT_GET_DATA
static int
cell_drawchar_size(uint8_t ch, int x, int y, int size)
{
  uint8_t bits;
  int c, r, ch_size;
  const uint8_t *char_buf = FONT_GET_DATA(ch);
  ch_size = FONT_GET_WIDTH(ch);
  //  if (y <= -FONT_GET_HEIGHT || y >= CELLHEIGHT || x <= -ch_size || x >= CELLWIDTH)
  //    return ch_size;
  if (x <= -ch_size*size)
    return ch_size*size;
  for (c = 0; c < FONT_GET_HEIGHT; c++) {
    for (int i=0; i < size; i++) {
      bits = *char_buf;
      if ((y + c*size+i) < 0 || (y + c*size+i) >= CELLHEIGHT)
        continue;
      for (r = 0; r < ch_size; r++) {
        for (int j = 0; j < size; j++) {
          if ((x+r*size + j) >= 0 && (x+r*size+j) < CELLWIDTH && (0x80 & bits))
            cell_buffer[(y+c*size+i)*CELLWIDTH + (x+r*size+j)] = foreground_color;
        }
        bits <<= 1;
      }
    }
    char_buf++;
  }
  return ch_size*size;
}
#endif

typedef struct {
  const void *vmt;
  int16_t x;
  int16_t y;
} screenPrintStream;

static msg_t cellPut(void *ip, uint8_t ch) {
  screenPrintStream *ps = ip;
  if (ps->x < CELLWIDTH){
    uint16_t w = FONT_GET_WIDTH(ch);
    cell_blit_bitmap(ps->x, ps->y, w, FONT_GET_HEIGHT, FONT_GET_DATA(ch));
    ps->x+= w;
  }
  return MSG_OK;
}

static msg_t cellPut7x13(void *ip, uint8_t ch) {
  screenPrintStream *ps = ip;
  if (ps->x < CELLWIDTH){
    uint16_t w = bFONT_GET_WIDTH(ch);
    cell_blit_bitmap(ps->x, ps->y, w, bFONT_GET_HEIGHT, bFONT_GET_DATA(ch));
    ps->x+= w;
  }
  return MSG_OK;
}

//#define ENABLE_WIDE_FONT_ON_CELL
#ifdef ENABLE_WIDE_FONT_ON_CELL
static msg_t cellPut10x14(void *ip, uint8_t ch) {
  screenPrintStream *ps = ip;
  if (ps->x < CELLWIDTH){
#ifdef wFONT_GET_DATA
    uint16_t w = wFONT_GET_WIDTH(ch);
    cell_blit_bitmap(ps->x, ps->y, w <=8 ? 9 : w, wFONT_GET_HEIGHT, wFONT_GET_DATA(ch));
#else
    w = cell_drawchar_size( ch, ps->x, ps->y, 2);
#endif
    ps->x+= w;
  }
  return MSG_OK;
}
#endif

// Simple print in buffer function
int cell_printf(int16_t x, int16_t y, const char *fmt, ...) {
  // skip always if right
  if (x>=CELLWIDTH) return 0;
  // Init small cell print stream
  struct cellprintStreamVMT {
    _base_sequential_stream_methods
  } cell_vmt = {NULL, NULL, NULL, NULL};
  screenPrintStream ps = {&cell_vmt, x, y};
  // Select font and skip print if not on cell (at top/bottom)
  switch (*fmt++){
    case _FONT_s:
      if ((uint32_t)(y+FONT_GET_HEIGHT) >= CELLHEIGHT + FONT_GET_HEIGHT) return 0;
      cell_vmt.put = cellPut;
      break;
#ifdef ENABLE_WIDE_FONT_ON_CELL
    case _FONT_w:
      if ((uint32_t)(y+FONT_GET_HEIGHT) >= CELLHEIGHT + FONT_GET_HEIGHT) return 0;
      cell_vmt.put = cellPut10x14;
      break;
#endif
    default:
      fmt--;
      __attribute__ ((fallthrough)); // prevent warning
    case _FONT_b:
      if ((uint32_t)(y+bFONT_GET_HEIGHT) >= CELLHEIGHT + bFONT_GET_HEIGHT) return 0;
      cell_vmt.put = cellPut7x13;
      break;
  }
  // Performing the print operation using the common code.
  va_list ap;
  va_start(ap, fmt);
  int retval = chvprintf((BaseSequentialStream *)(void *)&ps, fmt, ap);
  va_end(ap);
  // Return number of bytes that would have been written.
  return retval;
}

extern float temppeakLevel;

static void cell_grid_line_info(int x0, int y0)
{
  int xpos = GRID_X_TEXT - x0;
  int ypos = 0 - y0 + 2;
  ili9341_set_foreground(LCD_GRID_VALUE_COLOR);
  float   ref = get_trace_refpos();
  float scale = get_trace_scale();
  for (int i = 0; i < NGRIDY; i++){
    if (ypos >= CELLHEIGHT) break;
    cell_printf(xpos, ypos, FONT_s"% 7.3F", ref);
    ypos+=GRIDY;
    ref-=scale;
  }
}

static void trace_print_value_string(     // Only used at one place
    int xpos, int ypos,
    bool bold,
    int mi  // Marker number
    )
{
  (void) bold;
  int ref_marker=-1;
  int mtype = markers[mi].mtype;
  int   idx = markers[mi].index;
  float v   = marker_to_value(mi);
  char buf2[24];
  char *ptr2 = buf2;
  // Prepare marker type string
  *ptr2++ = mi == active_marker ? S_SARROW[0] : ' ';
  *ptr2++ = mi+'1';
//  if (mtype & M_REFERENCE)
//    *ptr2++  = 'R';
  if (mtype & M_TRACKING)
    *ptr2++  = 'T';
  if (mtype & M_DELTA) {
    *ptr2++ = S_DELTA[0];
    *ptr2++  = markers[mi].ref+'1';
  }
//  if (mtype & M_NOISE)
//    *ptr2++  = 'N';
  *ptr2++ =  ' ';
  // Not possible ???
  if (v == -INFINITY){
    cell_printf(xpos, ypos, FONT_b"%s-INF", buf2);
    return;
  }
  // Prepare output frequency and value
  freq_t freq = markers[mi].frequency;
  int unit_index = setting.unit;
  // Setup delta values
  if (mtype & M_DELTA) {
    ref_marker = markers[mi].ref;
//    *ptr2++ = S_DELTA[0];
    unit_index+= MAX_UNIT_TYPE;
    freq_t  ref_freq = markers[ref_marker].frequency;
    int ridx = markers[ref_marker].index;
    if (ridx > idx) {idx = ridx - idx; *ptr2++ = '-';}
    else            {idx = idx - ridx; *ptr2++ = '+';}
    if (ref_freq > freq) {freq = ref_freq - freq;}
    else                 {freq = freq - ref_freq;}
    v-= marker_to_value(ref_marker);
  } else
    freq += (setting.frequency_offset - FREQUENCY_SHIFT);

  int negative = (freq > HALF_FREQ);
  if (negative) {
    freq = - freq;
    *ptr2++ = '-';
  }

  // For CW mode output time
  if (FREQ_IS_CW()) {
    plot_printf(ptr2, sizeof(buf2) - 9, "%.3Fs", idx*setting.actual_sweep_time_us/(float)((sweep_points - 1)*ONE_SECOND_TIME));
  } else {
    freq_t step = getFrequency(1)-getFrequency(0);
    int digits = 1;
    if (freq>1000000 && step != 0) {
      while (step < 1000000 && digits<5) {
        digits++;
        step = step*10;
      }
    }
#ifdef TINYSA4
    if (freq >= 1000000000)
      digits += 3;
#endif
    plot_printf(ptr2, sizeof(buf2) - 9, "%9.*QHz", digits, freq);
  }
#ifdef __LEVEL_METER__
#ifdef TINYSA4
  if (freq_text[0] == 0) strcpy(freq_text, &ptr2[0]);
#endif
#endif
  const char *format;
  if (UNIT_IS_LINEAR(setting.unit))
    format = FONT_s"%s %.3F%s%s%s%s"; // 5 characters incl u, m, etc...
  else
    format = FONT_s"%s %.1f%s%s%s%s";
#ifdef TINYSA4
  format++; // Skip small prefix for bold output
#else
  if (bold) format++; // Skip small prefix for bold output
#endif
  cell_printf(xpos, ypos, format, buf2, v, unit_string[unit_index], (mtype & M_DELTA?"c":"") , (mtype & M_NOISE?"/Hz":""), (mtype & M_AVER?"/T":""));
#ifdef __LEVEL_METER__
  if (level_text[0] == 0){
    plot_printf(level_text, sizeof(level_text), &format[3], v, unit_string[unit_index], (mtype & M_DELTA?"c":"") , (mtype & M_NOISE?"/Hz":"") ,(mtype & M_AVER?"/T":""));
  }
#endif
}

static void cell_draw_marker_info(int x0, int y0)
{
  int t;
//  int ref_marker = 0;
  int j = 0;
//  int count = 0;
  int active=0;
  for (int i = 0; i < MARKER_COUNT; i++) {
    if (markers[i].enabled) {
//      if (markers[i].mtype & M_REFERENCE) {
//        ref_marker = i;
//      }
      active++;
    }
  }
#ifdef __CHANNEL_POWER__
  if (setting.measurement==M_CP || setting.measurement==M_SNR) {
    ili9341_set_foreground(LCD_FG_COLOR);
    freq_t bw = get_sweep_frequency(ST_SPAN)/3;
    for (int c=0; c<3;c++) {
      int xpos = 10 + (c)*(WIDTH/3) + CELLOFFSETX - x0;
      int ypos = 1 - y0;
      cell_printf(xpos, ypos, FONT_b"%4.1fdBm/%3QHz", channel_power[c], bw);
      ypos = 14 - y0;
      if (setting.measurement==M_CP ) {
        float v = 100.0 * channel_power_watt[c] /(channel_power_watt[0] + channel_power_watt[1] + channel_power_watt[2]);
        cell_printf(xpos, ypos, FONT_b"%4.1f%%", v );
#ifdef __LEVEL_METER__
        if (c == 1)
           plot_printf(level_text, sizeof(level_text), "%4.1f%%", v);
#endif
      }
      else if (c == 1) {
        float v = channel_power[1] - (channel_power[0] + channel_power[2])/2;
        cell_printf(xpos, ypos, FONT_b"SNR: %4.1fdB", v);
#ifdef __LEVEL_METER__
        plot_printf(level_text, sizeof(level_text), "%4.1f", v);
#endif
      }
    }
    return;
  }
#endif
  if (setting.measurement == M_THD && active >= 1)
    active = 2;
  for (int i = 0; i < MARKER_COUNT; i++) {
    if (i == 3) {
      if (setting.measurement == M_PASS_BAND) {
        freq_t f;
        if (markers[2].frequency>markers[1].frequency)
          f = markers[2].frequency-markers[1].frequency;
        else
          f = markers[1].frequency-markers[2].frequency;

        j = 3;
        int xpos = 1 + (j%2)*(WIDTH/2) + CELLOFFSETX - x0;
        int ypos = 1 + (j/2)*(16) - y0;
        cell_printf(xpos, ypos, FONT_b"WIDTH: %8.3QHz", f);
#ifdef __LEVEL_METER__
        plot_printf(level_text, sizeof(level_text), "%8.3Q", f);
#endif
        //        cell_drawstring(buf, xpos, ypos);
      } else if (setting.measurement == M_AM){
#ifdef AM_IN_VOLT
        int old_unit = setting.unit;
        setting.unit = U_VOLT;
        float level = (marker_to_value(1) + marker_to_value(2))/2 / marker_to_value(0);
        setting.unit = old_unit;
        int depth = (int)( level * 2.0 * 80.0) + 20;
#else
        float delta = actual_t[markers[1].index] - actual_t[markers[2].index];
        if (delta < -5 || delta > 5)
          break;
        float level = (actual_t[markers[1].index] + actual_t[markers[2].index])/2.0 -  actual_t[markers[0].index];
        if (level < -70 || level > 0)
          break;
//      int depth = powf(10.0, 2.0 + (level + 6.02)/20.0 ); //
//                  powf(10.0, level/20.0 + 6.02/20.0 + 2.0)
//                  powf(10.0, level            /20.0 ) *  powf(10.0, 6.02/20.0 + 2.0);
        int depth =       expf(level*(logf(10.0)/20.0)) * (powf(10.0, 6.02/20.0 + 2.0));
#endif
        j = 3;
        int xpos = 1 + (j%2)*(WIDTH/2) + CELLOFFSETX - x0;
        int ypos = 1 + (j/2)*(16) - y0;
        cell_printf(xpos, ypos, FONT_b"DEPTH: %3d%%", depth);
#ifdef __LEVEL_METER__
        plot_printf(level_text, sizeof(level_text), "%3d%%", depth);
#endif
      } else if (setting.measurement == M_FM){
        freq_t dev = markers[1].frequency + actual_rbw_x10*100.0;      // Temp value to prevent calculation of negative deviation
        if ( markers[2].frequency < dev)
          break;
        dev = ( markers[2].frequency - dev ) >> 1;
        j = 3;
        int xpos = 1 + (j%2)*(WIDTH/2) + CELLOFFSETX - x0;
        int ypos = 1 + (j/2)*(16) - y0;
        cell_printf(xpos, ypos, FONT_b"DEVIATION:%6.1QHz", dev);
#ifdef __LEVEL_METER__
        plot_printf(level_text, sizeof(level_text), "%6.1Q", dev);
#endif
#ifdef TINYSA4
#define THD_SHIFT   7
#else
#define THD_SHIFT   6
#endif
      } else if (setting.measurement == M_THD && markers[0].enabled && (markers[0].index << THD_SHIFT) > sweep_points ) {
        int old_unit = setting.unit;
        setting.unit = U_WATT;
        float p = marker_to_value(0);
        int h_i = 2;
        freq_t f = markers[0].frequency;
        float h = 0.0;
        int h_count = 0;
        int search_gate = 10;
        if (markers[0].index < search_gate)
          search_gate = markers[0].index * 2 / 3;
        freq_t stop = getFrequency(sweep_points-1);
        while (f * h_i < stop) {
          if (search_maximum(1, f*h_i, search_gate) ) {            // use marker 1 for searching harmonics
            h_count++;
            f = (f * 3 + markers[1].frequency / h_i) >> 2;
            h += marker_to_value(1);
          }
          h_i++;
        }
        float thd = 100.0 * sa_sqrtf(h/p);
        setting.unit = old_unit;
        ili9341_set_foreground(marker_color(0));
//        j = 1;
        int xpos = 1 + (j%2)*(WIDTH/2) + CELLOFFSETX - x0;
        int ypos = 1 + (j/2)*(16) - y0;
        cell_printf(xpos, ypos, FONT_b"#%d THD: %4.1f%%", h_count, thd);
#ifdef __LEVEL_METER__
        plot_printf(level_text, sizeof(level_text), "%4.1f%%", thd);
#endif
        break;
      }
#ifdef __NOISE_FIGURE__
    } else if (i>=2 && (setting.measurement == M_NF_TINYSA || setting.measurement == M_NF_VALIDATE || setting.measurement == M_NF_AMPLIFIER) && markers[0].enabled) {
      float aNP = 0;
      aNP = marker_to_value(0);
      measured_noise_figure = aNP + 173.93 - nf_gain;   // measured noise figure at 20C
      if (setting.measurement != M_NF_TINYSA) {
        float mnf = expf(measured_noise_figure/10.0 * logf(10));     // measure noise factor
        float tnf = expf(config.noise_figure/10.0 * logf(10.0));     // tinySA noise factor
        float amp_gain = expf(nf_gain/10.0 * logf(10.0));
        float anf = mnf - (tnf - 1.0)/amp_gain;
        measured_noise_figure = 10.0*logf(anf)/logf(10.0);
      }
      // powf(10,x) =  expf(x * logf(10))
      // log10f(x)  =  logf(x)/logf(10)


      ili9341_set_foreground(marker_color(0));
//        j = 1;
      int xpos = 1 + (j%2)*(WIDTH/2) + CELLOFFSETX - x0;
      int ypos = 1 + (j/2)*(16) - y0;
      if (setting.measurement == M_NF_TINYSA) {
        cell_printf(xpos, ypos, FONT_b"TINYSA NF: %4.1f", measured_noise_figure);
      } else {
        if (setting.measurement == M_NF_VALIDATE)
          cell_printf(xpos, ypos, FONT_b"TINYSA NF ERROR: %4.1f", measured_noise_figure);
        else
          cell_printf(xpos, ypos, FONT_b"GAIN: %4.1fdB   NF: %4.1f", nf_gain, measured_noise_figure);
      }
#ifdef __LEVEL_METER__
        plot_printf(level_text, sizeof(level_text), "%4.1f", measured_noise_figure);
#endif

      break;
#endif
    } else
    if (i >= 2 && setting.measurement == M_OIP3 && markers[2].enabled && markers[3].enabled) {
      float il = marker_to_value(2);
      float ir = marker_to_value(3);
      float sl = marker_to_value(0);
      float sr = marker_to_value(1);

      float ip = sl+ (sr - il)/2;
      j = 2;
      int xpos = 1 + (j%2)*(WIDTH/2) + CELLOFFSETX - x0;
      int ypos = 1 + (j/2)*(16) - y0;
      cell_printf(xpos, ypos, FONT_s"OIP3: %4.1fdB", ip);
#ifdef __LEVEL_METER__
      plot_printf(level_text, sizeof(level_text), "%4.1f", ip);
#endif


      ip = sr+ (sl - ir)/2;
      j = 3;
      xpos = 1 + (j%2)*(WIDTH/2) + CELLOFFSETX - x0;
      ypos = 1 + (j/2)*(16) - y0;
      cell_printf(xpos, ypos, FONT_s"OIP3: %4.1fdB", ip);
      break;
    }
#if 0
    if (i >= 2 && in_selftest) {
      j = 2;
      int xpos = 1 + CELLOFFSETX +25 - x0;
      int ypos = 1 + 16 - y0;

      cell_printf(xpos, ypos, FONT_b"DO NOT SWITCH OFF!!");
      break;
    }
#endif
    if (!markers[i].enabled)
      continue;
    for (t = TRACE_ACTUAL; t < TRACES_MAX; t++) { // Only show info on actual trace
      if (IS_TRACE_DISABLE(t))
        continue;
      if (markers[i].trace != t) {
        continue;
      }
      uint16_t color;
      int level = temppeakLevel - get_attenuation() + setting.external_gain;
      if ((!setting.normalized[t] && !setting.subtract[t]) &&     // Disabled when normalized or subtract
          ((setting.mode == M_LOW  &&
              (
#ifdef TINYSA4
                  setting.extra_lna ? level > -33 :
#endif
                      level > -10)
                  )
           || (setting.mode == M_HIGH && level > -29)
           || (setting.mode == M_LOW && (markers[i].mtype & M_NOISE) && vbwSteps > 1))   //MAXPEAK increases noise marker, should reduce span.
           )
        color = LCD_BRIGHT_COLOR_RED;
      else
        color = marker_color(i);
      ili9341_set_foreground(color);
      ili9341_set_background(LCD_BG_COLOR);
#if 1
      int xpos = 1 + (j%2)*(WIDTH/2) + CELLOFFSETX - x0;
      int ypos = 1 + (j/2)*(16) - y0;
#else
      int xpos = 1 + CELLOFFSETX - x0;
      int ypos = 1 + j*(FONT_GET_HEIGHT*2+1) - y0;
#endif
      trace_print_value_string(xpos, ypos, active == 1, i);
      j++;
   }
  }
}

void
draw_frequencies(void)
{
  char buf1[40];
  char buf2[32];
  if (MODE_OUTPUT(setting.mode))     // No frequencies during output
    return;
  if (current_menu_is_form() && !in_selftest)
    return;
#ifdef __BANDS__
  if (setting.multi_band && !setting.multi_trace) {
    ili9341_set_foreground(LCD_FG_COLOR);
    ili9341_set_background(LCD_BG_COLOR);
    ili9341_fill(FREQUENCIES_XPOS1, CHART_BOTTOM + 1, LCD_WIDTH - FREQUENCIES_XPOS1, LCD_HEIGHT - CHART_BOTTOM - 1);
    int idx=0;
    do {
      int next_idx = idx;
      int b = getBand(idx);
      while (b == getBand(++next_idx));
//      plot_printf(buf1, sizeof(buf1), "%.3QHz-%.3QHz %5.1QHz/", setting.bands[b].start + (setting.frequency_offset - FREQUENCY_SHIFT),  setting.bands[b].end + (setting.frequency_offset - FREQUENCY_SHIFT), grid_span);
      if (setting.bands[b].name[0])
        plot_printf(buf1, sizeof(buf1), "%s", setting.bands[b].name);
      else {
        if (next_idx - idx < sweep_points/4-20)
          plot_printf(buf1, sizeof(buf1), "%.3QHz", (setting.bands[b].start+setting.bands[b].end)/2 + (setting.frequency_offset - FREQUENCY_SHIFT), grid_span);
        else
          plot_printf(buf1, sizeof(buf1), "%.3QHz %5.1QHz/", setting.bands[b].start + (setting.frequency_offset - FREQUENCY_SHIFT), grid_span);
      }
      ili9341_drawstring(buf1, FREQUENCIES_XPOS1+idx, FREQUENCIES_YPOS);
      idx = next_idx;
    } while (idx < sweep_points - 10);
    return;
  }
#endif
  char *shift = (setting.frequency_offset == FREQUENCY_SHIFT ? "" : "shifted");
  if (FREQ_IS_CW()) {
    plot_printf(buf1, sizeof(buf1), " CW %QHz", get_sweep_frequency(ST_CW) + (setting.frequency_offset - FREQUENCY_SHIFT));
    // Show user actual select sweep time?
    uint32_t t = setting.actual_sweep_time_us;
    plot_printf(buf2, sizeof(buf2), " TIME %.3Fs", (float)t/ONE_SECOND_TIME);

  } else if (FREQ_IS_STARTSTOP()) {
    plot_printf(buf1, sizeof(buf1), " START %.3QHz    %5.1QHz/ %s", get_sweep_frequency(ST_START) + (setting.frequency_offset - FREQUENCY_SHIFT), grid_span, shift);
    plot_printf(buf2, sizeof(buf2), " STOP %.3QHz", get_sweep_frequency(ST_STOP) + (setting.frequency_offset - FREQUENCY_SHIFT));
  } else if (FREQ_IS_CENTERSPAN()) {
    plot_printf(buf1, sizeof(buf1), " CENTER %.3QHz    %5.1QHz/ %s", get_sweep_frequency(ST_CENTER) + (setting.frequency_offset - FREQUENCY_SHIFT), grid_span, shift);
    plot_printf(buf2, sizeof(buf2), " SPAN %.3QHz", get_sweep_frequency(ST_SPAN));
  }
  ili9341_set_foreground(LCD_FG_COLOR);
  ili9341_set_background(LCD_BG_COLOR);
  ili9341_fill(FREQUENCIES_XPOS1, CHART_BOTTOM + 1, LCD_WIDTH - FREQUENCIES_XPOS1, LCD_HEIGHT - CHART_BOTTOM - 1);
  if (uistat.lever_mode == LM_CENTER)
    buf1[0] = S_SARROW[0];
  if (uistat.lever_mode == LM_SPAN)
    buf2[0] = S_SARROW[0];
//  int p2 = FREQUENCIES_XPOS2;
//  if (FREQ_IS_CW()) {
    int p2 = LCD_WIDTH - FONT_WIDTH*strlen(buf2);
//  }
  ili9341_drawstring(buf2, p2, FREQUENCIES_YPOS);
  ili9341_drawstring(buf1, FREQUENCIES_XPOS1, FREQUENCIES_YPOS);
  ili9341_set_foreground(LCD_BRIGHT_COLOR_RED);
#ifdef TINYSA4
  if (get_sweep_frequency(ST_STOP) > 2000000000ULL && setting.attenuate_x2 >= 16 ) {
    ili9341_drawstring("REDUCED LINEARITY", p2 - 18*7, FREQUENCIES_YPOS);
  }// else
   // ili9341_drawstring("                 ", p2 - 18*7, FREQUENCIES_YPOS);
#endif
}

// Draw battery level
#define BATTERY_TOP_LEVEL       4200
#define BATTERY_BOTTOM_LEVEL    3300
#define BATTERY_WARNING_LEVEL   3300
#define BATTERY_MID_LEVEL       3900

static void draw_battery_status(void)
{
#ifdef  __USE_SD_CARD__
static const uint8_t sd_icon [] = {
  _BMP16(0b1111111111111000),  //
  _BMP16(0b1000000000001000),  // 1
  _BMP16(0b1000000000001000),  // 2
  _BMP16(0b1000000000001000),  // 3
  _BMP16(0b1001100111001000),  // 4
  _BMP16(0b1010010100101000),  // 5
  _BMP16(0b1001000100101000),  // 6
  _BMP16(0b1000100100101000),  // 7
  _BMP16(0b1010010100101000),  // 8
  _BMP16(0b1001100111001000),  // 9
  _BMP16(0b0100000000001000),  //10
  _BMP16(0b0100000000001000),  //11
  _BMP16(0b1100000000001000),  //12
  _BMP16(0b1000000000001000),  //13
  _BMP16(0b0101010101011000),  //14
  _BMP16(0b0111111111111000)   //
  };
  if (SD_Inserted()){
    ili9341_set_foreground(LCD_BRIGHT_COLOR_GREEN);
    ili9341_blitBitmap(4, SD_CARD_START, 16, 16, sd_icon);
//  ili9341_drawstring("-SD-", x, SD_CARD_START);
  }
  else{
    ili9341_set_background(LCD_BG_COLOR);
    ili9341_fill(4, SD_CARD_START, 16, 16);
    SD_PowerOff();
#ifdef __DISABLE_HOT_INSERT__
    sd_card_inserted_at_boot = false;
#endif
  }
#endif
  int16_t vbat = adc_vbat_read();
  if (vbat <= 0)
    return;
  uint8_t string_buf[24];
  // Set battery color
  ili9341_set_foreground(vbat < BATTERY_WARNING_LEVEL ? LCD_LOW_BAT_COLOR : (vbat < BATTERY_MID_LEVEL ? LCD_TRACE_1_COLOR : LCD_NORMAL_BAT_COLOR));
  ili9341_set_background(LCD_BG_COLOR);

  // Prepare battery bitmap image
  // Battery top
  int x = 0;
  string_buf[x++] = 0b00000000;
  string_buf[x++] = 0b00111100;
  string_buf[x++] = 0b00111100;
  string_buf[x++] = 0b11111111;
  // Fill battery status
  for (int power=BATTERY_TOP_LEVEL; power > BATTERY_BOTTOM_LEVEL; ){
    if ((x&3) == 0) {string_buf[x++] = 0b10000001; continue;}
    string_buf[x++] = (power > vbat) ? 0b10000001 : // Empty line
                                       0b10111101;  // Full line
    power-=100;
  }
  // Battery bottom
  string_buf[x++] = 0b10000001;
  string_buf[x++] = 0b11111111;
  // Draw battery
  ili9341_blitBitmap(7, BATTERY_START, 8, x, string_buf);
  plot_printf((char*)string_buf, sizeof string_buf, "%.2fv", vbat/1000.0);
  ili9341_drawstring((char*)string_buf, 1, BATTERY_START+x+3);
}

void
request_to_redraw_grid(void)
{
  redraw_request |= REDRAW_AREA;
}

void
redraw_frame(void)
{
  ili9341_set_background(LCD_BG_COLOR);
  ili9341_clear_screen();
  draw_frequencies();
  draw_cal_status();
}

int display_test_pattern(pixel_t p)
{
  // write and read display, return false on fail.
  for (int h = 0; h < LCD_HEIGHT; h++) {
    // write test pattern to LCD
    for (int w = 0; w < LCD_WIDTH; w++)
      spi_buffer[w] = p;
    ili9341_bulk(0, h, LCD_WIDTH, 1);
    // Cleanup buffer
    memset(spi_buffer, 0, LCD_WIDTH * sizeof(pixel_t));
    // try read data
    ili9341_read_memory(0, h, LCD_WIDTH, 1, spi_buffer);
    // Check pattern from data    for (volatile int w = 0; w < LCD_WIDTH; w++)
    for (int w = 0; w < LCD_WIDTH; w++)
      if (spi_buffer[w] != p)
        return false;
  }
  return true;
}

int display_test(void)
{
#if 0
  if (!display_test_pattern(RGB565(0,0,0)))
      return false;
  if (!display_test_pattern(RGB565(255,255,255)))
      return false;
  if (!display_test_pattern(RGB565(255,0,0)))
      return false;
  if (!display_test_pattern(RGB565(0,255,0)))
      return false;
  if (!display_test_pattern(RGB565(0,0,255)))
      return false;
  if (!display_test_pattern(0xd200))
      return false;
#endif
  for (int h = 0; h < LCD_HEIGHT; h++) {
    // write test pattern to LCD
    for (int w = 0; w < LCD_WIDTH; w++)
      spi_buffer[w] = h*w; //(h<<8)+w;
    ili9341_bulk(0, h, LCD_WIDTH, 1);
    // Cleanup buffer
    memset(spi_buffer, 0, LCD_WIDTH * sizeof(pixel_t));
    // try read data
    ili9341_read_memory(0, h, LCD_WIDTH, 1, spi_buffer);
    // Check pattern from data
    for (volatile int w = 0; w < LCD_WIDTH; w++)
      if (spi_buffer[w] != (pixel_t) (h*w) )// ((h<<8)+w)) // WARNING: Comparison fails without typecast of (w*h) to pixel_t
        return false;
  }
  return true;
}

#define _USE_WATERFALL_PALETTE
#ifdef  _USE_WATERFALL_PALETTE
#include "waterfall.c"
#endif

static void update_waterfall(void){
  int i = CHART_BOTTOM-1;
  int w_width = area_width < WIDTH ? area_width : WIDTH;
//  START_PROFILE;
#ifdef TINYSA4
#define WATERFALL_MULTI (SPI_BUFFER_SIZE / WIDTH)
  if (!auto_capture) {
    for (i = CHART_BOTTOM-WATERFALL_MULTI; i>=graph_bottom; i-=WATERFALL_MULTI) {		// Scroll down WATERFALL_MULTI lines at once
      ili9341_read_memory(OFFSETX, i, w_width, WATERFALL_MULTI, spi_buffer);
      ili9341_bulk(OFFSETX, i+1, w_width, WATERFALL_MULTI);
    }
    i = CHART_BOTTOM-((CHART_BOTTOM-graph_bottom)/WATERFALL_MULTI)*WATERFALL_MULTI-1;
  }
#endif
  for (; i >=graph_bottom; i--) {        // Scroll down
    ili9341_read_memory(OFFSETX, i, w_width, 1, spi_buffer);
    ili9341_bulk(OFFSETX, i+1, w_width, 1);
  }
  index_y_t *index = NULL;
  for (int t=0;t<TRACES_MAX;t++) {                      // Find trace with active measurement
    if (IS_TRACE_ENABLE(t) && setting.average[t] == AV_OFF) {
      index = trace_index_y[t];
      break;
    }
  }
  if (index == NULL) {
    for (int t=0;t<TRACES_MAX;t++) {                      // Find trace with measurement
      if (IS_TRACE_ENABLE(t)) {
        index = trace_index_y[t];
        break;
      }
    }
  }
  if (index == NULL)
    return;
  int j = 0;
  for (i=0; i< sweep_points; i++) {			// Add new topline
    uint16_t color;
#ifdef _USE_WATERFALL_PALETTE
    uint16_t y = _PALETTE_ALIGN(255 - graph_bottom + index[i]); // should be always in range 0 - graph_bottom
//    y = (uint8_t)i;  // for test
    if (y > 255)            // at start the index_y_t table could be empty leading to negative y
      break;
    color = waterfall_palette[y];
#elif 1
    uint16_t y = index[i]; // should be always in range 0 - graph_bottom
    uint16_t ratio = (graph_bottom - y)*4;
//    ratio = (i*2);    // Uncomment for testing the waterfall colors
    int16_t b = 255 - ratio;
    if (b > 255) b = 255;
    if (b < 0) b = 0;
    int16_t r = ratio - 255;
    if (r > 255) r = 255;
    if (r < 0) r = 0;
    int16_t g = 255 - b - r;
#define gamma_correct(X) X = (X < 64 ? X * 2 : X < 128 ? 128 + (X-64) : X < 192 ? 192 + (X - 128)/2 : 225 + (X - 192) / 4)
    gamma_correct(r);
    gamma_correct(g);
    gamma_correct(b);
    color = RGB565(r, g, b);
#else
    uint16_t y = SMALL_WATERFALL - index[i]* (graph_bottom == BIG_WATERFALL ? 2 : 1); // should be always in range 0 - graph_bottom *2 depends on height of scroll
    // Calculate gradient palette for range 0 .. 192
    // idx     r   g   b
    //   0 - 127   0   0
    //  32 - 255 127   0
    //  64 - 255 255 127
    //  96 - 255 255 255
    // 128 - 127 255 255
    // 160 -   0 127 255
    // 192 -   0   0 127
    // 224 -   0   0   0
//    y = (uint8_t)i;  // for test
         if (y <  32) color = RGB565( 127+((y-  0)*4),   0+((y-  0)*4),               0);
    else if (y <  64) color = RGB565(             255, 127+((y- 32)*4),   0+((y- 32)*4));
    else if (y <  96) color = RGB565(             255,             255, 127+((y- 64)*4));
    else if (y < 128) color = RGB565( 252-((y- 96)*4),             255,             255);
    else if (y < 160) color = RGB565( 124-((y-128)*4), 252-((y-128)*4),             255);
    else              color = RGB565(               0, 124-((y-160)*4), 252-((y-160)*4));

#endif
    while (j * sweep_points  < (i+1) * WIDTH) {   // Scale waterfall to WIDTH points
      spi_buffer[j++] = color;
    }
  }
  ili9341_bulk(OFFSETX, graph_bottom, w_width, 1);
//  STOP_PROFILE;
}
#if 0
//extern float peakLevel;
//extern float min_level;
//int w_max = -130;
//int w_min = 0;
uint8_t old_waterfall = 100;
void
set_waterfall(void)
{
  if (old_waterfall == setting.waterfall)
    return;
  old_waterfall = setting.waterfall;
  if (setting.waterfall == W_SMALL)       graph_bottom = SMALL_WATERFALL;
  else if (setting.waterfall == W_BIG)    graph_bottom = BIG_WATERFALL;
  else /*if (setting.waterfall == W_OFF)*/graph_bottom = NO_WATERFALL;
  _grid_y = graph_bottom / NGRIDY;
  area_height = AREA_HEIGHT_NORMAL;
  if (setting.waterfall != W_OFF){
    ili9341_set_background(LCD_BG_COLOR);
    ili9341_fill(OFFSETX, graph_bottom, LCD_WIDTH - OFFSETX, CHART_BOTTOM - graph_bottom);
  }
  request_to_redraw_grid();
}
#endif
void
disable_waterfall(void)
{
  setting.waterfall = W_OFF;
  set_level_meter_or_waterfall();
}

#ifdef __LEVEL_METER__
static void update_level_meter(void){
  if (level_text[0] == 0)
    return;
  ili9341_set_background(LCD_BG_COLOR);
//  const int minimum_text_width = 6*5*7;
//  level_text[6] = 0;
//  if (area_width-minimum_text_width > 0)
//    ili9341_fill(OFFSETX+minimum_text_width, graph_bottom+1, area_width-minimum_text_width, CHART_BOTTOM - graph_bottom);
  ili9341_set_foreground(LCD_FG_COLOR);
  int x_max = area_width+OFFSETX;
#ifdef TINYSA4
  int  BIG_SIZE = (auto_capture?2:4);
#else
#define BIG_SIZE 2
#endif
  int w = ili9341_drawstring_size(level_text, OFFSETX + 3, graph_bottom + 3,BIG_SIZE, x_max);         // TODO Size 4 does not transfer to remote desktop??????
  if (w < x_max)
    ili9341_fill(w, graph_bottom+3, x_max - w, BIG_SIZE * wFONT_GET_HEIGHT);
#ifdef TINYSA4
  w = ili9341_drawstring_size(freq_text, OFFSETX + 3, graph_bottom + 3 + BIG_SIZE * wFONT_STR_HEIGHT ,BIG_SIZE, x_max);         // TODO Size 4 does not transfer to remote desktop??????
  if (w < x_max)
    ili9341_fill(w, graph_bottom+3+ BIG_SIZE * wFONT_STR_HEIGHT, x_max - w, BIG_SIZE * wFONT_GET_HEIGHT);
#endif
}

void
set_level_meter_or_waterfall(void)
{
  if (setting.waterfall == W_BIG
#ifdef TINYSA4
      || setting.level_meter
#endif
      )                                                 graph_bottom = BIG_WATERFALL;
  else if (setting.waterfall == W_SMALL
#ifndef TINYSA4
      || setting.level_meter
#endif
    )                                                    graph_bottom = SMALL_WATERFALL;
#ifdef TINYSA4
  else if (setting.waterfall == W_SUPER)                 graph_bottom = SUPER_WATERFALL;
#endif
  else                                                   graph_bottom = NO_WATERFALL;
  _grid_y = graph_bottom / NGRIDY;
  area_height = AREA_HEIGHT_NORMAL;
  if (setting.waterfall != W_OFF || setting.level_meter){
    ili9341_set_background(LCD_BG_COLOR);
    ili9341_fill(OFFSETX, graph_bottom, LCD_WIDTH - OFFSETX, CHART_BOTTOM - graph_bottom);
  }
  request_to_redraw_grid();
}

void
disable_level_meter(void)
{
  setting.level_meter = false;
  set_level_meter_or_waterfall();
}
#endif

void
plot_init(void)
{
  redraw_request|= REDRAW_AREA | REDRAW_BATTERY | REDRAW_CAL_STATUS | REDRAW_FREQUENCY;
  draw_all(true);
}

#pragma GCC pop_options
