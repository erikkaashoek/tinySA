static uint16_t file_count;
static uint16_t page_count;
static uint16_t current_page;
static uint16_t sel_mode;

// Buttons in browser
enum {FILE_BUTTON_LEFT = 0, FILE_BUTTON_RIGHT, FILE_BUTTON_EXIT, FILE_BUTTON_DEL, FILE_BUTTON_FILE};
// Button position on screen
typedef struct  {
  uint16_t x;
  uint16_t y;
  uint16_t w;
  uint8_t  h;
  uint8_t  ofs;
} browser_btn_t;
static const browser_btn_t browser_btn[] = {
  [FILE_BUTTON_LEFT] = {         0  + 40, LCD_HEIGHT - FILE_BOTTOM_HEIGHT, LCD_WIDTH/2 - 80, FILE_BOTTOM_HEIGHT, (LCD_WIDTH/2 - 80 - FONT_WIDTH)/2}, // < previous
  [FILE_BUTTON_RIGHT]= {LCD_WIDTH/2 + 40, LCD_HEIGHT - FILE_BOTTOM_HEIGHT, LCD_WIDTH/2 - 80, FILE_BOTTOM_HEIGHT, (LCD_WIDTH/2 - 80 - FONT_WIDTH)/2}, // > next
  [FILE_BUTTON_EXIT] = {LCD_WIDTH   - 40, LCD_HEIGHT - FILE_BOTTOM_HEIGHT,               40, FILE_BOTTOM_HEIGHT, (              40 - FONT_WIDTH)/2}, // X exit
  [FILE_BUTTON_DEL]  = {         0  +  0, LCD_HEIGHT - FILE_BOTTOM_HEIGHT,               40, FILE_BOTTOM_HEIGHT, (            40 - 3*FONT_WIDTH)/2}, // DEL
  // File button, only size and start position, must be idx = FILE_BUTTON_FILE
  [FILE_BUTTON_FILE] = {               0,                               0, LCD_WIDTH/FILES_COLUMNS, FILE_BUTTON_HEIGHT,                          5},
};

static void browser_get_button_pos(int idx, browser_btn_t *b) {
  int n = idx >= FILE_BUTTON_FILE ? FILE_BUTTON_FILE : idx;
#if 0
  memcpy(b, &browser_btn[n], sizeof(browser_btn_t));
#else
  b->x = browser_btn[n].x;
  b->y = browser_btn[n].y;
  b->w = browser_btn[n].w;
  b->h = browser_btn[n].h;
  b->ofs = browser_btn[n].ofs;
#endif
  if (idx > FILE_BUTTON_FILE) { // for file buttons use multiplier from start offset
    idx-= FILE_BUTTON_FILE;
    b->x+= b->w * (idx / FILES_ROWS);
    b->y+= b->h * (idx % FILES_ROWS);
  }
}

static void browser_draw_button(int idx, const char *txt) {
  if (idx < 0) return;
  ui_button_t b;
  browser_btn_t btn;
  browser_get_button_pos(idx, &btn);
  // Mark DEL button in file delete mode
  b.bg = (idx == FILE_BUTTON_DEL && sel_mode) ? LCD_LOW_BAT_COLOR : LCD_MENU_COLOR;
  b.fg = LCD_MENU_TEXT_COLOR;
  b.border = (idx == selection) ? BROWSER_BUTTON_BORDER|BUTTON_BORDER_FALLING : BROWSER_BUTTON_BORDER|BUTTON_BORDER_RISE;
  if (txt == NULL) b.border|= BUTTON_BORDER_NO_FILL;
  draw_button(btn.x, btn.y, btn.w, btn.h, &b);
  if (txt) ili9341_drawstring_7x13(txt, btn.x + btn.ofs, btn.y + (btn.h - bFONT_STR_HEIGHT) / 2);
}

static char to_lower(char c) {return (c >='A' && c <= 'Z') ? c - 'A' + 'a' : c;}

static bool strcmpi(const char *t1, const char *t2) {
  int i = 0;
  while (1) {
    char ch1 = to_lower(t1[i]), ch2 = to_lower(t2[i]);
    if (ch1 != ch2) return false;
    if (ch1 ==   0) return true;
    i++;
  }
}

static bool compare_ext(const char *name, const char *ext) {
  int i = 0, j = 0;
  while (name[i]) if (name[i++] == '.') j = i;    // Get last '.' position + 1
  return j == 0 ? false : strcmpi(&name[j], ext); // Compare text after '.' and ext
}

static FRESULT sd_findnext(DIR* dp, FILINFO* fno) {
  while (f_readdir(dp, fno) == FR_OK && fno->fname[0]) {
    if (fno->fattrib & AM_DIR) continue;
    if (compare_ext(fno->fname, dp->pat)) return FR_OK;
//#if FF_USE_LFN && FF_USE_FIND == 2
//    if (compare_ext(fno->altname, dp->pat)) return FR_OK;
//#endif
  }
  return FR_NO_FILE;
}

static FRESULT sd_open_dir(DIR* dp, const TCHAR* path, const TCHAR* pattern) {
  dp->pat = pattern;
  return f_opendir(dp, path);
}

static void browser_open_file(int sel) {
  FILINFO fno;
  FRESULT res;
  DIR dj;
  int cnt;
  if ((uint16_t)sel >= file_count) return;
  if (f_mount(fs_volume, "", 1) != FR_OK) return;
repeat:
  cnt = sel;
  if (sd_open_dir(&dj, "", file_ext[keypad_mode]) != FR_OK) return;  // open dir
  while (sd_findnext(&dj, &fno) == FR_OK && cnt != 0) cnt--;         // skip cnt files
  f_closedir(&dj);
  if (cnt != 0) return;

  // Delete file if in delete mode
  if (sel_mode) {f_unlink(fno.fname); return;}
run_file:
  {}
  const char *error = NULL;
  bool leave_show = true;
  UINT size;
  if (f_open(fs_file, fno.fname, FA_READ) != FR_OK) return;

  ili9341_set_foreground(LCD_FG_COLOR);
  ili9341_set_background(LCD_BG_COLOR);
  switch (keypad_mode) {
  case FMT_TBL_FILE:
  {
    const int buffer_size = 256;
    const int line_size = 128;
    char *buf_8 = (char *)spi_buffer; // must be greater then buffer_size + line_size
    char *line  = buf_8 + buffer_size;
    uint16_t j = 0, i, count = 0;
    while (f_read(fs_file, buf_8, buffer_size, &size) == FR_OK && size > 0) {
      for (i = 0; i < size; i++) {
        uint8_t c = buf_8[i];
        if (c == '\r') {                                                     // New line (Enter)
          line[j] = 0; j = 0;
          char *args[16];
          int nargs = parse_line(line, args, 16);                            // Parse line to 16 args
          if (nargs < 2 || args[0][0] == '#' || args[0][0] == '!') continue; // No data or comment or settings
          if (count >= LIMITS_MAX) {error = "Format err"; goto finish2;}
          setting.limits[current_trace][count].frequency = my_atoui(args[0]);// Get frequency
          setting.limits[current_trace][count].level = my_atof(args[1]);     // Get frequency
          setting.limits[current_trace][count].enabled = true;               // Get frequency
          count++;
        }
        else if (c < 0x20) continue;                 // Others (skip)
        else if (j < line_size) line[j++] = (char)c; // Store
      }
    }
finish2:
    for (; count < LIMITS_MAX; i++) {
      setting.limits[current_trace][count].frequency = 0;
      setting.limits[current_trace][count].level = 0;
      setting.limits[current_trace][count].enabled = false;
      count++;
    }
    setting.stored[current_trace] = true;
    TRACE_ENABLE(1<<current_trace);
    set_average(current_trace,AV_TABLE);
    dirty = true;
    limits_update();
    break;
  }
#ifdef __BANDS__
  case FMT_BND_FILE:
  {
    const int buffer_size = 256;
    const int line_size = 128;
    char *buf_8 = (char *)spi_buffer; // must be greater then buffer_size + line_size
    char *line  = buf_8 + buffer_size;
    uint16_t j = 0, i, count = 0;
    while (f_read(fs_file, buf_8, buffer_size, &size) == FR_OK && size > 0) {
      for (i = 0; i < size; i++) {
        uint8_t c = buf_8[i];
        if (c == '\r') {                                                     // New line (Enter)
          line[j] = 0; j = 0;
          char *args[16];
          int nargs = parse_line(line, args, 16);                            // Parse line to 16 args
          if (nargs < 2 || args[0][0] == '#' || args[0][0] == '!') continue; // No data or comment or settings
          if (count >= BANDS_MAX) {error = "Format err"; goto finish3;}
          setting.bands[count].start = my_atoui(args[0]);// Get frequency
          setting.bands[count].end = my_atoui(args[1]);// Get frequency
          setting.bands[count].level = my_atof(args[2]);     // Get frequency
          setting.bands[count].enabled = true;               // Get frequency
          count++;
        }
        else if (c < 0x20) continue;                 // Others (skip)
        else if (j < line_size) line[j++] = (char)c; // Store
      }
    }
finish3:
    for (; count < BANDS_MAX; i++) {
      setting.bands[count].start = 0;
      setting.bands[count].end = 0;
      setting.bands[count].level = 0;
      setting.bands[count].enabled = false;
      count++;
    }
    dirty = true;
    update_frequencies();
    break;
  }
#endif
  case FMT_CSV_FILE:
  {
    const int buffer_size = 256;
    const int line_size = 128;
    char *buf_8 = (char *)spi_buffer; // must be greater then buffer_size + line_size
    char *line  = buf_8 + buffer_size;
    uint16_t j = 0, i, count = 0;
    freq_t start = 0, stop = 0, f;
    while (f_read(fs_file, buf_8, buffer_size, &size) == FR_OK && size > 0) {
      for (i = 0; i < size; i++) {
        uint8_t c = buf_8[i];
        if (c == ',') c = ' ';
        if (c == '\r') {                                                     // New line (Enter)
          line[j] = 0; j = 0;
          char *args[16];
          int nargs = parse_line(line, args, 16);                            // Parse line to 16 args
          if (nargs < 2 || args[0][0] == '#' || args[0][0] == '!') continue; // No data or comment or settings
          f = my_atoui(args[0]);                                             // Get frequency
          if (count >= POINTS_COUNT || f > maxFreq) {error = "Format err"; goto finish;}
          if (count == 0)  {
            start = f;                                                      // For index 0 set as start
            setting.stored[current_trace] = true;
            TRACE_ENABLE(1<<current_trace);
          }
          stop  = f;                                                         // last set as stop
          measured[current_trace][count] = my_atof(args[1]);
         count++;
        }
        else if (c < 0x20) continue;                 // Others (skip)
        else if (j < line_size) line[j++] = (char)c; // Store
      }
    }
finish:
    if (count != 0) { // Points count not zero, so apply data to traces
//      pause_sweep();
      setting._sweep_points = count;
      set_sweep_frequency(ST_START, start);
      set_sweep_frequency(ST_STOP, stop);
//      request_to_redraw(REDRAW_AREA);
    }
    break;
  }

  case FMT_CMD_FILE:
  {
static char cmd_buffer[256+128];
    const int buffer_size = 256;
    const int line_size = 128;
    char *buf_8 = cmd_buffer; // (char *)spi_buffer; // must be greater then buffer_size + line_size
    char *line  = buf_8 + buffer_size;
    uint16_t j = 0, i;
    break_execute = false;
    while (f_read(fs_file, buf_8, buffer_size, &size) == FR_OK && size > 0) {
      for (i = 0; i < size; i++) {
        if (break_execute) goto done;
        uint8_t c = buf_8[i];
        if (c == '\r') {                             // New line (Enter)
          line[j] = 0; j = 0;
          if (line[0] == 'r' && line[1] == 'u' && line[2] == 'n') {
            j = 4;
            while (line[j] == ' ' || line[j] == '"')
              j++;
            i = 0;
            while (line[j] != '"' && line[j] != 0)
              fno.fname[i++] = line[j++];
            fno.fname[i] = 0;
            f_close(fs_file);
            goto run_file;
          }
          shell_executeCMDLine(line);
        }
        else if (c < 0x20) continue;                 // Others (skip)
        else if (j < line_size) line[j++] = (char)c; // Store
      }
    }
    done:
    break;
  }
  /*
   * BMP file load procedure, load only device screenshots
   */
  case FMT_BMP_FILE:
  {
    int y;
    leave_show = false;      // allow step up/down load bitmap
    uint16_t *buf_16 = spi_buffer; // prepare buffer
    res = f_read(fs_file, (void *)buf_16, sizeof(bmp_header_v4), &size); // read heaser
    if (res != FR_OK || buf_16[9] != LCD_WIDTH || buf_16[11] != LCD_HEIGHT || buf_16[14] != 16) {error = "Format err"; break;}
    for (y = LCD_HEIGHT-1; y >=0 && res == FR_OK; y--) {
      res = f_read(fs_file, (void *)buf_16, LCD_WIDTH * sizeof(uint16_t), &size);
      swap_bytes(buf_16, LCD_WIDTH);
      ili9341_bulk(0, y, LCD_WIDTH, 1);
    }
    ili9341_drawstring_7x13(fno.fname, 0, LCD_HEIGHT - 3*bFONT_STR_HEIGHT);
  }
  break;
  /*
   *  Load calibration
   */
//  case FMT_CAL_FILE:
//  {
//    uint32_t magic;
//    char *src = (char*)&current_props + sizeof(magic);
//    uint32_t total = sizeof(current_props) - sizeof(magic);
//    // Compare file size and try read magic header, if all OK load it
//    if (fno.fsize == sizeof(current_props) && f_read(fs_file, &magic, sizeof(magic), &size) == FR_OK &&
//        magic == PROPS_MAGIC && f_read(fs_file, src, total, &size) == FR_OK)
//        load_properties(NO_SAVE_SLOT);
//    else error = "Format err";
//  }
//  break;
  /*
   *  Load preset
   */
  case FMT_PRS_FILE:
  {
    uint32_t magic;
    char *src = (char*)&setting + sizeof(magic);
    uint32_t total = sizeof(setting_t) - sizeof(magic);
    // Compare file size and try read magic header, if all OK load it
    if (fno.fsize == sizeof(setting) && f_read(fs_file, &magic, sizeof(magic), &size) == FR_OK &&
        magic == SETTING_MAGIC && f_read(fs_file, src, total, &size) == FR_OK) {

     // TODO remove code duplication with flash.c
      update_min_max_freq();
      update_frequencies();
      set_scale(setting.scale);
      set_reflevel(setting.reflevel);
      set_level_meter_or_waterfall();
      // Remember where this preset came from so storing it back can default to it
      plot_printf(sd_preset_path, sizeof(sd_preset_path), "%s", fno.fname);
    }
    else error = "Format err";
  }
  break;
  /*
   *  Load preset
   */
  case FMT_CFG_FILE:
  {
    uint32_t magic;
    char *src = (char*)&config + sizeof(magic);
    uint32_t total = sizeof(config_t) - sizeof(magic);
    // Compare file size and try read magic header, if all OK load it
    if (fno.fsize == sizeof(config) && f_read(fs_file, &magic, sizeof(magic), &size) == FR_OK &&
        magic == CONFIG_MAGIC && f_read(fs_file, src, total, &size) == FR_OK) {
      config_save();
      drawMessageBox("Info", "Restart tinySA to load new config", 2000);
    }
    else error = "Format err";
  }
  break;
  default: break;
  }
  f_close(fs_file);
  if (error) {
    ili9341_clear_screen();
    drawMessageBox(error, fno.fname, leave_show ? 2000 : 0);
  } else
    if (leave_show) drawMessageBox("Loaded", fno.fname, leave_show ? 1000 : 0);
  if (leave_show) return;
  // Process input
  while (1) {
    uint16_t status = btn_check();
    int key = -1;
    if (status & EVT_DOWN) key = 0;
    if (status & EVT_UP  ) key = 1;
    if (status & EVT_BUTTON_SINGLE_CLICK) key = 2;

    status = touch_check();
    if (status == EVT_TOUCH_PRESSED || status == EVT_TOUCH_DOWN) {
      int touch_x, touch_y;
      touch_position(&touch_x, &touch_y);
           if (touch_x < LCD_WIDTH *1/3) key = 0;
      else if (touch_x < LCD_WIDTH *2/3) key = 2;
      else                               key = 1;
      touch_wait_release();
    }
    chThdSleepMilliseconds(100);
    int old_sel = sel;
         if (key == 0) {if (--sel < 0) sel = file_count - 1;}
    else if (key == 1) {if (++sel > file_count - 1) sel = 0;}
    else if (key == 2) break;
    if (old_sel != sel) goto repeat;
  }
}

static void browser_draw_buttons(void) {
  browser_draw_button(FILE_BUTTON_DEL, "DEL");
  browser_draw_button(FILE_BUTTON_LEFT,  "<");
  browser_draw_button(FILE_BUTTON_RIGHT, ">");
  browser_draw_button(FILE_BUTTON_EXIT,  "X");
}

static void browser_draw_page(int page) {
  FILINFO fno;
  DIR dj;
  // Mount SD card and open directory
  if (f_mount(fs_volume, "", 1) != FR_OK ||
      sd_open_dir(&dj, "", file_ext[keypad_mode]) != FR_OK) {
    drawMessageBox("ERROR", "NO CARD", 2000);
    ui_mode_normal();
    return;
  }
  // Draw Browser UI
  int cnt = 0;
  uint16_t start_file = (page - 1) * FILES_PER_PAGE;
  ili9341_set_background(LCD_MENU_COLOR);
  ili9341_clear_screen();
  lcd_set_font(FONT_NORMAL);
  while (sd_findnext(&dj, &fno) == FR_OK) {
    if (cnt >= start_file && cnt < (start_file + FILES_PER_PAGE)) {
      //uint16_t sec = ((fno.ftime<<1)  & 0x3F);
      //uint16_t min = ((fno.ftime>>5)  & 0x3F);
      //uint16_t h   = ((fno.ftime>>11) & 0x1F);
      //uint16_t d   = ((fno.fdate>>0)  & 0x1F);
      //uint16_t m   = ((fno.fdate>>5)  & 0x0F);
      //uint16_t year= ((fno.fdate>>9)  & 0x3F) + 1980;
      //lcd_printf(x, y, "%2d %s %u - %u/%02u/%02u %02u:%02u:%02u", cnt, fno.fname, fno.fsize, year, m, d, h, min, sec);
      browser_draw_button(cnt - start_file + FILE_BUTTON_FILE, fno.fname);
    }
    cnt++;
    if (file_count && (start_file + FILES_PER_PAGE == cnt)) break;
  }
  f_closedir(&dj);
  // Calculate page and file count on first run
  if (file_count == 0) {
    file_count = cnt;
    page_count = cnt == 0 ? 1 : (file_count + FILES_PER_PAGE - 1) / FILES_PER_PAGE;
  }
  browser_draw_buttons();
  lcd_printf(LCD_WIDTH / 2 - 3 * bFONT_WIDTH, LCD_HEIGHT - (FILE_BOTTOM_HEIGHT + bFONT_STR_HEIGHT) / 2, "- %u | %u -", page, page_count);
  lcd_set_font(FONT_SMALL);
  return;
}

static void browser_key_press(int key) {
  int page;
  switch (key) {
    case FILE_BUTTON_LEFT:
    case FILE_BUTTON_RIGHT: // Switch page on left / right change
      page = current_page;
      if (key == FILE_BUTTON_LEFT  && --current_page < 1) current_page = page_count;
      if (key == FILE_BUTTON_RIGHT && ++current_page > page_count) current_page = 1;
      if (page != current_page)
        browser_draw_page(current_page);
    break;
    case FILE_BUTTON_EXIT:  //Exit
      ui_mode_normal();
    break;
    case FILE_BUTTON_DEL:   // Toggle delete mode
      sel_mode^= 1;
      browser_draw_buttons();
    break;
    case FILE_BUTTON_FILE:  // Open or delete file
    default:
      browser_open_file(key - FILE_BUTTON_FILE + (current_page - 1) * FILES_PER_PAGE);
      if (sel_mode) {
        file_count = 0;     // Reeset file count (recalculate on draw page)
        selection = -1;     // Reset delection
        sel_mode = 0;       // Exit file delete mode
        browser_draw_page(current_page);
        return;
      }
      ui_mode_normal(); // Exit
    break;
  }
}

static int browser_get_max(void) {
  // get max buttons depend from page and file count
  int max = current_page == page_count ? (file_count % FILES_PER_PAGE) : FILES_PER_PAGE;
  if (file_count > 0 && max == 0) max = FILES_PER_PAGE;
  return max + FILE_BUTTON_FILE - 1;
}

// Process UI input for browser
static void browser_apply_touch(int touch_x, int touch_y) {
  browser_btn_t btn;
  int old = selection;
  int max = browser_get_max();
  for (int idx = 0; idx <= max; idx++) {
    browser_get_button_pos(idx, &btn);
    if (touch_x < btn.x || touch_x >= btn.x + btn.w ||
        touch_y < btn.y || touch_y >= btn.y + btn.h) continue;
    // Found button under touch
    browser_draw_button(selection = idx, NULL);  // draw new selection
    browser_draw_button(old, NULL);              // clear old
    touch_wait_release();
    selection = -1;
    browser_draw_button(idx, NULL);              // clear selection
    browser_key_press(idx);
    return;
  }
}

static void ui_process_browser_lever(void) {
  uint16_t status = btn_check();
  if (status == 0) return;
  if (status == EVT_BUTTON_SINGLE_CLICK) {
    if (selection >= 0) browser_key_press(selection); // Process click
    return;
  }
  int max = browser_get_max();
  do {
    int old = selection;
    if((status & EVT_DOWN) && --selection < 0) selection = max;
    if((status & EVT_UP)   && ++selection > max) selection = 0;
    if (old != selection) {
      browser_draw_button(old, NULL);       // clear old selection
      browser_draw_button(selection, NULL); // draw new selection
    }
    chThdSleepMilliseconds(100);
  } while ((status = btn_wait_release()) != 0);
}

static UI_FUNCTION_CALLBACK(menu_sdcard_browse_cb) {
  (void)item;
  if (ui_mode == UI_BROWSER)
    return;
  area_width = 0;
  area_height = 0;
  ui_mode = UI_BROWSER;
  keypad_mode = data;
  current_page = 1;

  file_count = 0;
  selection = -1;
  sel_mode = 0;
  browser_draw_page(current_page);
}
