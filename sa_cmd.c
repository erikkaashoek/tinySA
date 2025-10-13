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
#pragma GCC push_options
#pragma GCC optimize ("Os")

uint32_t xtoi(char *t);

static int points = 101; // For 's' and 'm' commands

VNA_SHELL_FUNCTION(cmd_mode)
{
#ifdef TINYSA3
  static const char cmd_low_high[] = "low|high";
#else
  static const char cmd_low_high[] = "low";
#endif
  static const char cmd_in_out[] = "input|output";
#ifdef TINYSA3
  if (argc != 2) {
#else
    if (argc != 1 && argc != 2) {
#endif
    usage:
#ifdef TINYSA4
    shell_printf("usage: mode [low] %s\r\n", cmd_in_out);
#else
    shell_printf("usage: mode %s %s\r\n", cmd_low_high,cmd_in_out);
#endif
    return;
  }
  int io = 0;
  int lh = 0;
#ifdef TINYSA4
  lh = get_str_index(argv[0], cmd_low_high);
  if (lh != 0) {
    lh = 0;
    io = get_str_index(argv[0], cmd_in_out);
  } else
    io = get_str_index(argv[1], cmd_in_out);
#else
  lh = get_str_index(argv[0], cmd_low_high);
  io = get_str_index(argv[1], cmd_in_out);
#endif
  if (lh < 0 || io<0)
    goto usage;
  menu_move_top();
  switch(lh+io*2)
  {
  case 0:
    set_mode(M_LOW);
    ui_mode_normal();
    break;
#ifdef TINYSA3
  case 1:
    set_mode(M_HIGH);
    ui_mode_normal();
    break;
#endif
  case 2:
    set_mode(M_GENLOW);
    menu_push_lowoutput();
    break;
#ifdef TINYSA3
  case 3:
    set_mode(M_GENHIGH);
    menu_push_highoutput();
    break;
#endif
  }
}

VNA_SHELL_FUNCTION(cmd_modulation )
{
#ifdef TINYSA4
  static const char cmd_mod[] = "off|am|fm|freq|depth|deviation";
#else
  static const char cmd_mod[] = "off|am|nfm|wfm|extern|freq";
#endif
  if (argc < 1) {
  usage:
    usage_printf("modulation %s 100..6000\r\n", cmd_mod);
    return;
  }
#ifdef TINYSA4
  static const int cmd_mod_val[] = { MO_NONE, MO_AM, MO_WFM, -1, -1, -1};
#else
  static const int cmd_mod_val[] = { MO_NONE, MO_AM, MO_NFM, MO_WFM, MO_EXTERNAL, -1};
#endif
  int m = get_str_index(argv[0], cmd_mod);
  if (m<0)
     goto usage;
  if (cmd_mod_val[m] >=0)
    set_modulation(cmd_mod_val[m]);
  else {
    if (argc != 2)
      goto usage;
    int a = my_atoi(argv[1]);
#ifdef TINYSA4
    switch(m) {
    case 3: set_modulation_frequency(a); break;
    case 4: set_depth(a); break;
    case 5: set_deviation(a); break;

    }
#else
    set_modulation_frequency(a);
#endif
  }
}

VNA_SHELL_FUNCTION(cmd_calc )
{
  int t = 0;
#ifdef TINYSA4
  static const char cmd_cal[] = "off|minh|maxh|maxd|aver4|aver16|aver|quasi|log|lin";
#else
  static const char cmd_cal[] = "off|minh|maxh|maxd|aver4|aver16|aver|quasi";
#endif
  if (argc < 1) {
  usage:
    usage_printf("calc [{trace#}] %s\r\n%s\r\n", cmd_cal,averageText[setting.average[t]]);
    return;
  }
  int next_arg = 0;
  if ('0' <= argv[0][0] && argv[0][0] <= '9') {
    t = my_atoi(argv[0]) - 1;
    next_arg++;
    argc--;
    if (t < 0 || t >= TRACES_MAX)
      goto usage;
    if (argc < 1)
      goto usage;
  }

  int m = get_str_index(argv[next_arg], cmd_cal);
  if (m<0)
     goto usage;
#ifdef TINYSA4
  if (m>=8) {
    linear_averaging = (m == 9);
  } else
#endif
  set_average(t, m);
}

int generic_option_cmd( const char *cmd, const char *cmd_list, int argc, char *argv)
{
  if (argc != 1) {
  usage:
    usage_printf("%s %s\r\n", cmd, cmd_list);
    return -1;
  }
  int m = get_str_index(argv, cmd_list); // this will catch the ?
  if (m < 0)
    goto usage;
  return m;
}

#ifdef __SPUR__
VNA_SHELL_FUNCTION(cmd_spur)
{
#ifdef TINYSA4
  int m = generic_option_cmd("spur", "off|on|auto", argc, argv[0]);
#else
  int m = generic_option_cmd("spur", "off|on", argc, argv[0]);
#endif
  if (m>=0) {
    set_spur(m);
    redraw_request |= REDRAW_CAL_STATUS | REDRAW_AREA;
  }
}
#endif

#ifdef TINYSA4
VNA_SHELL_FUNCTION(cmd_lna)
{
  int m = generic_option_cmd("lna", "off|on", argc, argv[0]);
  if (m>=0) {
    set_extra_lna(m);
    redraw_request |= REDRAW_CAL_STATUS | REDRAW_AREA;
  }
}

VNA_SHELL_FUNCTION(cmd_avoid)
{
  extern freq_t dynamic_spur_table[];       // Frequencies to be calculated
  extern int dynamic_spur_table_size;
  int m = generic_option_cmd("avoid", "auto|off|on|dump", argc, argv[0]);
  if (m == 3) {
    for (int i=0; i < dynamic_spur_table_size; i++)
      shell_printf("%D\n",dynamic_spur_table[i]);
  }
  else if (m>=0) {
    set_avoid(m);
    if (m == 0)
      toggle_debug_avoid();
    redraw_request |= REDRAW_CAL_STATUS | REDRAW_AREA;
  }
}

#endif
#ifdef __ULTRA__
VNA_SHELL_FUNCTION(cmd_ultra)
{
  const char *ultra_cmd = "off|on|auto|start|harm";
  if (argc<1 || argc>2)
    goto usage;
  if (argv[0][0] == '?')
    goto usage;
  int m = get_str_index(argv[0], ultra_cmd);
  freq_t a = ULTRA_AUTO;
  switch(m) {
  case 0:
  case 1:
    config.ultra = m;
    update_min_max_freq();
    break;
  case 2:
    goto auto_label;
  case 3:
    if (argc != 2)
      goto usage;
    if (get_str_index(argv[1], ultra_cmd)== 2)
       goto auto_label;
    a = my_atoui(argv[1]);
    auto_label:
    config.ultra_start = a;
    ultra_start = a;
    dirty = true;
    break;
  case 4:
    if (argc != 2)
      goto usage;
    a = my_atoui(argv[1]);
    config.harmonic_start = a;
    dirty = true;
    break;
  default:
    {
     usage:
      usage_printf("ultra %s {freq}\r\n", ultra_cmd);
    }
  }
}
/*
VNA_SHELL_FUNCTION(cmd_ultra_start)
{
  if (argc != 1 || argv[0][0] == '?') {
    usage_printf("ultra_start {0..4290M}\r\n%QHz\r\n", config.ultra_threshold);
    return;
  } else {
    freq_t a = (freq_t)my_atoi(argv[0]);
    config.ultra_threshold = a;
    ultra_threshold = (config.ultra_threshold == 0 ? DEFAULT_ULTRA_THRESHOLD : config.ultra_threshold);
    dirty = true;
    config_save();
  }
}
*/
#endif

VNA_SHELL_FUNCTION(cmd_output)
{
#ifdef TINYSA4
  int m = generic_option_cmd("output", "on|off|normal|mixer", argc, argv[0]);
#else
  int m = generic_option_cmd("output", "on|off", argc, argv[0]);
#endif
  switch(m)
  {
  case 0:
  case 1:
    setting.mute = m;
    break;
#ifdef TINYSA4
  case 2:
  case 3:
    setting.mixer_output = m-2;
#endif
  }
  dirty = true;
}

VNA_SHELL_FUNCTION(cmd_load)
{
  if (argc != 1 || argv[0][0] == '?') {
  usage:
    usage_printf("load 0..4\r\n");
    return;
  }
  uint16_t a = my_atoui(argv[0]);
  if (a > 4)
    goto usage;
  if (caldata_recall(a) == -1) {
    if (a == 0)
      reset_settings(setting.mode);
  }
}

#ifdef TINYSA4

static uint8_t reg_agc_lna = 0;

VNA_SHELL_FUNCTION(cmd_lna2)
{
  int a;
  if (argc != 1  || argv[0][0] == '?') {
    usage_printf("lna2 0..7|auto\r\n");
    return;
  }
  if (get_str_index(argv[0],"auto") == 0) {
    reg_agc_lna &= 0xf0;
  } else {
    a = my_atoi(argv[0]);
    reg_agc_lna &= 0xf0;
    reg_agc_lna |= 0x08 | a;
  }
  SI446x_set_AGC_LNA(reg_agc_lna);
}

VNA_SHELL_FUNCTION(cmd_agc)
{
  int a;
  if (argc != 1  || argv[0][0] == '?') {
    usage_printf("agc 0..7|auto\r\n");
    return;
  }
  if (get_str_index(argv[0],"auto") == 0) {
    reg_agc_lna &= 0x0f;
  } else {
    a = my_atoi(argv[0]);
    reg_agc_lna &= 0x0f;
    reg_agc_lna |= 0x80 | (a << 4);
  }
  SI446x_set_AGC_LNA(reg_agc_lna);
}

#endif


VNA_SHELL_FUNCTION(cmd_attenuate)
{
  if (argc != 1 || argv[0][0] == '?') {
    usage_printf("attenuate 0..31|auto\r\n%4.2F\r\n", get_attenuation());
    return;
  }
  if (get_str_index(argv[0],"auto") == 0) {
    if (!setting.auto_attenuation)
      set_auto_attenuation();
  } else {
    int a = my_atoi(argv[0]);
//    if (a < 0 || a>31)
//      goto usage;
    if (get_attenuation() != a)
      set_attenuation(a);
  }
  redraw_request |= REDRAW_CAL_STATUS | REDRAW_AREA;
}

VNA_SHELL_FUNCTION(cmd_level)
{
  if (argc != 1  || argv[0][0] == '?') {
    if (setting.mode==M_GENLOW)
      usage_printf("level -76..-6\r\n");
    if (setting.mode==M_GENHIGH)
      usage_printf("level -38..13\r\n");
    return;
  }
  float f = my_atof(argv[0]);
  set_level(f);
}

VNA_SHELL_FUNCTION(cmd_sweeptime)
{
  if (argc != 1  || argv[0][0] == '?') {
    usage_printf("sweeptime 0.003..60\r\n%5.3Fs\r\n",((float)setting.actual_sweep_time_us)/ONE_SECOND_TIME);
    return;
  }
  float f = my_atof(argv[0]);
  set_sweep_time_us(f*ONE_SECOND_TIME);
}



VNA_SHELL_FUNCTION(cmd_ext_gain)
{
  if (argc != 1 || argv[0][0] == '?') {
    usage_printf("ext_gain -100.0..+100.0\r\n");
    return;
  }
  float o = my_atof(argv[0]);
  set_external_gain(o);
}




VNA_SHELL_FUNCTION(cmd_levelchange)
{
  if (argc != 1) {
#ifdef TINYSA4
    shell_printf("usage: levelchange -90..+90\r\n");
#else
    shell_printf("usage: levelchange -70..+70\r\n");
#endif
    return;
  }
  float f = my_atof(argv[0]);
  set_level_sweep(f);
}

VNA_SHELL_FUNCTION(cmd_leveloffset)
{
  //                                     0    1      2
#ifdef TINYSA4
  static const char cmd_mode_list[] = "low|switch|receive_switch|out_switch|lna|harmonic|shift|shift1|shift2|shift3|shift4|drive1|drive2|drive3|direct|direct_lna|ultra|ultra_lna|harmonic_lna|adf|direct_output|low_output";
#else
  static const char cmd_mode_list[] = "low|high|switch|receive_switch";
#endif
  if (argc == 0) {
    const char *p = "leveloffset %s %.1f\r\n";
#ifdef TINYSA3
    shell_printf(p, "low",          config.low_level_offset);
    shell_printf(p, "high",         config.high_level_offset);
    shell_printf(p, "low output",   config.low_level_output_offset);
    shell_printf(p, "high output",  config.high_level_output_offset);
    shell_printf(p, "switch",       config.switch_offset);
    shell_printf(p, "receive_switch",config.receive_switch_offset);
#endif
#ifdef TINYSA4
    shell_printf(p, "low",          config.low_level_offset);
    shell_printf(p, "low_output",   config.low_level_output_offset);
    shell_printf(p, "switch",       config.switch_offset);
    shell_printf(p, "receive_switch",config.receive_switch_offset);
    shell_printf(p, "out_switch",   config.out_switch_offset);
    shell_printf(p, "lna",          config.lna_level_offset);
    shell_printf(p, "harmonic",     config.harmonic_level_offset);
    shell_printf(p, "harmonic_lna", config.harmonic_lna_level_offset);
    shell_printf(p, "shift",        config.shift_level_offset);
    shell_printf(p, "shift1",       config.shift1_level_offset);
    shell_printf(p, "shift2",       config.shift2_level_offset);
    shell_printf(p, "shift3",       config.shift3_level_offset);
    shell_printf(p, "shift4",       config.shift4_level_offset);
    shell_printf(p, "drive1",       config.drive1_level_offset);
    shell_printf(p, "drive2",       config.drive2_level_offset);
    shell_printf(p, "drive3",       config.drive3_level_offset);
    shell_printf(p, "direct",       config.direct_level_offset);
    shell_printf(p, "direct_lna",   config.direct_lna_level_offset);
    shell_printf(p, "ultra",        config.ultra_level_offset);
    shell_printf(p, "ultra_lna",    config.ultra_lna_level_offset);
    shell_printf(p, "adf",          config.adf_level_offset);
    shell_printf(p, "direct_output",config.direct_level_output_offset);
#endif
    return;
  }
  if (argv[0][0] == '?')
    goto usage;

  int mode = get_str_index(argv[0], cmd_mode_list);
  if (mode < 0) goto usage;
  float v;
  if (argc == 2){
    v = my_atof(argv[1]);
    switch (mode){
#ifdef TINYSA3
      case 0: config.low_level_offset = v; break;
      case 1: config.high_level_offset = v; break;
      case 2: config.switch_offset = v; break;
      case 3: config.receive_switch_offset = v; break;
#endif
#ifdef TINYSA4
      case 0: config.low_level_offset = v; config.input_is_calibrated = true; break;
      case 1: config.switch_offset = v; break;
      case 2: config.receive_switch_offset = v; break;
      case 3: config.out_switch_offset = v; break;
      case 4: config.lna_level_offset = v; break;
      case 5: config.harmonic_level_offset = v; break;
      case 6: config.shift_level_offset = v; break;
      case 7: config.shift1_level_offset = v; break;
      case 8: config.shift2_level_offset = v; break;
      case 9: config.shift3_level_offset = v; break;
      case 10: config.shift4_level_offset = v; break;
      case 11: config.drive1_level_offset = v; break;
      case 12: config.drive2_level_offset = v; break;
      case 13: config.drive3_level_offset = v; break;
      case 14: config.direct_level_offset = v; break;
      case 15: config.direct_lna_level_offset = v; break;
      case 16: config.ultra_level_offset = v; break;
      case 17: config.ultra_lna_level_offset = v; break;
      case 18: config.harmonic_lna_level_offset = v; break;
      case 19: config.adf_level_offset = v; break;
      case 20: config.direct_level_output_offset = v; break;
      case 21: config.low_level_output_offset = v; break;
#endif
      default: goto usage;
    }
    dirty = true;
    return;
  }
  if (argc == 3 && get_str_index(argv[1], "output") == 0){
    v = my_atof(argv[2]);
    switch (mode){
      case 0: config.low_level_output_offset = v; config.output_is_calibrated = true; break;
      case 1: config.high_level_output_offset = v; break;
#ifdef TINYSA4
      case 19: config.direct_level_output_offset = v; break;
#endif
      default: goto usage;
    }
    dirty = true;
    return;
  }
usage:
  shell_printf("leveloffset [%s] {output} [-20..+20]\r\n", cmd_mode_list);
}

VNA_SHELL_FUNCTION(cmd_deviceid)
{
  if (argc == 0) {
    shell_printf("deviceid %d\r\n", config.deviceid);
    return;
  } else if (argc == 1) {
    if (argv[0][0] == '?')
      goto usage;
    float v = my_atoui(argv[0]);
    config.deviceid = v;
  } else {
  usage:
    usage_printf("deviceid [<number>]\r\n");
  }
}

#ifdef __SWEEP_OUTPUT__
VNA_SHELL_FUNCTION(cmd_sweep_voltage)
{
  float value;
  if (argc != 1 || argv[0][0] == '?') {
   usage_printf("sweep_voltage {value(0-3.3)}\r\n"\
                 "current value: %f\r\n", config.sweep_voltage);
    return;
  }
  value = my_atof(argv[0]);
  config.sweep_voltage = value;
}
#endif

#ifdef __NOISE_FIGURE__
VNA_SHELL_FUNCTION(cmd_nf)
{
  if (argc != 1  || argv[0][0] == '?') {
//usage:
    usage_printf("nf {value}\r\n"\
                 "%f\r\n", config.noise_figure);
    return;
  }
  config.noise_figure = my_atof(argv[0]);
  dirty = true;
}
#endif

VNA_SHELL_FUNCTION(cmd_rbw)
{
  if (argc != 1 || argv[0][0] == '?') {
  usage:
#ifdef TINYSA4
	usage_printf("rbw 0.2..850|auto\r\n%.1FHz\r\n", actual_rbw_x10*100.0);
#else
	usage_printf("rbw 2..600|auto\r\n%.1FHz\r\n", actual_rbw_x10*100.0);
#endif
	return;
  }
  if (get_str_index(argv[0], "auto|0")>=0) {
    if (setting.rbw_x10 != 0)
      set_RBW(0);
  } else {
    float a = my_atof(argv[0]);
    if (a < 0.2 ||
#ifdef TINYSA4
        a>850
#else
        a>600
#endif
        )
      goto usage;
    if (setting.rbw_x10 != a*10)
      set_RBW((int) ( a*10));
 }
}

VNA_SHELL_FUNCTION(cmd_if)
{
#ifdef TINYSA4
  char *t = "975M..979M";
#endif
  if (argc != 1 || argv[0][0] == '?') {
  usage:
#ifdef TINYSA4
  if (hw_if)
    t = "1067M..1073M";
  usage_printf("usage: if {%s}\r\n%QHz\r\n", t, setting.frequency_IF);
#else
    usage_printf("usage: if {433M..435M}\r\n%QHz\r\n", setting.frequency_IF);
#endif
    return;
  }
  freq_t a = (freq_t)my_atoi(argv[0]);
  freq_t f = DEFAULT_IF;
#ifdef TINYSA4
  if (hw_if)
    f = DEFAULT_IF_PLUS;
#endif
  if (a!= 0 &&( a < (f - (freq_t)5000000) || a>(f + (freq_t)5000000)))
    goto usage;
  setting.auto_IF = false;
  set_IF(a);
}

VNA_SHELL_FUNCTION(cmd_zero)
{
  if (argc != 1 || argv[0][0] == '?') {
    usage_printf("zero {level}\r\n%ddBm\r\n", config.ext_zero_level);
    return;
  }
  config.ext_zero_level = my_atoi(argv[0]);
}

#ifdef TINYSA4
VNA_SHELL_FUNCTION(cmd_direct)
{
  if (argc<1 || argc>2)
    goto usage;
  if (argv[0][0] == '?')
    goto usage;
  freq_t value = 0;
  if (argc == 2) value = my_atoui(argv[1]);
  // Parse direct {start|stop} {freq(Hz)}
  static const char direct_cmd[] = "start|stop|on|off";
  int type = get_str_index(argv[0], direct_cmd);
  switch(type) {
  case 0:
    config.direct_start = value;
    return;
  case 1:
    config.direct_stop = value;
    return;
  case 2:
    config.direct = true;
    return;
  case 3:
    config.direct = false;
    return;
  }
usage:
  usage_printf("direct {%s} {freq(Hz)}\r\n", direct_cmd);
}

VNA_SHELL_FUNCTION(cmd_if1)
{
  if (argc != 1 || argv[0][0] == '?') {
  usage:
    usage_printf("if1 {975M..979M}\r\n%QHz\r\n", config.frequency_IF1);
    return;
  } else {
    freq_t a = (freq_t)my_atoui(argv[0]);
    if (a!= 0 &&( a < (DEFAULT_IF - (freq_t)80000000) || a>(DEFAULT_IF + (freq_t)80000000)))
      goto usage;
    config.frequency_IF1 = a;
    config_save();
  }
}

VNA_SHELL_FUNCTION(cmd_actual_freq)
{
  if (argc != 1 || argv[0][0] == '?') {
    shell_printf("%D\r\n", config.setting_frequency_30mhz);
    return;
  } else {
    set_actual_freq(my_atoui(argv[0]));
  }
}


VNA_SHELL_FUNCTION(cmd_freq_correction)
{
  if (argc != 1 || argv[0][0] == '?') {
    shell_printf("%d ppb\r\n", (int)(((int64_t)config.setting_frequency_30mhz - (int64_t)3000000000ULL)/3));
    return;
  } else {
    set_freq_corr(my_atoi(argv[0]));
  }
}

#endif


#ifdef __DRAW_LINE__
VNA_SHELL_FUNCTION(cmd_line)
{
  if (argc != 1  || argv[0][0] == '?')
    goto usage;
  int type = get_str_index(argv[0], "off");
  switch(type) {
  case -1: {
      float l = my_atof(argv[0]);
      setting.draw_line = true;
      set_trigger_level(to_dBm(l));
      set_trigger(T_AUTO);
      redraw_request |= REDRAW_AREA;
    }
    return;
  case 0:
    setting.draw_line = false;
    redraw_request |= REDRAW_AREA;
    return;
  }
usage:
  usage_printf("line off|{level}\r\n");
}
#endif


#ifdef TINYSA3
VNA_SHELL_FUNCTION(cmd_actual_freq)
{
  if (argc != 1 || argv[0][0] == '?') {
    shell_printf("%DHz\r\n", config.setting_frequency_10mhz);
    return;
  } else {
    set_10mhz(my_atoui(argv[0]));
  }
}
#endif

VNA_SHELL_FUNCTION(cmd_trigger)
{
  if (argc == 0)
    goto usage;
  if(argv[0][0] == '?')
    goto usage;
  if (( '0' <= argv[0][0] && argv[0][0] <= '9') || argv[0][0] == '-') {
    float t = my_atof(argv[0]);
    if (setting.trigger == T_AUTO )
      set_trigger(T_NORMAL);
    set_trigger_level(to_dBm(t));
    goto update;
  }
  static const char cmd_trigger_list[] = "auto|normal|single";
  if (argc == 1) {
    int type = get_str_index(argv[0], cmd_trigger_list);
    if (type >= 0) {
      set_trigger(type);
      goto update;
    }
    goto usage;
  }
update:
  redraw_request |= REDRAW_CAL_STATUS | REDRAW_AREA;
  completed = true;
  return;
usage:
  shell_printf("trigger {value}\r\n"\
               "trigger {%s}\r\n" , cmd_trigger_list);
}


VNA_SHELL_FUNCTION(cmd_selftest)
{
  if (argc < 1 || argc > 2 || argv[0][0] == '?') {
    usage_printf("selftest (1-3) [arg]\r\n");
    return;
  }
  setting.test = my_atoi(argv[0]);
  if (argc == 1)
    setting.test_argument = 0;
  else
    setting.test_argument = my_atoui(argv[1]);
  sweep_mode = SWEEP_SELFTEST;
}

#ifdef __SINGLE_LETTER__

static int VFO = 0;


uint32_t xtoi(char *t)
{

  uint32_t v=0;
  while (*t) {
    if ('0' <= *t && *t <= '9')
      v = v*16 + *t - '0';
    else if ('a' <= *t && *t <= 'f')
      v = v*16 + *t - 'a' + 10;
    else if ('A' <= *t && *t <= 'F')
      v = v*16 + *t - 'A' + 10;
    else
      return v;
    t++;
  }
  return v;
}

#ifdef __ADF4351__

VNA_SHELL_FUNCTION(cmd_x)
{
  uint32_t reg;


  if (argc != 1) {
    usage_printf("x value(0-FFFFFFFF)\r\n");
    return;
  }
  reg = xtoi(argv[0]);

  if ((reg & 7) == 5) {
   if (reg & (1<<22))
      VFO = 1;
    else
      VFO = 0;
 //  reg &= ~0xc00000;    // Force led to show lock
 //  reg |=  0x400000;
  }

  ADF4351_WriteRegister32(VFO, reg);
  ADF4351_Latch();
  shell_printf("x=%x\r\n", reg);
}
#endif

VNA_SHELL_FUNCTION(cmd_i)
{
  (void)argc;
  (void)argv;
return;             // Don't use!!!!
#ifdef __SI4432__
int rvalue;
  SI4432_Init();
  shell_printf("SI4432 init done\r\n");
  if (argc == 1) {
    rvalue = my_atoui(argv[0]);
    set_switches(rvalue);
    set_mode(rvalue);
    shell_printf("SI4432 mode %d set\r\n", rvalue);
  }
#endif
}

VNA_SHELL_FUNCTION(cmd_o)
{
  (void) argc;
  freq_t value = my_atoi(argv[0]);
  if (VFO == 0)
    setting.frequency_IF = value;
  set_freq(VFO, value);
}

VNA_SHELL_FUNCTION(cmd_d)
{
  (void) argc;
  (void) argv;
  int32_t a = my_atoi(argv[0]);
#if false
  int32_t d;
  if (argc == 2)
    d = my_atoi(argv[1]);
  else {
    d = a;
    a = 2;
  }
  switch (a) {
  case 1:
    SI4463_set_output_level(d);
    break;
  case 2:
    ADF4351_drive (d);
    break;
  case 3:
    ADF4351_aux_drive(d);
    break;
  }
//  setting.lo_drive=a;
//  dirty = true;
#else
  setting.lo_drive=a;
  dirty = true;
#endif
}

#if 0
extern int16_t adc_buf_read(uint16_t *result, uint32_t count);

VNA_SHELL_FUNCTION(cmd_g)
{
  (void) argc;
  (void) argv;
  int32_t a = my_atoi(argv[0]);
  systime_t start_of_read = chVTGetSystemTimeX();
  adc_buf_read(spi_buffer, 256);
  systime_t time_of_read = chVTGetSystemTimeX() - start_of_read;
  shell_printf("Time: %d\r\n", time_of_read);
  for (int i=0;i<20;i++)
    shell_printf("[%d] = %d\r\n", (int)i, (int)(spi_buffer[i]));
}
#endif


VNA_SHELL_FUNCTION(cmd_a)
{
  (void)argc;
  if (argc != 1) {
    shell_printf("a=%U\r\n", frequencyStart);
    return;
  }
  freq_t value = my_atoui(argv[0]);
  frequencyStart = value;
}


VNA_SHELL_FUNCTION(cmd_b)
{
  (void)argc;
  if (argc != 1) {
    shell_printf("b=%U\r\n", frequencyStop);
    return;
  }
  freq_t value = my_atoui(argv[0]);
  frequencyStop = value;
}

VNA_SHELL_FUNCTION(cmd_t)
{
  (void)argc;
  (void)argv;
}

VNA_SHELL_FUNCTION(cmd_e)
{
  (void)argc;
  if (argc != 1) {
    shell_printf("e=%d\r\n", setting.tracking);
    return;
  }
  setting.tracking = my_atoi(argv[0]);
  if (setting.tracking == -1)
    setting.tracking = false;
  else
    setting.tracking = true;

  if (argc >1)
    frequencyExtra = my_atoi(argv[1]);
}

VNA_SHELL_FUNCTION(cmd_s)
{
  (void)argc;
  if (argc != 1) {
    shell_printf("s=%d\r\n", points);
    return;
  }
  points = my_atoi(argv[0]);
}


VNA_SHELL_FUNCTION(cmd_v)
{
    if (argc != 1) {
        shell_printf("%d\r\n", VFO);
        return;
    }
    VFO = my_atoi(argv[0]) > 0 ? 1 : 0;
    shell_printf("VFO %d\r\n", VFO);
}

VNA_SHELL_FUNCTION(cmd_y)
{
  if (argc < 1) {
    usage_printf("y {addr(0-95)} [value(0-0xFF)]\r\n");
    return;
  }
#ifdef __SI4432__
  int lvalue = 0;
  int rvalue;
  rvalue = xtoi(argv[0]);
  SI4432_Sel = VFO;
  if (argc == 2){
    lvalue = my_atoui(argv[1]);
    SI4432_Write_Byte(rvalue, lvalue);
  } else {
    lvalue = SI4432_Read_Byte(rvalue);
    shell_printf("%x\r\n", lvalue);
  }
#endif
#ifdef __SI4463__
  uint8_t data[16];
  data[0] = xtoi(argv[0]);
  for (int i=1; i < argc; i++) {
    data[i] = xtoi(argv[i]);
  }
  SI4463_do_api(data, argc, data, 16);
  for (int i=0; i<16; i++)
    shell_printf("%02x ", data[i]);
  shell_printf("\r\n");
#endif
}
#ifdef TINYSA4
VNA_SHELL_FUNCTION(cmd_z)
{
  if (argc != 1) {
    usage_printf("z 0..30000\r\n%d\r\n", SI4432_step_delay);
    return;
  }
  if (argc == 1) {
    setting.step_delay = my_atoi(argv[0]);
    dirty = true;
  }
}

VNA_SHELL_FUNCTION(cmd_n)
{
  if (argc != 1) {
    usage_printf("z 0..30000\r\n%d\r\n", SI4432_offset_delay);
    return;
  }
  if (argc == 1) {
    setting.offset_delay = my_atoi(argv[0]);
    dirty = true;
  }
}
#endif
#if 0       // not used
VNA_SHELL_FUNCTION(cmd_z)
{
  static const char cmd_z_list[] = "t|r|i";
  if (argc != 1) {
    usage_printf("z %s\r\n", cmd_z_list);
    return;
  }
  if (argc == 1) {
#ifdef __SI4432__
    SI4432_Sel = VFO;
    int type = get_str_index(argv[0], cmd_z_list);
    switch(type) {
    case 0:
      SI4432_Transmit(3);
      break;
    case 1:
      SI4432_Receive();
      break;
    case 2:
      SI4432_Reset();
      break;
    }
#endif
  }
}
#endif


#endif

#ifdef __SINGLE_LETTER__

void sweep_remote(void)
{
  uint32_t i;
  uint32_t step = (points - 1);
  freq_t span = frequencyStop - frequencyStart;
  freq_t delta = span / step;
  freq_t error = span % step;
  freq_t f = frequencyStart - setting.frequency_IF, df = step>>1;
  freq_t old_step = setting.frequency_step;
  setting.frequency_step = delta;
  streamPut(shell_stream, '{');
  dirty = true;
  for (i = 0; i <= step; i++, f+=delta) {
    if (operation_requested)
      break;
    int val = perform(false, i, f, false) + float_TO_PURE_RSSI(config.ext_zero_level);
    streamPut(shell_stream, 'x');
    streamPut(shell_stream, (uint8_t)(val & 0xFF));
    streamPut(shell_stream, (uint8_t)((val>>8) & 0xFF));
    df+=error;if (df >=step) {f++;df -= step;}
  }
  streamPut(shell_stream, '}');
  setting.frequency_step = old_step;
  sweep_mode = 0;
}

VNA_SHELL_FUNCTION(cmd_m)
{
  (void)argc;
  (void)argv;

//  set_mode(0);
//  setting.tracking = false; //Default test setup
//  setting.step_atten = false;
//  set_attenuation(0);
//  set_reflevel(-10);
//  set_sweep_frequency(ST_START,frequencyStart - setting.frequency_IF );
//  set_sweep_frequency(ST_STOP, frequencyStop - setting.frequency_IF);
//  draw_cal_status();

  pause_sweep();
//  update_rbw();
  chThdSleepMilliseconds(10);
  sweep_mode = SWEEP_REMOTE;
  while (sweep_mode != 0)
    chThdSleepMilliseconds(10);
  //  update_rbw();
}

VNA_SHELL_FUNCTION(cmd_p)
{
  (void)argc;
  int p = my_atoi(argv[0]);
#ifdef TINYSA4
  SI4463_set_output_level(p);
#endif
return;
  int a = my_atoi(argv[1]);
  if (p==5)
    set_attenuation(-a);
  if (p==6)
    set_mode(a);
  if (p==1)
    if (get_refer_output() != a)
      set_refer_output(a);
}

#ifdef TINYSA4
VNA_SHELL_FUNCTION(cmd_g)
{
  (void)argc;
  int p = my_atoi(argv[0]);
  int a = my_atoi(argv[1]);
  SI4463_set_gpio(p,a);
}

VNA_SHELL_FUNCTION(cmd_r)
{
  (void)argc;
  int r = my_atoi(argv[0]);
  set_R(r);
}

VNA_SHELL_FUNCTION(cmd_c)
{
  (void)argc;
  int r = my_atoi(argv[0]);
  ADF4351_CP(r);
}

VNA_SHELL_FUNCTION(cmd_h)
{
  (void)argc;
  int r = my_atoi(argv[0]);
  set_modulo(r);
}




#endif

VNA_SHELL_FUNCTION(cmd_w)
{
  (void)argc;
  int p = my_atoi(argv[0]);
return;
  set_RBW(p*10);
}

VNA_SHELL_FUNCTION(cmd_u)
{
  (void)argc;
  (void)argv;
//  int p = my_atoi(argv[0]);
  toggle_debug_avoid();
}

VNA_SHELL_FUNCTION(cmd_f)
{
  (void)argc;
  setting.test = 5;
  setting.test_argument =  my_atoi(argv[0]);;
  sweep_mode = SWEEP_SELFTEST;
}
#endif

//#ifdef  DIRECT_CORRECTION
//  #define CORRECTION_DIRECT         4
//  #define CORRECTION_LNA_DIRECT     5
//#endif
//  #define CORRECTION_LOW_OUT        6
//  #define CORRECTION_LOW_OUT_DIRECT 7
//  #define CORRECTION_LOW_OUT_ADF    8
//  #define CORRECTION_LOW_OUT_MIXER  9
//  #define CORRECTION_HIGH           10
//  #define CORRECTION_SIZE           11

VNA_SHELL_FUNCTION(cmd_correction)
{
  (void)argc;
#ifdef TINYSA4
  static const char cmd[] = "low|lna|ultra|ultra_lna|direct|direct_lna|harm|harm_lna|out|out_direct|out_adf|out_ultra|off|on";
  static const char range[] = "0-19";
#else
  static const char cmd[] = "low|high|out";
  static const char range[] = "0-9";
#endif
  int m = get_str_index(argv[0], cmd);
  if (argc == 1 && m >=0) {
#ifdef TINYSA4
    switch(m) {
    case CORRECTION_SIZE: // Off
      setting.disable_correction = true;
      goto show;
    case CORRECTION_SIZE+1: // on
      setting.disable_correction = false;
    show:
      dirty = true;       // recalculate intermediate table
      shell_printf("correction %s\r\n", (setting.disable_correction ? "off":"on"));
      return;
    }
#endif
    shell_printf("index frequency value\r\n");
    for (int i=0; i<CORRECTION_POINTS; i++) {
      shell_printf("correction %s %d %D %.1f\r\n", argv[0], i, config.correction_frequency[m][i], config.correction_value[m][i]);
    }
    return;
  }
  if (argc == 2 && (get_str_index(argv[1],"reset") == 0)) {
    for (int i=0; i<CORRECTION_POINTS; i++) {
      config.correction_value[m][i] = 0.0;
    }
    dirty = true;       // recalculate intermediate table
    shell_printf("correction table %s reset\r\n", argv[0]);
    return;
  }
  if (argc != 4) {
    usage_printf("correction %s %s frequency(Hz) value(dB)\r\n", cmd, range);
    return;
  }
  int i = my_atoi(argv[1]);
  freq_t f = my_atoui(argv[2]);
  float v = my_atof(argv[3]);
  config.correction_frequency[m][i] = f;
  config.correction_value[m][i] = v;
  dirty = true;       // recalculate intermediate table
  redraw_request|=REDRAW_AREA;                  // to ensure the change in level will be visible
  shell_printf("updated %d to %D %.1f\r\n", i, config.correction_frequency[m][i], config.correction_value[m][i]);
}

VNA_SHELL_FUNCTION(cmd_abort)
{
  static const char cmd_on_off[] = "off|on";
  if (argc > 1) {
    usage:
    shell_printf("usage: abort [%s]\r\n", cmd_on_off);
    return;
  }
  if (argc == 1) {
    int i = get_str_index(argv[0], cmd_on_off);
    if (i < 0)
      goto usage;
    abort_enabled = i;
  } else
    operation_requested = OP_CONSOLE;
}

VNA_SHELL_FUNCTION(cmd_scanraw)
{
  freq_t start, stop;
  uint32_t points = sweep_points;
  uint8_t unbuffered = 0;
  if (argc < 2 || argc > 4) {
    usage_printf("scanraw {start(Hz)} {stop(Hz)} [points] [options]\r\n");
    return;
  }

  start = my_atoui(argv[0]);
  stop = my_atoui(argv[1]);
  if (start > stop) {
      shell_printf("frequency range is invalid\r\n");
      return;
  }
  if (argc > 2) {
    points = my_atoi(argv[2]);
  }

  if (argc > 3) {
    unbuffered = my_atoi(argv[3]); // arg != 0 -> ubuffered
  }

  if (setting.waterfall)
    disable_waterfall();            // display dma hangs when waterfall is enabled

  freq_t old_step = setting.frequency_step;
  float f_step = (stop-start)/ points;
  setting.frequency_step = (freq_t)f_step;
again:
  operation_requested = false;
  dirty = true;
//  adc_stop_analog_watchdog();
  int oldpos = 0;
  ili9341_set_background(LCD_BG_COLOR);
  ili9341_fill(OFFSETX, CHART_BOTTOM+1, WIDTH, 1);
  ili9341_set_background(LCD_SWEEP_LINE_COLOR);
#define BUFFER_SIZE  64
  uint8_t buf[BUFFER_SIZE];
  int idx = 0;
  buf[idx++] = '{';
  for (uint32_t i = 0; i < points; i++) {
    int val = perform(false, i, start +(freq_t)(f_step * i), false) + float_TO_PURE_RSSI(config.ext_zero_level);
    if (operation_requested || ((uint32_t)shell_stream == (uint32_t)&SDU1 && SDU1.config->usbp->state != USB_ACTIVE)) // break on operation in perform
      break;
    buf[idx++] = 'x';
    buf[idx++] = (uint8_t)(val & 0xFF);
    buf[idx++] = (uint8_t)((val>>8) & 0xFF);
    if ((unbuffered & 1) || idx >= BUFFER_SIZE - 4) {
      streamWrite(shell_stream, buf, idx);
      idx = 0;
    }
    int pos = i * (WIDTH+1) / points;
    if (pos - oldpos > 8) {
      ili9341_fill(OFFSETX + oldpos, CHART_BOTTOM+1, pos - oldpos, 1);     // update sweep progress bar
      oldpos = pos;
    }
  }
  buf[idx++] = '}';
  streamWrite(shell_stream, buf, idx);
//  adc_start_analog_watchdog();
  if ((unbuffered & 2) && !operation_requested)
     goto again;
  ili9341_set_background(LCD_BG_COLOR);
  ili9341_fill(OFFSETX, CHART_BOTTOM+1, WIDTH, 1);
  setting.frequency_step = old_step;
  dirty = true;
  redraw_request = 0; // disable screen update in this mode
}

VNA_SHELL_FUNCTION(cmd_caloutput)
{
  static const char cmd[] = "off|30|15|10|4|3|2|1";
  if (argc != 1) {
    usage_printf("caloutput %s\r\n", cmd);
    return;
  }
  int m = get_str_index(argv[0], cmd);
  if (m != -1)
    set_refer_output(m - 1);
}

#ifdef TINYSA4
VNA_SHELL_FUNCTION(cmd_q)
{
  if (argc < 1) {
    usage:
    usage_printf("q [s0..1|d-1,0..18|a0..63|p0..4]\r\n");
    force_signal_path=false;
    test_output_switch = false;
    test_output_drive = 0;
    test_output_attenuate = 0;
    test_path = 0;
    dirty = true;
    return;
  }
  int i = 0;
  force_signal_path=true;
  dirty = true;
again:
  if (argc == 0)
    return;
  char *a = argv[i++];
  char m = *a++;
  argc--;
  switch (m) {
  default: goto usage;
  case 's':  test_output_switch = *a - '0'; break;
  case 'd': test_output_drive = my_atoi(a); break;
  case 'a': test_output_attenuate = my_atoi(a); break;
#ifdef TINYSA4
  case 'p': test_path = *a - '0'; break;
#endif
  }
  goto again;
}
#endif

#ifdef TINYSA4

extern float Si446x_get_temp(void);
VNA_SHELL_FUNCTION(cmd_k)
{
  (void)argc;
  (void)argv;
  shell_printf("%.2f\r\n", Si446x_get_temp());
}


#endif


#pragma GCC pop_options


