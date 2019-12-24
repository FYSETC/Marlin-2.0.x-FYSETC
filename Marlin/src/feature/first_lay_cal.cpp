#include "../inc/MarlinConfig.h"

#ifdef FIRST_LAYER_CAL

#include "first_lay_cal.h"
#include "../gcode/queue.h"
#include "../module/temperature.h"
#if ENABLED(EXTENSIBLE_UI)
  #include "../lcd/extensible_ui/ui_api.h"
#endif

FirstLayerCalculate layer1Cal;

void FirstLayerCalculate::start(uint8_t option) {
  flag = true;  
  if(option > 4) option = 0; // Only 4 filaments supported
  filament = option;
  step = 0;  
}

void FirstLayerCalculate::stop() {
  queue.clear();
  #if HOTENDS >= 1
    thermalManager.setTargetHotend(0, 0);
  #endif
  #if HAS_HEATED_BED
    thermalManager.setTargetBed(0);
  #endif
  step = 0;  
  flag = false;
  i = 0;
  j = 0;
  #if ENABLED(EXTENSIBLE_UI)
    ExtUI::OnLayer1Done();
  #endif
}

void FirstLayerCalculate::quit() {
  if(step >= 7) return;
  queue.clear();
  i = 0;
  j = 0;
  step = 7;
}

void FirstLayerCalculate::task(void) {
  if(!flag) return;

	char cmd1[30];

  const millis_t ms = millis();
  static millis_t next_task_ms = 0;

  if (ELAPSED(ms, next_task_ms)) {
    next_task_ms = ms + TASK_INTERVAL_MS;
    if (!queue.has_commands_queued()) {
      switch(step) {
      case 0:
        DEBUG_ECHOLNPGM("step 0");
        step = 1;        
        break;    
      case 1:
        DEBUG_ECHOLNPGM("step 1");
        if(preheat()) step = 2;        
        break;
      case 2:
        DEBUG_ECHOLNPGM("step 2");
        load_filament(cmd1);
        step = 3;        
        break;
      case 3:
        DEBUG_ECHOLNPGM("step 3");
        #if ENABLED(EXTENSIBLE_UI)
          ExtUI::OnLayer1PreheatDone();
        #endif         
        print_intro();
        step = 4;        
        break;
      case 4:
        DEBUG_ECHOLNPGM("step 4");
        if(setup_meander()) step = 5;        
        break;
      case 5:
        DEBUG_ECHOLNPGM("step 5");
        if(print_meander(cmd1)) step = 6;        
        break;
      case 6:
        DEBUG_ECHOLNPGM("step 6");
        if(print_all_square(cmd1,0,16)) step = 7;
        break;
      case 7:
        DEBUG_ECHOLNPGM("step 7");
        if(end()) step = 8;
        break;
      case 8:
        DEBUG_ECHOLNPGM("step 8");
        #if ENABLED(EXTENSIBLE_UI)
          ExtUI::OnLayer1Done();
        #endif

        step = 0;
        i = 0;
        j = 0;
        flag = false;        
        break;
      }
    }
  }
}

int8_t FirstLayerCalculate::enqueue_commands(const char * pgcode) {
  if (queue.length < BUFSIZE) {
    queue.enqueue_one_now(pgcode);
    DEBUG_ECHOLNPGM("enqueue_commands");
    return 0;
  }
  else {
    DEBUG_ECHOLNPGM("enqueue_commands NO");
    return -1;
  }
}

int8_t FirstLayerCalculate::enqueue_commands_P(const char * const pgcode) {
  if (queue.length < BUFSIZE) {
    queue.enqueue_now_P(pgcode);
    DEBUG_ECHOLNPGM("enqueue_commands_P");
    return 0;
  }
  else {
    DEBUG_ECHOLNPGM("enqueue_commands_P NO");
    return -1;
  }
}

//! @brief Preheat
bool FirstLayerCalculate::preheat() {
  DEBUG_ECHOLNPAIR("f:",filament);

  static const char cmd_preheat_0[] PROGMEM = "M107";
  
  static const char cmd_preheat_1[] PROGMEM = "M104 S" STRINGIFY(PREHEAT_1_TEMP_HOTEND);
  static const char cmd_preheat_2[] PROGMEM = "M140 S" STRINGIFY(PREHEAT_1_TEMP_BED);
  static const char cmd_preheat_3[] PROGMEM = "M190 S" STRINGIFY(PREHEAT_1_TEMP_BED);
  static const char cmd_preheat_4[] PROGMEM = "M109 S" STRINGIFY(PREHEAT_1_TEMP_HOTEND);
  
  static const char cmd_preheat_5[] PROGMEM = "M104 S" STRINGIFY(PREHEAT_2_TEMP_HOTEND);
  static const char cmd_preheat_6[] PROGMEM = "M140 S" STRINGIFY(PREHEAT_2_TEMP_BED);
  static const char cmd_preheat_7[] PROGMEM = "M190 S" STRINGIFY(PREHEAT_2_TEMP_BED);
  static const char cmd_preheat_8[] PROGMEM = "M109 S" STRINGIFY(PREHEAT_2_TEMP_HOTEND);
  
  static const char cmd_preheat_9[] PROGMEM = "M104 S" STRINGIFY(PREHEAT_3_TEMP_HOTEND);
  static const char cmd_preheat_10[] PROGMEM = "M140 S" STRINGIFY(PREHEAT_3_TEMP_BED);
  static const char cmd_preheat_11[] PROGMEM = "M190 S" STRINGIFY(PREHEAT_3_TEMP_BED);
  static const char cmd_preheat_12[] PROGMEM = "M109 S" STRINGIFY(PREHEAT_3_TEMP_HOTEND);
  
  static const char cmd_preheat_13[] PROGMEM = "M104 S" STRINGIFY(PREHEAT_4_TEMP_HOTEND);
  static const char cmd_preheat_14[] PROGMEM = "M140 S" STRINGIFY(PREHEAT_4_TEMP_BED);
  static const char cmd_preheat_15[] PROGMEM = "M190 S" STRINGIFY(PREHEAT_4_TEMP_BED);
  static const char cmd_preheat_16[] PROGMEM = "M109 S" STRINGIFY(PREHEAT_4_TEMP_HOTEND);

  static const char cmd_preheat_17[] PROGMEM = "G28";
  static const char cmd_preheat_18[] PROGMEM = "G29";
  static const char cmd_preheat_19[] PROGMEM = "G1 X15 Y15 F6000";
  static const char cmd_preheat_20[] PROGMEM = "G92 E0.0";
  
  static const char cmd_preheat_21[] PROGMEM = "M117 First layer cal.";

  static const char * const preheat_cmd[][10] PROGMEM = {
    { cmd_preheat_0, cmd_preheat_1, cmd_preheat_2, cmd_preheat_3, cmd_preheat_4, cmd_preheat_21, cmd_preheat_17, cmd_preheat_18,cmd_preheat_19, cmd_preheat_20 },
    { cmd_preheat_0, cmd_preheat_5, cmd_preheat_6, cmd_preheat_7, cmd_preheat_8, cmd_preheat_21, cmd_preheat_17, cmd_preheat_18,cmd_preheat_19, cmd_preheat_20 },
    { cmd_preheat_0, cmd_preheat_9, cmd_preheat_10, cmd_preheat_11, cmd_preheat_12, cmd_preheat_21, cmd_preheat_17, cmd_preheat_18,cmd_preheat_19, cmd_preheat_20 },
    { cmd_preheat_0, cmd_preheat_13, cmd_preheat_14, cmd_preheat_15, cmd_preheat_16, cmd_preheat_21, cmd_preheat_17, cmd_preheat_18,cmd_preheat_19, cmd_preheat_20 }
  };

  for (; i < 10; ++i) {
    if(0 > enqueue_commands_P(static_cast<const char*>(pgm_read_ptr(&preheat_cmd[filament-1][i])))) {
      return false;
    }
  }

  i = 0;
  return true;
}

//! @brief Load filament
//! @param cmd_buffer character buffer needed to format gcodes
//! @param filament filament to use (applies for MMU only)
void FirstLayerCalculate::load_filament(char *cmd_buffer)
{
  i = 0;
}

//! @brief Print intro line
void FirstLayerCalculate::print_intro()
{
  static const char cmd_intro_line_0[] PROGMEM = "G1 Z0.30";  
  static const char cmd_intro_line_1[] PROGMEM = "G1 X60.0 E9.0 F1000.0";
  static const char cmd_intro_line_2[] PROGMEM = "G1 X100.0 E12.5 F1000.0";

  static const char * const cmd_intro_line[] PROGMEM = {
    cmd_intro_line_0,
    cmd_intro_line_1,
    cmd_intro_line_2,
  };
  
  for (; i < COUNT(cmd_intro_line); ++i) {
    if(0 > enqueue_commands_P(static_cast<const char*>(pgm_read_ptr(&cmd_intro_line[i])))) {
      return;
    }    
  }

  i = 0;
}

//! @brief Setup for printing meander
bool FirstLayerCalculate::setup_meander() {
  
  static const char cmd_pre_meander_0[] PROGMEM = "G92 E0.0";
  static const char cmd_pre_meander_1[] PROGMEM = "G21"; //set units to millimeters TODO unsupported command
  static const char cmd_pre_meander_2[] PROGMEM = "G90"; //use absolute coordinates
  static const char cmd_pre_meander_3[] PROGMEM = "M83"; //use relative distances for extrusion TODO: duplicate
  static const char cmd_pre_meander_4[] PROGMEM = "G1 E-1.50000 F2100.00000";
  static const char cmd_pre_meander_5[] PROGMEM = "G1 Z5 F7200.000";
  static const char cmd_pre_meander_6[] PROGMEM = "M204 S1000"; //set acceleration
  static const char cmd_pre_meander_7[] PROGMEM = "G1 F4000";

  static const char * const cmd_pre_meander[] PROGMEM = {
    cmd_pre_meander_0,
    cmd_pre_meander_1,
    cmd_pre_meander_2,
    cmd_pre_meander_3,
    cmd_pre_meander_4,
    cmd_pre_meander_5,
    cmd_pre_meander_6,
    cmd_pre_meander_7,
  };

  for (; i < COUNT(cmd_pre_meander); ++i) {
    if(0 > enqueue_commands_P(static_cast<const char*>(pgm_read_ptr(&cmd_pre_meander[i])))) {
      return false;
    }    
  }

  i = 0;
  return true;
}


//! @brief Count extrude length
//!
//! @param layer_height layer height in mm
//! @param extrusion_width extrusion width in mm
//! @param extrusion_length extrusion length in mm
//! @return filament length in mm which needs to be extruded to form line
static constexpr float count_e(float layer_height, float extrusion_width, float extrusion_length) {
  return (extrusion_length * layer_height * extrusion_width / (M_PI * pow(1.75, 2) / 4));
}

static const float width = 0.4; //!< line width
static const float length = 20 - width; //!< line length
static const float height = 0.2; //!< layer height TODO This is wrong, as current Z height is 0.15 mm
static const float extr = count_e(height, width, length); //!< E axis movement needed to print line

//! @brief Print meander
//! @param cmd_buffer character buffer needed to format gcodes
bool FirstLayerCalculate::print_meander(char *cmd_buffer) {
  //static uint8_t i = 0;
  char str_1[16];

  static const char cmd_meander_0[] PROGMEM = "G1 X50 Y155";
  static const char cmd_meander_1[] PROGMEM = "G1 Z0.30 F7200.000";
  static const char cmd_meander_2[] PROGMEM = "G1 F1080";
  static const char cmd_meander_3[] PROGMEM = "G1 X75 Y155 E2.5";
  static const char cmd_meander_4[] PROGMEM = "G1 X100 Y155 E2";
  static const char cmd_meander_5[] PROGMEM = "G1 X200 Y155 E2.62773";
  static const char cmd_meander_6[] PROGMEM = "G1 X200 Y135 E0.66174";
  static const char cmd_meander_7[] PROGMEM = "G1 X50 Y135 E3.62773";
  static const char cmd_meander_8[] PROGMEM = "G1 X50 Y115 E0.49386";
  static const char cmd_meander_9[] PROGMEM = "G1 X200 Y115 E3.62773";
  static const char cmd_meander_10[] PROGMEM = "G1 X200 Y95 E0.49386";
  static const char cmd_meander_11[] PROGMEM = "G1 X50 Y95 E3.62773";
  static const char cmd_meander_12[] PROGMEM = "G1 X50 Y75 E0.49386";
  static const char cmd_meander_13[] PROGMEM = "G1 X200 Y75 E3.62773";
  static const char cmd_meander_14[] PROGMEM = "G1 X200 Y55 E0.49386";
  static const char cmd_meander_15[] PROGMEM = "G1 X50 Y55 E3.62773";

  static const char * const cmd_meander[] PROGMEM = {
    cmd_meander_0,
    cmd_meander_1,
    cmd_meander_2,
    cmd_meander_3,
    cmd_meander_4,
    cmd_meander_5,
    cmd_meander_6,
    cmd_meander_7,
    cmd_meander_8,
    cmd_meander_9,
    cmd_meander_10,
    cmd_meander_11,
    cmd_meander_12,
    cmd_meander_13,
    cmd_meander_14,
    cmd_meander_15,
  };

  for (; i < COUNT(cmd_meander); ++i) {
    if(0 > enqueue_commands_P(static_cast<const char*>(pgm_read_ptr(&cmd_meander[i])))) {
      return false;
    }
  }

  dtostrf(extr, 2, 3, str_1);
  sprintf_P(cmd_buffer, PSTR("G1 X50 Y35 E%s"), str_1);
  if(0 > enqueue_commands(cmd_buffer)) {
    return false;
  }

  i = 0;
  return true;
}

//! @brief Print square
//!
//! This function needs to be called 16 times for i from 0 to 15.
//!
//! @param cmd_buffer character buffer needed to format gcodes
//! @param i iteration
bool FirstLayerCalculate::print_square(char *cmd_buffer, uint8_t ind) {
  //static uint8_t j = 0;
  char str_1[16], str_e[16],str_es[16];;

  const float extr_short_segment = count_e(height, width, width);

  static const char fmt1[] PROGMEM = "G1 X%d Y%s E%s";
  static const char fmt2[] PROGMEM = "G1 Y%s E%s";

  dtostrf(extr, 2, 3, str_e);
  dtostrf(extr_short_segment, 2, 3, str_es);
  
  if(j == 0) {
    dtostrf((35 - ind*width * 2), 2, 2, str_1);
    sprintf_P(cmd_buffer, fmt1, 70, str_1, str_e);
    if(0 > enqueue_commands(cmd_buffer)) {
      return false;
    }
    j++;
  }

  if(j == 1) {
    dtostrf((35 - (2 * ind + 1)*width), 2, 2, str_1);
    sprintf_P(cmd_buffer, fmt2, str_1, str_es);
    if(0 > enqueue_commands(cmd_buffer)) {
      return false;
    }
    j++;
  }

  if(j == 2) {
    dtostrf((35 - (2 * ind + 1)*width), 2, 2, str_1);
    sprintf_P(cmd_buffer, fmt1, 50, str_1, str_e);
    if(0 > enqueue_commands(cmd_buffer)) {
      return false;
    }
    j++;
  }

  if(j == 3) {
    dtostrf((35 - (ind + 1)*width * 2), 2, 2, str_1);
    sprintf_P(cmd_buffer, fmt2, str_1, str_es);
    if(0 > enqueue_commands(cmd_buffer)) {
      return false;
    }
    j++;
  }

  j = 0;
  return true;
}

bool FirstLayerCalculate::print_all_square(char *cmd_buffer, uint8_t i1, uint8_t i2) {
  for (; i < i2; i++) {
    if(!print_square(cmd_buffer, i)) {
      return false;
    }
  }

  i = 0;
  return true;
}

bool FirstLayerCalculate::end() {
  //static uint8_t i = 0;

  static const char cmd_end_0[] PROGMEM = "M500";
  static const char cmd_end_1[] PROGMEM = "M107";
  //static const char cmd_end_2[] PROGMEM = "G1 E-0.07500 F2100.00000";
  static const char cmd_end_2[] PROGMEM = "M104 S0";
  static const char cmd_end_3[] PROGMEM = "M140 S0";
  static const char cmd_end_4[] PROGMEM = "G1 Z10 F1300.000";
  static const char cmd_end_5[] PROGMEM = "G1 X10 Y180 F4000";
  //static const char cmd_end_6[] PROGMEM = "G28";
  static const char cmd_end_6[] PROGMEM = "M84";
  
  static const char * const cmd_end[] PROGMEM = {
    cmd_end_0,
    cmd_end_1,
    cmd_end_2,
    cmd_end_3,
    cmd_end_4,
    cmd_end_5,
    cmd_end_6
  };

  for (; i < COUNT(cmd_end); ++i) {
    if(0 > enqueue_commands_P(static_cast<const char*>(pgm_read_ptr(&cmd_end[i])))) {
      return false;
    }
  }

  i = 0;
  j = 0;
  return true;
}


#endif
