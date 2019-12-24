//! @file
//! @date Jun 10, 2019
//! @author Marek Bel

#pragma once

#include <stdint.h>

#define DEBUG_LAYER1CAL
#define DEBUG_OUT ENABLED(DEBUG_LAYER1CAL)
#include "../core/debug_out.h"

#define TASK_INTERVAL_MS 300

class FirstLayerCalculate {
  public:
    FirstLayerCalculate():flag(false),i(0),j(0),  step(0),filament(0){};
    void start(uint8_t option);
    void stop();
    void quit();
    void task();

  private:
    int8_t enqueue_commands(const char * pgcode);
    int8_t enqueue_commands_P(const char * const pgcode);
    bool preheat();
    void load_filament(char *cmd_buffer);
    void print_intro();
    bool setup_meander();
    bool print_meander(char *cmd_buffer);
    bool print_square(char *cmd_buffer, uint8_t i);
    bool print_all_square(char *cmd_buffer, uint8_t i1, uint8_t i2);
    bool end();

  private:
    bool flag;
    uint8_t i;
    uint8_t j;
    uint8_t step;
    uint8_t filament;
};

extern FirstLayerCalculate layer1Cal;
