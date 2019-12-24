/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "../../../inc/MarlinConfig.h"

#if ENABLED(SINGLE_Z_CALIBRATION)

  #include "../../gcode.h"
  #include "../../../feature/tmc_util.h"
  #include "../../../module/stepper/indirection.h"
  #include "../../../module/motion.h"
  #if ENABLED(EXTENSIBLE_UI)
    #include "../../../lcd/extensible_ui/ui_api.h"
  #endif
  void GcodeSuite::M915() {
    const uint16_t _rms = parser.seenval('S') ? parser.value_int() : CALIBRATION_CURRENT,
                   _z = parser.seenval('Z') ? parser.value_linear_units() : CALIBRATION_EXTRA_HEIGHT;

    if (!TEST(axis_known_position, Z_AXIS)) {
      SERIAL_ECHOLNPGM("\nPlease home Z axis first");
      return;
    }

    #if AXIS_IS_TMC(Z)
      const uint16_t Z_current_1 = stepperZ.rms_current();
      stepperZ.rms_current(_rms);
    #endif
    #if AXIS_IS_TMC(Z2)
      const uint16_t Z2_current_1 = stepperZ2.rms_current();
      stepperZ2.rms_current(_rms);
    #endif

    SERIAL_ECHOPAIR("\nCalibration current: Z", _rms);

    soft_endstops_enabled = false;

    do_blocking_move_to_z(Z_MAX_POS+_z);

    #if AXIS_IS_TMC(Z)
      stepperZ.rms_current(Z_current_1);
    #endif
    #if AXIS_IS_TMC(Z2)
      stepperZ2.rms_current(Z2_current_1);
    #endif

    do_blocking_move_to_z(Z_MAX_POS);
    soft_endstops_enabled = true;

    SERIAL_ECHOLNPGM("\nHoming Z due to lost steps");
  }
  
  void GcodeSuite::M920() {
    #if ENABLED(EXTENSIBLE_UI)
      ExtUI::OnSigleZCalibrationDone();
    #endif
  }
#endif // SINGLE_Z_CALIBRATION