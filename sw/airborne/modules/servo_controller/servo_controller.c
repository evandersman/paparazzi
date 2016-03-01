/*
 * Copyright (C) 2010 The Paparazzi Team
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file modules/sensors/turbulence_adc.c
 * Read an airspeed or differential pressure sensor via onboard ADC.
 */

#include "modules/sensors/potentiometer_adc.h"
#include "modules/servo_controller/servo_controller.h"
#include BOARD_CONFIG
#include "generated/airframe.h"
#include "state.h"
#include <stdio.h>
#include "std.h"
#include "inter_mcu.h"
#include "subsystems/commands.h"
#include "mcu_periph/sys_time.h"

struct servo_data left_wing;
struct servo_data right_wing;

float pgain;
uint16_t servo_count;

float pot_left_wing_scaled;
float pot_right_wing_scaled;
float left_wing_last_err;

void servo_controller_init(void)
{
  pot_left_wing_scaled  = 0.;
  pot_right_wing_scaled = 0.;
  left_wing_last_err    = 0.;
  left_wing.err         = 0.;
  left_wing.d_err       = 0.;
  left_wing.cmd         = 0.;

  servo_count           = 0;

  left_wing.pgain       = SERVO_PGAIN;
  left_wing.dgain       = SERVO_DGAIN;
  left_wing.igain       = SERVO_IGAIN;
  left_wing.offset      = POT_LEFT_OFFSET;
  left_wing.gain        = POT_LEFT_GAIN;

  right_wing.pgain      = SERVO_PGAIN;
  right_wing.igain      = SERVO_IGAIN;
  right_wing.offset     = POT_RIGHT_OFFSET;
  right_wing.gain       = POT_RIGHT_GAIN;
}

void servo_controller_update(void)
{
  pot_left_wing_scaled = -(potentiometer_adc_raw_left - left_wing.offset) * left_wing.gain;
  //Bound(pot_right_wing_scaled, -9600, 9600);
  pot_right_wing_scaled = -(potentiometer_adc_raw_right - right_wing.offset) * right_wing.gain;
  //Bound(pot_right_wing_scaled, -9600, 9600);
  
  // debug
/*
  if (servo_count < 1) {
  left_wing.pwm_cw = -8000;
  left_wing.pwm_ccw = -9600;
  servo_count = 1;
  }
  else {
  left_wing.pwm_cw = -9600;
  left_wing.pwm_ccw = -8000;
  servo_count = 0;
  }
*/

  // position error
  left_wing.err = pot_left_wing_scaled - commands[1];

  // change in error
  left_wing.d_err = left_wing.err - left_wing_last_err;
  left_wing_last_err = left_wing.err;

  // command
  left_wing.cmd = left_wing.err * left_wing.pgain + left_wing.d_err * left_wing.dgain;

  // determine whether to turn clockwise or counterclockwise
  if (left_wing.cmd > 0) {
    left_wing.pwm_ccw = left_wing.cmd - 9600;
    Bound(left_wing.pwm_ccw, -9600, 9600); 
    left_wing.pwm_cw = -9600.;
  }
  else {
    left_wing.pwm_cw = -left_wing.cmd - 9600;
    Bound(left_wing.pwm_cw, -9600, 9600);
    left_wing.pwm_ccw = -9600.;
  }
/*
  // the error
  left_wing.err = abs(pot_left_wing_scaled - commands[1]);
  right_wing.err = abs(pot_right_wing_scaled - commands[1]);
  // change in error
  left_wing.d_err = left_wing.err - left_wing_last_err;
  left_wing_last_err = left_wing.err;
  // sum error
  left_wing.sum_err = left_wing.err + left_wing.err *  1.0 / CONTROL_FREQUENCY;

  // decide to turn cw or ccw
  if (pot_left_wing_scaled - commands[1] > 0) {
    left_wing.pwm_ccw = (left_wing.err * left_wing.pgain + left_wing.d_err * left_wing.dgain) - 9600;
    Bound(left_wing.pwm_ccw, -9600, 9600); 
    left_wing.pwm_cw = -9600.;
  }
  else {
    left_wing.pwm_cw = (left_wing.err * left_wing.pgain + left_wing.d_err * left_wing.dgain) - 9600;
    Bound(left_wing.pwm_cw, -9600, 9600);
    left_wing.pwm_ccw = -9600.;
  }
*/  
  if (pot_right_wing_scaled - commands[1] > 0) {
    right_wing.pwm_cw = right_wing.err * right_wing.pgain;
    Bound(right_wing.pwm_cw, -9600, 9600);
    right_wing.pwm_ccw = -9600;
  }
  else {
    right_wing.pwm_ccw = right_wing.err * left_wing.pgain;
    Bound(right_wing.pwm_ccw, -9600, 9600);
    right_wing.pwm_cw = -9600;
  }

}
