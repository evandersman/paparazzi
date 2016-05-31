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
float right_wing_last_err;

void servo_controller_init(void)
{
  pot_left_wing_scaled  = 0.;
  pot_right_wing_scaled = 0.;

  left_wing_last_err    = 0.;
  left_wing.err         = 0.;
  left_wing.d_err       = 0.;
  left_wing.cmd         = 0.;

  right_wing_last_err    = 0.;
  right_wing.err         = 0.;
  right_wing.d_err       = 0.;
  right_wing.cmd         = 0.;

  servo_count           = 0;

  left_wing.pgain       = SERVO_PGAIN;
  left_wing.dgain       = SERVO_DGAIN;
  left_wing.igain       = SERVO_IGAIN;
  left_wing.offset      = POT_LEFT_OFFSET;
  left_wing.gain        = POT_LEFT_GAIN;

  right_wing.pgain      = SERVO_PGAIN;
  right_wing.dgain      = SERVO_DGAIN;
  right_wing.igain      = SERVO_IGAIN;
  right_wing.offset     = POT_RIGHT_OFFSET;
  right_wing.gain       = POT_RIGHT_GAIN;
}

void servo_controller_update(void)
{

  // potentiometer value
  pot_left_wing_scaled = -(potentiometer_adc_raw_left - left_wing.offset) * left_wing.gain;
  pot_right_wing_scaled = -(potentiometer_adc_raw_right - right_wing.offset) * right_wing.gain;

#ifdef SERVO_DEBUG
  if (servo_count < 500) {

  if (servo_count < 250) {
    left_wing.pwm_cw = -6000;
    left_wing.pwm_ccw = -9600;
  }
  if (servo_count > 249) {
  left_wing.pwm_cw = -9600;
  left_wing.pwm_ccw = -6000;
  }
  servo_count++;
  }
  else {
  servo_count = 0;
  }

#else

  // position error
  left_wing.err = pot_left_wing_scaled - commands[1];
  right_wing.err = pot_right_wing_scaled - commands[1];

  // change in error
  left_wing.d_err = left_wing.err - left_wing_last_err;
  left_wing_last_err = left_wing.err;

  right_wing.d_err = right_wing.err - right_wing_last_err;
  right_wing_last_err = right_wing.err;

  // command
  left_wing.cmd = left_wing.err * left_wing.pgain + left_wing.d_err * left_wing.dgain;
  right_wing.cmd = right_wing.err * right_wing.pgain + right_wing.d_err * right_wing.dgain;

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
 
  if (right_wing.cmd > 0) {
    right_wing.pwm_ccw = right_wing.cmd - 9600;
    Bound(right_wing.pwm_ccw, -9600, 9600); 
    right_wing.pwm_cw = -9600.;
  }
  else {
    right_wing.pwm_cw = -right_wing.cmd - 9600;
    Bound(right_wing.pwm_cw, -9600, 9600);
    right_wing.pwm_ccw = -9600.;
  }
#endif
/*
  // position error
  left_wing.err = pot_left_wing_scaled - commands[1];
  right_wing.err = pot_right_wing_scaled - commands[1];

  //Propagate the second order filter on the potentiometer output
  float omega2 = indi_omega * indi_omega;
  filtered_pos.left_wing  = filtered_pos.left_wing + filtered_rate.left_wing * 1.0 / CONTROL_FREQUENCY;
  filtered_rate.left_wing = filtered_rate.left_wing + filtered_acc.left_wing * 1.0 / CONTROL_FREQUENCY;
  filtered_acc.left_wing  = -filtered_rate.left_wing * 2 * indi_zeta * indi_omega   + (pot_left_wing_scaled - filtered_pos.left_wing) * omega2;

  // Calculate required servo rate
  rate_ref.left_wing = reference_rate.err_p * left_wing.err;

  // Increments in rate require increments in pwm
  dpwm.left_wing = 1.0/G_SERVO * (rate_ref.left_wing - filtered_rate.left_wing);

  // Add the increment to the total control input
  pwm_in.left_wing = pwm.left_wing + dpwm.left_wing;

  // Bound the total control input
  Bound(pwm_in.left_wing, -4500, 4500);

  // First order actuator dynamics
  motor_dyn.left_wing = motor_dyn.left_wing + tau_act_dyn_p * (pwm_in.left_wing - motor_dyn.left_wing);

  // Sensor filter
  pwm.left_wing = pwm.left_wing + pwmdot.left_wing * 1.0 / CONTROL_FREQUENCY;
  pwmdot.left_wing =  pwmdot.left_wing + pwmdotdot.left_wing * 1.0 / CONTROL_FREQUENCY;
  pwmdotdot.left_wing = -pwmdot.left_wing * 2 * indi_zeta * indi_omega   + (motor_dyn.left_wing - pwm.left_wing) * omega2;

  // INDI feedback
  left_wing.cmd = pwm_in.left_wing;*/

}
