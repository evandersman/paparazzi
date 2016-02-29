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

/** @file modules/sensors/potentiometer_adc.h
 * Read servo potentiometer via onboard ADC.
 */

#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <inttypes.h>

struct servo_data {
  float offset;
  float gain;
  float err;
  float d_err;
  float sum_err;
  float pwm_cw;
  float pwm_ccw;
  float pgain;
  float dgain;
  float igain;
};

extern struct servo_data left_wing;
extern struct servo_data right_wing;
extern float pot_left_wing_scaled;
extern float pot_right_wing_scaled;
extern float left_wing_last_err;

void servo_controller_init(void);
void servo_controller_update(void);

#endif /* SERVO_CONTROLLER_H */
