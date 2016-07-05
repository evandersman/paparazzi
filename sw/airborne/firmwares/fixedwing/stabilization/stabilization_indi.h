/*
 * Copyright (C) 2006  Pascal Brisset, Antoine Drouin, Michel Gorraz
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

/**
 * @file firmwares/fixedwing/stabilization/stabilization_attitude.h
 *
 * Fixed wing horizontal control.
 *
 */

#ifndef FW_H_CTL_I_H
#define FW_H_CTL_I_H

#include <inttypes.h>
#include "std.h"
#include "paparazzi.h"
#include "generated/airframe.h"
#include "math/pprz_algebra_float.h"

extern float G_ROLL;
extern float G_PITCH;
extern float tau_act_dyn_p;
extern float indi_omega;
extern float indi_zeta;
extern float indi_omega_r;

extern uint8_t servo_delay;
extern uint8_t delay_p;
extern uint8_t delay_q;
extern struct FloatRates servo_input[SERVO_DELAY];
extern struct FloatRates servo_delayed_input;
extern struct FloatRates u_act_dyn_previous;

struct ReferenceSystem {
  float err_p;
  float err_q;
  float err_r;
  float rate_p;
  float rate_q;
  float rate_r;
};

struct IndiVariables {
  struct FloatRates filtered_rate;
  struct FloatRates filtered_rate_deriv;
  struct FloatRates filtered_rate_2deriv;
  struct FloatRates angular_accel_ref;
  struct FloatRates du;
  struct FloatRates u_act_dyn;
  struct FloatRates u_in;
  struct FloatRates u;
  struct FloatRates udot;
  struct FloatRates udotdot;
};


extern struct ReferenceSystem reference_acceleration;
extern struct IndiVariables indi;

void stabilization_indi_second_order_filter(struct FloatRates *input, struct FloatRates *filter_ddx,
    struct FloatRates *filter_dx, struct FloatRates *filter_x, float omega, float zeta, float omega_r);

#endif /* FW_H_CTL_I_H */
