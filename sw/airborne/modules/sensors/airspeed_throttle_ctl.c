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
 * Read an airspeed via turbulence module and control the throttle setting.
 */

#include "modules/sensors/airspeed_throttle_ctl.h"
#include "modules/sensors/turbulence_adc.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"
#include "generated/airframe.h"
#include "modules/air_data/air_data.h"
#include "state.h"
#include "subsystems/radio_control.h"

#ifndef NOMINAL_AIRSPEED
#error Please define NOMINAL_AIRSPEED in airframe file
#endif

float nominal_airspeed_setpoint = NOMINAL_AIRSPEED;

float throttle_ctl_airspeed_pgain = THROTTLE_CTL_AIRSPEED_PGAIN;
float throttle_ctl_airspeed_igain = THROTTLE_CTL_AIRSPEED_IGAIN;
float throttle_ctl_airspeed_sum_err;
float throttle_command;

void airspeed_throttle_ctl_init(void){
  throttle_ctl_airspeed_sum_err = 0;
  throttle_command = 0;
}

float throttle_controller(void)
{
  // calculate throttle value
  float err_airspeed = (nominal_airspeed_setpoint - air_data.airspeed);
  throttle_ctl_airspeed_sum_err += err_airspeed;
  BoundAbs(throttle_ctl_airspeed_sum_err, 5000);
  throttle_command = (err_airspeed + throttle_ctl_airspeed_sum_err * throttle_ctl_airspeed_igain) * throttle_ctl_airspeed_pgain;
  return throttle_command;
}


