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

#ifndef AIRSPEED_THROTTLE_CTL_H
#define AIRSPEED_THROTTLE_CTL_H

#include "std.h"

extern float nominal_airspeed_setpoint;

extern float throttle_ctl_airspeed_pgain;
extern float throttle_ctl_airspeed_igain;
extern float throttle_ctl_airspeed_sum_err;
extern float nominal_cruise_throttle;
extern float throttle_command;

extern void airspeed_throttle_ctl_init(void);
extern float throttle_controller(void);

#endif /* AIRSPEED_THROTTLE_CTL_H */
