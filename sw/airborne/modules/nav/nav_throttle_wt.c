/*
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file modules/nav/nav_throttle_wt.h
 * @brief throttle control for wind tunnel autonomous flight
 */

#include "modules/nav/nav_throttle_wt.h"

#include "state.h"
#include "generated/airframe.h"
#include "firmwares/fixedwing/nav.h"
#include "firmwares/fixedwing/autopilot.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include "generated/flight_plan.h"
#include "subsystems/navigation/common_nav.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/gps/gps_datalink.h"

float h_ctl_throttle_pgain = H_CTL_THROTTLE_PGAIN;
float h_ctl_throttle_igain = H_CTL_THROTTLE_IGAIN;
float h_ctl_throttle_dgain = H_CTL_THROTTLE_DGAIN;
float real_nominal_throttle = V_CTL_AUTO_THROTTLE_NOMINAL_CRUISE_THROTTLE;

float  sum_err_x;

void nav_nom_throttle_init(void)
{
  sum_err_x = 0.;
}
/**
 *  \brief Computes the nominal throttle to  desired segment.
 */
void nav_nom_throttle_calc(void)
{
  /** distance to waypoint in x, error used for the P term */
  float err_x = waypoints[WP_STDBY].y - enu_posf.y; // in the range of +/- 0.5m
  /* D term calculation */
  static float last_err;
  float d_err_x = err_x - last_err;
  last_err = err_x;
  /* I term calculation */
  sum_err_x += h_ctl_throttle_igain * err_x;
  if (v_ctl_throttle_setpoint < 2500) { sum_err_x = 0; }
  Bound(sum_err_x, -0.05, 0.05);
  /* calculate the new nominal throttle */
  float cruise = real_nominal_throttle + h_ctl_throttle_pgain * err_x + h_ctl_throttle_igain * sum_err_x + h_ctl_throttle_dgain * d_err_x;
  RunOnceEvery(100, DOWNLINK_SEND_NOM_THROTTLE(DefaultChannel, DefaultDevice, &err_x, &cruise));
  Bound(cruise, V_CTL_AUTO_THROTTLE_MIN_CRUISE_THROTTLE, V_CTL_AUTO_THROTTLE_MAX_CRUISE_THROTTLE);
  v_ctl_auto_throttle_cruise_throttle = cruise;
}

