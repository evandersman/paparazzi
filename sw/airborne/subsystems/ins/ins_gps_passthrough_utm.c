/*
 * Copyright (C) 2004-2012 The Paparazzi Team
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
 * @file subsystems/ins/ins_gps_passthrough_utm.c
 *
 * Simply passes GPS UTM position and velocity through to the state interface.
 * For fixedwing firmware since it sets UTM pos only.
 */

#include "subsystems/ins/ins_gps_passthrough.h"
#include "subsystems/ins.h"

#include <inttypes.h>
#include <math.h>

#include "state.h"
#include "subsystems/gps.h"
#include "firmwares/fixedwing/nav.h"


#include "subsystems/abi.h"
/** ABI binding for gps data.
 * Used for GPS ABI messages.
 */
#ifndef INS_PT_GPS_ID
#define INS_PT_GPS_ID GPS_MULTI_ID
#endif
PRINT_CONFIG_VAR(INS_PT_GPS_ID)
static abi_event gps_ev;

struct InsGpsPassthrough {
  struct LtpDef_i  ltp_def;
  bool           ltp_initialized;

  /* output LTP NED */
  struct NedCoor_i ltp_pos;
  struct NedCoor_i ltp_speed;
  struct NedCoor_i ltp_accel;
};

struct InsGpsPassthrough ins_gp;
#include "led.h"

static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct GpsState *gps_s)
{
  if (gps_s->fix < GPS_FIX_3D) {
    return;
  }
  if (!ins_gp.ltp_initialized) {
    ins_reset_local_origin();
  }

  /* simply scale and copy pos/speed from gps */
  struct NedCoor_i gps_pos_cm_ned;
  ned_of_ecef_point_i(&gps_pos_cm_ned, &ins_gp.ltp_def, &gps_s->ecef_pos);
  INT32_VECT3_SCALE_2(ins_gp.ltp_pos, gps_pos_cm_ned,
                      INT32_POS_OF_CM_NUM, INT32_POS_OF_CM_DEN);
  stateSetPositionNed_i(&ins_gp.ltp_pos);

  struct NedCoor_i gps_speed_cm_s_ned;
  ned_of_ecef_vect_i(&gps_speed_cm_s_ned, &ins_gp.ltp_def, &gps_s->ecef_vel);
  INT32_VECT3_SCALE_2(ins_gp.ltp_speed, gps_speed_cm_s_ned,
                      INT32_SPEED_OF_CM_S_NUM, INT32_SPEED_OF_CM_S_DEN);
  stateSetSpeedNed_i(&ins_gp.ltp_speed);
  LED_TOGGLE(3);
  /*struct UtmCoor_f utm = utm_float_from_gps(gps_s, nav_utm_zone0);

  // set position
  stateSetPositionUtm_f(&utm);

  struct NedCoor_f ned_vel = {
    gps_s->ned_vel.x / 100.0f,
    gps_s->ned_vel.y / 100.0f,
    gps_s->ned_vel.z / 100.0f
  };
  // set velocity
  stateSetSpeedNed_f(&ned_vel);*/
}


void ins_gps_passthrough_init(void)
{
  #if USE_INS_NAV_INIT
  struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
  llh_nav0.lat = NAV_LAT0;
  llh_nav0.lon = NAV_LON0;
  /* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
  llh_nav0.alt = NAV_ALT0 + NAV_MSL0;

  struct EcefCoor_i ecef_nav0;
  ecef_of_lla_i(&ecef_nav0, &llh_nav0);

  ltp_def_from_ecef_i(&ins_gp.ltp_def, &ecef_nav0);
  ins_gp.ltp_def.hmsl = NAV_ALT0;
  //stateSetLocalOrigin_i(&ins_gp.ltp_def);

  ins_gp.ltp_initialized = true;
#else
  ins_gp.ltp_initialized  = false;
#endif

  INT32_VECT3_ZERO(ins_gp.ltp_pos);
  INT32_VECT3_ZERO(ins_gp.ltp_speed);
  INT32_VECT3_ZERO(ins_gp.ltp_accel);

  struct UtmCoor_f utm0 = { nav_utm_north0, nav_utm_east0, 0., nav_utm_zone0 };
  stateSetLocalUtmOrigin_f(&utm0);
  stateSetPositionUtm_f(&utm0);

  AbiBindMsgGPS(INS_PT_GPS_ID, &gps_ev, gps_cb);
}

void ins_reset_local_origin(void)
{
  ltp_def_from_ecef_i(&ins_gp.ltp_def, &gps.ecef_pos);
  ins_gp.ltp_def.lla.alt = gps.lla_pos.alt;
  ins_gp.ltp_def.hmsl = gps.hmsl;
  //stateSetLocalOrigin_i(&ins_gp.ltp_def);
  ins_gp.ltp_initialized = true;

  struct UtmCoor_f utm = utm_float_from_gps(&gps, 0);

  // reset state UTM ref
  stateSetLocalUtmOrigin_f(&utm);
}

void ins_reset_altitude_ref(void)
{
  struct LlaCoor_i lla = {
    .lat = state.ned_origin_i.lla.lat,
    .lon = state.ned_origin_i.lla.lon,
    .alt = gps.lla_pos.alt
  };
  ltp_def_from_lla_i(&ins_gp.ltp_def, &lla);
  ins_gp.ltp_def.hmsl = gps.hmsl;
  //stateSetLocalOrigin_i(&ins_gp.ltp_def);

  struct UtmCoor_f utm = state.utm_origin_f;
  utm.alt = gps.hmsl / 1000.0f;
  stateSetLocalUtmOrigin_f(&utm);
}
