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
 * @file firmwares/fixedwing/stabilization/stabilization_attitude.c
 *
 * Fixed wing horizontal control.
 *
 */

#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include "std.h"
#include "led.h"
#include "state.h"
#include "firmwares/fixedwing/nav.h"
#include "generated/airframe.h"
#include CTRL_TYPE_H
#include "firmwares/fixedwing/autopilot.h"

/* outer loop parameters */
float h_ctl_course_setpoint; /* rad, CW/north */
float h_ctl_course_pre_bank;
float h_ctl_course_pre_bank_correction;
float h_ctl_course_pgain;
float h_ctl_course_dgain;
float h_ctl_roll_max_setpoint;

/* roll and pitch disabling */
bool_t h_ctl_disabled;

/* AUTO1 rate mode */
bool_t h_ctl_auto1_rate;


/* inner roll loop parameters */
float  h_ctl_roll_setpoint;
float  h_ctl_roll_pgain;
pprz_t h_ctl_aileron_setpoint;
float  h_ctl_roll_slew;

/* inner pitch loop parameters */
float  h_ctl_pitch_setpoint;
float  h_ctl_pitch_loop_setpoint;
float  h_ctl_pitch_pgain;
float  h_ctl_pitch_dgain;
pprz_t h_ctl_elevator_setpoint;

/* inner yaw loop parameters */
#if H_CTL_YAW_LOOP
float  h_ctl_yaw_rate_setpoint;
pprz_t h_ctl_rudder_setpoint;
#endif

/* inner CL loop parameters */
#if H_CTL_CL_LOOP
pprz_t h_ctl_flaps_setpoint;
#endif

#ifdef USE_AOA
uint8_t h_ctl_pitch_mode;
#endif

/* inner loop pre-command */
float h_ctl_aileron_of_throttle;
float h_ctl_elevator_of_roll;

/* rate loop */
#ifdef H_CTL_RATE_LOOP
float h_ctl_roll_rate_setpoint;
float h_ctl_roll_rate_mode;
float h_ctl_roll_rate_setpoint_pgain;
float h_ctl_hi_throttle_roll_rate_pgain;
float h_ctl_lo_throttle_roll_rate_pgain;
float h_ctl_roll_rate_igain;
float h_ctl_roll_rate_dgain;
#endif

#ifdef H_CTL_COURSE_SLEW_INCREMENT
float h_ctl_course_slew_increment;
#endif


inline static void h_ctl_roll_loop(void);
inline static void h_ctl_pitch_loop(void);
#ifdef H_CTL_RATE_LOOP
static inline void h_ctl_roll_rate_loop(void);
#endif

#ifndef H_CTL_COURSE_PRE_BANK_CORRECTION
#define H_CTL_COURSE_PRE_BANK_CORRECTION 1.
#endif

#ifndef H_CTL_COURSE_DGAIN
#define H_CTL_COURSE_DGAIN 0.
#endif

#ifndef H_CTL_ROLL_RATE_GAIN
#define H_CTL_ROLL_RATE_GAIN 0.
#endif

float h_ctl_roll_attitude_gain;
float h_ctl_roll_rate_gain;

#ifdef AGR_CLIMB
static float nav_ratio;
#endif

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_calibration(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_CALIBRATION(trans, dev, AC_ID,  &v_ctl_auto_throttle_sum_err, &v_ctl_auto_throttle_submode);
}
#endif

void h_ctl_init(void)
{
  h_ctl_course_setpoint = 0.;
  h_ctl_course_pre_bank = 0.;
  h_ctl_course_pre_bank_correction = H_CTL_COURSE_PRE_BANK_CORRECTION;
  h_ctl_course_pgain = H_CTL_COURSE_PGAIN;
  h_ctl_course_dgain = H_CTL_COURSE_DGAIN;
  h_ctl_roll_max_setpoint = H_CTL_ROLL_MAX_SETPOINT;

#ifdef USE_AOA
  h_ctl_pitch_mode = 0;
#endif

  h_ctl_disabled = FALSE;

  h_ctl_roll_setpoint = 0.;
#ifdef H_CTL_ROLL_PGAIN
  h_ctl_roll_pgain = H_CTL_ROLL_PGAIN;
#endif
  h_ctl_aileron_setpoint = 0;
#ifdef H_CTL_AILERON_OF_THROTTLE
  h_ctl_aileron_of_throttle = H_CTL_AILERON_OF_THROTTLE;
#endif

  h_ctl_pitch_setpoint = 0.;
  h_ctl_pitch_loop_setpoint = 0.;
  h_ctl_pitch_pgain = H_CTL_PITCH_PGAIN;
  h_ctl_pitch_dgain = H_CTL_PITCH_DGAIN;
  h_ctl_elevator_setpoint = 0;
  h_ctl_elevator_of_roll = H_CTL_ELEVATOR_OF_ROLL;

#ifdef H_CTL_RATE_LOOP
  h_ctl_roll_rate_mode = H_CTL_ROLL_RATE_MODE_DEFAULT;
  h_ctl_roll_rate_setpoint_pgain = H_CTL_ROLL_RATE_SETPOINT_PGAIN;
  h_ctl_hi_throttle_roll_rate_pgain = H_CTL_HI_THROTTLE_ROLL_RATE_PGAIN;
  h_ctl_lo_throttle_roll_rate_pgain = H_CTL_LO_THROTTLE_ROLL_RATE_PGAIN;
  h_ctl_roll_rate_igain = H_CTL_ROLL_RATE_IGAIN;
  h_ctl_roll_rate_dgain = H_CTL_ROLL_RATE_DGAIN;
#endif

#ifdef H_CTL_ROLL_SLEW
  h_ctl_roll_slew = H_CTL_ROLL_SLEW;
#endif

#ifdef H_CTL_COURSE_SLEW_INCREMENT
  h_ctl_course_slew_increment = H_CTL_COURSE_SLEW_INCREMENT;
#endif

#ifdef H_CTL_ROLL_ATTITUDE_GAIN
  h_ctl_roll_attitude_gain = H_CTL_ROLL_ATTITUDE_GAIN;
  h_ctl_roll_rate_gain = H_CTL_ROLL_RATE_GAIN;
#endif

#ifdef AGR_CLIMB
  nav_ratio = 0;
#endif

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "CALIBRATION", send_calibration);
#endif
}

/**
 * \brief
 *
 */
void h_ctl_course_loop(void)
{
  static float last_err;

  // Ground path error
  float err = stateGetHorizontalSpeedDir_f() - h_ctl_course_setpoint;
  NormRadAngle(err);

#ifdef STRONG_WIND
  // Usefull path speed
  const float reference_advance = (NOMINAL_AIRSPEED / 2.);
  float advance = cos(err) * stateGetHorizontalSpeedNorm_f() / reference_advance;

  if (
    (advance < 1.)  &&                          // Path speed is small
    (stateGetHorizontalSpeedNorm_f() < reference_advance)  // Small path speed is due to wind (small groundspeed)
  ) {
    /*
    // rough crabangle approximation
    float wind_mod = sqrt(wind_east*wind_east + wind_north*wind_north);
    float wind_dir = atan2(wind_east,wind_north);

    float wind_course = h_ctl_course_setpoint - wind_dir;
    NormRadAngle(wind_course);

    estimator_hspeed_dir = estimator_psi;

    float crab = sin(wind_dir-estimator_psi) * atan2(wind_mod,NOMINAL_AIRSPEED);
    //crab = estimator_hspeed_mod - estimator_psi;
    NormRadAngle(crab);
    */

    // Heading error
    float herr = stateGetNedToBodyEulers_f()->psi - h_ctl_course_setpoint; //+crab);
    NormRadAngle(herr);

    if (advance < -0.5) {            //<! moving in the wrong direction / big > 90 degree turn
      err = herr;
    } else if (advance < 0.) {       //<!
      err = (-advance) * 2. * herr;
    } else {
      err = advance * err;
    }

    // Reset differentiator when switching mode
    //if (h_ctl_course_heading_mode == 0)
    //  last_err = err;
    //h_ctl_course_heading_mode = 1;
  }
  /*  else
      {
      // Reset differentiator when switching mode
      if (h_ctl_course_heading_mode == 1)
      last_err = err;
      h_ctl_course_heading_mode = 0;
      }
  */
#endif //STRONG_WIND

  float d_err = err - last_err;
  last_err = err;

  NormRadAngle(d_err);

#ifdef H_CTL_COURSE_SLEW_INCREMENT
  /* slew severe course changes (i.e. waypoint moves, block changes or perpendicular routes) */
  static float h_ctl_course_slew_rate = 0.;
  float nav_angle_saturation = h_ctl_roll_max_setpoint / h_ctl_course_pgain; /* heading error corresponding to max_roll */
  float half_nav_angle_saturation = nav_angle_saturation / 2.;
  if (launch) {  /* prevent accumulator run-up on the ground */
    if (err > half_nav_angle_saturation) {
      h_ctl_course_slew_rate = Max(h_ctl_course_slew_rate, 0.);
      err = Min(err, (half_nav_angle_saturation + h_ctl_course_slew_rate));
      h_ctl_course_slew_rate += h_ctl_course_slew_increment;
    } else if (err < -half_nav_angle_saturation) {
      h_ctl_course_slew_rate = Min(h_ctl_course_slew_rate, 0.);
      err = Max(err, (-half_nav_angle_saturation + h_ctl_course_slew_rate));
      h_ctl_course_slew_rate -= h_ctl_course_slew_increment;
    } else {
      h_ctl_course_slew_rate = 0.;
    }
  }
#endif

  float speed_depend_nav = stateGetHorizontalSpeedNorm_f() / NOMINAL_AIRSPEED;
  Bound(speed_depend_nav, 0.66, 1.5);

  float cmd = -h_ctl_course_pgain * speed_depend_nav * (err + d_err * h_ctl_course_dgain);



#if defined(AGR_CLIMB) && !USE_AIRSPEED
  /** limit navigation during extreme altitude changes */
  if (AGR_BLEND_START > AGR_BLEND_END && AGR_BLEND_END > 0) { /* prevent divide by zero, reversed or negative values */
    if (v_ctl_auto_throttle_submode == V_CTL_AUTO_THROTTLE_AGRESSIVE ||
        v_ctl_auto_throttle_submode == V_CTL_AUTO_THROTTLE_BLENDED) {
      BoundAbs(cmd, h_ctl_roll_max_setpoint); /* bound cmd before NAV_RATIO and again after */
      /* altitude: z-up is positive -> positive error -> too low */
      if (v_ctl_altitude_error > 0) {
        nav_ratio = AGR_CLIMB_NAV_RATIO + (1 - AGR_CLIMB_NAV_RATIO) * (1 - (fabs(v_ctl_altitude_error) - AGR_BLEND_END) /
                    (AGR_BLEND_START - AGR_BLEND_END));
        Bound(nav_ratio, AGR_CLIMB_NAV_RATIO, 1);
      } else {
        nav_ratio = AGR_DESCENT_NAV_RATIO + (1 - AGR_DESCENT_NAV_RATIO) * (1 - (fabs(v_ctl_altitude_error) - AGR_BLEND_END) /
                    (AGR_BLEND_START - AGR_BLEND_END));
        Bound(nav_ratio, AGR_DESCENT_NAV_RATIO, 1);
      }
      cmd *= nav_ratio;
    }
  }
#endif

  float roll_setpoint = cmd + h_ctl_course_pre_bank_correction * h_ctl_course_pre_bank;

#ifdef H_CTL_ROLL_SLEW
  float diff_roll = roll_setpoint - h_ctl_roll_setpoint;
  BoundAbs(diff_roll, h_ctl_roll_slew);
  h_ctl_roll_setpoint += diff_roll;
#else
  h_ctl_roll_setpoint = roll_setpoint;
#endif

  BoundAbs(h_ctl_roll_setpoint, h_ctl_roll_max_setpoint);
}

void h_ctl_attitude_loop(void)
{
  if (!h_ctl_disabled) {
    h_ctl_roll_loop();
    h_ctl_pitch_loop();
  }
}

/** Computes h_ctl_aileron_setpoint from h_ctl_roll_setpoint */
inline static void h_ctl_roll_loop(void)
{
  //calculate the attitude error
  float err = stateGetNedToBodyEulers_f()->phi - h_ctl_roll_setpoint;

  //calculate the rate setpoint
  h_ctl_roll_rate_setpoint = h_ctl_roll_rate_setpoint_pgain * err;
  BoundAbs(h_ctl_roll_rate_setpoint, H_CTL_ROLL_RATE_MAX_SETPOINT);

  //Propagate the second order filter on the gyroscopes
  struct FloatRates *body_rates = stateGetBodyRates_f();
  stabilization_indi_second_order_filter(body_rates, &indi.filtered_rate_2deriv, &indi.filtered_rate_deriv,
                                         &indi.filtered_rate, STABILIZATION_INDI_FILT_OMEGA, STABILIZATION_INDI_FILT_ZETA, STABILIZATION_INDI_FILT_OMEGA_R);
  // Calculate rate error
  float err = stateGetBodyRates_f()->p - h_ctl_roll_rate_setpoint;

  //Calculate required angular acceleration
  indi.angular_accel_ref.p = reference_acceleration.err_p * err
                             - reference_acceleration.rate_p * stateGetBodyRates_f()->p;

  //Incremented in angular acceleration requires increment in control input
  indi.du.p = J / K * (indi.angular_accel_ref.p - indi.filtered_rate_deriv.p);

  //add the increment to the total control input
  indi.u_in.p = indi.u.p + indi.du.p;

  //bound the total control input
  Bound(indi.u_in.p, -4500, 4500);

  //Propagate input filters
  //first order actuator dynamics
  indi.u_act_dyn.p = indi.u_act_dyn.p + STABILIZATION_INDI_ACT_DYN_P * (indi.u_in.p - indi.u_act_dyn.p);

  //sensor filter
  stabilization_indi_second_order_filter(&indi.u_act_dyn, &indi.udotdot, &indi.udot, &indi.u,
                                         STABILIZATION_INDI_FILT_OMEGA, STABILIZATION_INDI_FILT_ZETA, STABILIZATION_INDI_FILT_OMEGA_R);

  //Don't increment if thrust is off
  if (stabilization_cmd[COMMAND_THRUST] < 300) {
    FLOAT_RATES_ZERO(indi.u);
    FLOAT_RATES_ZERO(indi.du);
    FLOAT_RATES_ZERO(indi.u_act_dyn);
    FLOAT_RATES_ZERO(indi.u_in);
    FLOAT_RATES_ZERO(indi.udot);
    FLOAT_RATES_ZERO(indi.udotdot);
  }

  /*  INDI feedback */
  cmd = indi.u_in.p;

  h_ctl_aileron_setpoint = TRIM_PPRZ(cmd);
}

// This is a simple second order low pass filter
void stabilization_indi_second_order_filter(struct FloatRates *input, struct FloatRates *filter_ddx,
    struct FloatRates *filter_dx, struct FloatRates *filter_x, float omega, float zeta, float omega_r)
{
  float_rates_integrate_fi(filter_x, filter_dx, 1.0 / PERIODIC_FREQUENCY);
  float_rates_integrate_fi(filter_dx, filter_ddx, 1.0 / PERIODIC_FREQUENCY);
  float omega2 = omega * omega;
  float omega2_r = omega_r * omega_r;

  filter_ddx->p = -filter_dx->p * 2 * zeta * omega   + (input->p - filter_x->p) * omega2;    \
  filter_ddx->q = -filter_dx->q * 2 * zeta * omega   + (input->q - filter_x->q) * omega2;    \
  filter_ddx->r = -filter_dx->r * 2 * zeta * omega_r + (input->r - filter_x->r) * omega2_r;
}

#ifdef LOITER_TRIM

float v_ctl_auto_throttle_loiter_trim = V_CTL_AUTO_THROTTLE_LOITER_TRIM;
float v_ctl_auto_throttle_dash_trim = V_CTL_AUTO_THROTTLE_DASH_TRIM;

inline static float loiter(void)
{
  static float last_elevator_trim;
  float elevator_trim;

  float throttle_dif = v_ctl_auto_throttle_cruise_throttle - v_ctl_auto_throttle_nominal_cruise_throttle;
  if (throttle_dif > 0) {
    float max_dif = Max(v_ctl_auto_throttle_max_cruise_throttle - v_ctl_auto_throttle_nominal_cruise_throttle, 0.1);
    elevator_trim = throttle_dif / max_dif * v_ctl_auto_throttle_dash_trim;
  } else {
    float max_dif = Max(v_ctl_auto_throttle_nominal_cruise_throttle - v_ctl_auto_throttle_min_cruise_throttle, 0.1);
    elevator_trim = - throttle_dif / max_dif * v_ctl_auto_throttle_loiter_trim;
  }

  float max_change = (v_ctl_auto_throttle_loiter_trim - v_ctl_auto_throttle_dash_trim) / 80.;
  Bound(elevator_trim, last_elevator_trim - max_change, last_elevator_trim + max_change);

  last_elevator_trim = elevator_trim;
  return elevator_trim;
}
#endif


inline static void h_ctl_pitch_loop(void)
{
  static float last_err;
  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  /* sanity check */
  if (h_ctl_elevator_of_roll < 0.) {
    h_ctl_elevator_of_roll = 0.;
  }

  h_ctl_pitch_loop_setpoint =  h_ctl_pitch_setpoint + h_ctl_elevator_of_roll / h_ctl_pitch_pgain * fabs(att->phi);

  float err = 0;

#ifdef USE_AOA
  switch (h_ctl_pitch_mode) {
    case H_CTL_PITCH_MODE_THETA:
      err = att->theta - h_ctl_pitch_loop_setpoint;
      break;
    case H_CTL_PITCH_MODE_AOA:
      err = stateGetAngleOfAttack_f() - h_ctl_pitch_loop_setpoint;
      break;
    default:
      err = att->theta - h_ctl_pitch_loop_setpoint;
      break;
  }
#else //NO_AOA
  err = att->theta - h_ctl_pitch_loop_setpoint;
#endif


  float d_err = err - last_err;
  last_err = err;
  float cmd = -h_ctl_pitch_pgain * (err + h_ctl_pitch_dgain * d_err);
#ifdef LOITER_TRIM
  cmd += loiter();
#endif
  h_ctl_elevator_setpoint = TRIM_PPRZ(cmd);
}
