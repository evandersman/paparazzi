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

#include "modules/sensors/turbulence_adc.h"
#include "mcu_periph/adc.h"
#include "mcu_periph/uart.h"
#include BOARD_CONFIG
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include "generated/airframe.h"
#include "state.h"
#include <stdio.h>
#include "std.h"
#include "inter_mcu.h"

#ifndef ADC_CHANNEL_TURBULENCE_NB_SAMPLES
//#define ADC_CHANNEL_TURBULENCE_NB_SAMPLES DEFAULT_AV_NB_SAMPLE
#define ADC_CHANNEL_TURBULENCE_NB_SAMPLES 4
#endif

/// Default offset value
#ifndef AIRSPEED_LEFT_OFFSET
#define AIRSPEED_LEFT_OFFSET 2050
#endif
#ifndef PITCH_LEFT_OFFSET
#define PITCH_LEFT_OFFSET 2050
#endif
#ifndef AIRSPEED_RIGHT_OFFSET
#define AIRSPEED_RIGHT_OFFSET 2050
#endif
#ifndef PITCH_RIGHT_OFFSET
#define PITCH_RIGHT_OFFSET 2050
#endif

#ifndef TURB_PGAIN
#define TURB_PGAIN 40000
#endif


static struct adc_buf buf_airspeed_left;
static struct adc_buf buf_pitch_left;
static struct adc_buf buf_airspeed_right;
static struct adc_buf buf_pitch_right;

struct TurbulenceAdc airspeed_left_adc;
struct TurbulenceAdc pitch_left_adc;
struct TurbulenceAdc airspeed_right_adc;
struct TurbulenceAdc pitch_right_adc;

float pgain;
float acc_gain;
float probes_ang_acc;
float pitch_omega;
float pitch_zeta;
float pitch_left_adc_previous;
float pitch_left_adc_previous_dx;
float pitch_right_adc_previous;
float pitch_right_adc_previous_dx;
float cmd_left;
float cmd_right;
pprz_t cmd_trimmed_left;
pprz_t cmd_trimmed_right;

void turbulence_adc_init(void)
{
  adc_buf_channel(ADC_CHANNEL_AIRSPEED_LEFT, &buf_airspeed_left, ADC_CHANNEL_TURBULENCE_NB_SAMPLES);
  adc_buf_channel(ADC_CHANNEL_PITCH_LEFT, &buf_pitch_left, ADC_CHANNEL_TURBULENCE_NB_SAMPLES);
  adc_buf_channel(ADC_CHANNEL_AIRSPEED_RIGHT, &buf_airspeed_right, ADC_CHANNEL_TURBULENCE_NB_SAMPLES);
  adc_buf_channel(ADC_CHANNEL_PITCH_RIGHT, &buf_pitch_right, ADC_CHANNEL_TURBULENCE_NB_SAMPLES);

  airspeed_left_adc.offset = AIRSPEED_LEFT_OFFSET;
  pitch_left_adc.offset = PITCH_LEFT_OFFSET;
  airspeed_right_adc.offset = AIRSPEED_RIGHT_OFFSET;
  pitch_right_adc.offset = PITCH_RIGHT_OFFSET;
  pgain = TURB_PGAIN;
  acc_gain = TURB_ACC_GAIN;
  pitch_omega = PITCH_OMEGA;
  pitch_zeta = PITCH_ZETA;

  probes_ang_acc = 0;
  pitch_left_adc.filtered = 0;
  pitch_left_adc.filtered_dx = 0;
  pitch_left_adc.filtered_ddx = 0;
  pitch_left_adc.scaled_dx = 0;
  pitch_left_adc.scaled_ddx = 0;

  pitch_right_adc.filtered = 0;
  pitch_right_adc.filtered_dx = 0;
  pitch_right_adc.filtered_ddx = 0;
  pitch_right_adc.scaled_dx = 0;
  pitch_right_adc.scaled_ddx = 0;
}

void turbulence_adc_update(void)
{
  // 12bit adc values
  airspeed_left_adc.raw = buf_airspeed_left.sum / buf_airspeed_left.av_nb_sample;
  pitch_left_adc.raw = buf_pitch_left.sum / buf_pitch_left.av_nb_sample;
  airspeed_right_adc.raw = buf_airspeed_right.sum / buf_airspeed_right.av_nb_sample;
  pitch_right_adc.raw = buf_pitch_right.sum / buf_pitch_right.av_nb_sample;

  // 12 bit calibration offset
  airspeed_left_adc.calibration = ANGLE_BFP_OF_REAL(ANGLE_FLOAT_OF_BFP(airspeed_left_adc.raw*3.3) - 1.5);
  pitch_left_adc.calibration = ANGLE_BFP_OF_REAL(ANGLE_FLOAT_OF_BFP(pitch_left_adc.raw*3.3) - 1.5);
  airspeed_right_adc.calibration = ANGLE_BFP_OF_REAL(ANGLE_FLOAT_OF_BFP(airspeed_right_adc.raw*3.3) - 1.5);
  pitch_right_adc.calibration = ANGLE_BFP_OF_REAL(ANGLE_FLOAT_OF_BFP(pitch_right_adc.raw*3.3) - 1.5);
  
  
  // pressure differential in millipascal first covert to voltage by using the scaling factor raw*3.3/2^12 and the convert to pressure by using formula in the datasheet
  // positive pressure diff means a gust is coming from above
  airspeed_left_adc.scaled = (ANGLE_FLOAT_OF_BFP(airspeed_left_adc.raw*3.3)-ANGLE_FLOAT_OF_BFP(airspeed_left_adc.offset)-0.1*3.3)*7.6-10.0;
  pitch_left_adc.scaled = (ANGLE_FLOAT_OF_BFP(pitch_left_adc.raw*3.3)-ANGLE_FLOAT_OF_BFP(pitch_left_adc.offset)-0.1*3.3)*7.6-10.0;
  airspeed_right_adc.scaled = (ANGLE_FLOAT_OF_BFP(airspeed_right_adc.raw*3.3)-ANGLE_FLOAT_OF_BFP(airspeed_right_adc.offset)-0.1*3.3)*7.6-10.0;
  pitch_right_adc.scaled = (ANGLE_FLOAT_OF_BFP(pitch_right_adc.raw*3.3)-ANGLE_FLOAT_OF_BFP(pitch_right_adc.offset)-0.1*3.3)*7.6-10.0;

  //high pass filter
  pitch_left_adc.filtered = pitch_left_adc.filtered + pitch_left_adc.filtered_dx * 1.0 / MODULES_FREQUENCY;
  pitch_left_adc.filtered_dx = pitch_left_adc.filtered_dx + pitch_left_adc.filtered_ddx * 1.0 / MODULES_FREQUENCY;

  pitch_left_adc.scaled_dx = (pitch_left_adc.scaled - pitch_left_adc_previous) * MODULES_FREQUENCY;
  pitch_left_adc.scaled_ddx = (pitch_left_adc.scaled_dx - pitch_left_adc_previous_dx) * MODULES_FREQUENCY;

  pitch_right_adc.filtered = pitch_right_adc.filtered + pitch_right_adc.filtered_dx * 1.0 / MODULES_FREQUENCY;
  pitch_right_adc.filtered_dx = pitch_right_adc.filtered_dx + pitch_right_adc.filtered_ddx * 1.0 / MODULES_FREQUENCY;

  pitch_right_adc.scaled_dx = (pitch_right_adc.scaled - pitch_right_adc_previous) * MODULES_FREQUENCY;
  pitch_right_adc.scaled_ddx = (pitch_right_adc.scaled_dx - pitch_right_adc_previous_dx) * MODULES_FREQUENCY;

  float omega2 = pitch_omega * pitch_omega;
  pitch_left_adc_previous = pitch_left_adc.scaled;
  pitch_left_adc_previous_dx = pitch_left_adc.scaled_dx;

  pitch_right_adc_previous = pitch_right_adc.scaled;
  pitch_right_adc_previous_dx = pitch_right_adc.scaled_dx;

  pitch_left_adc.filtered_ddx = -pitch_left_adc.filtered_dx * 2 * pitch_zeta * pitch_omega + pitch_left_adc.scaled_ddx - pitch_left_adc.filtered * omega2;
  pitch_right_adc.filtered_ddx = -pitch_right_adc.filtered_dx * 2 * pitch_zeta * pitch_omega + pitch_right_adc.scaled_ddx - pitch_right_adc.filtered * omega2;
  
  #if PROBES_FF_ANG_ACC
  // predict angular acceleration due to turbulence for indi controller
  probes_ang_acc = (pitch_left_adc.scaled - pitch_right_adc.scaled)*acc_gain;
  #else
  // calculate command in floats
  cmd_left = (pitch_left_adc.scaled)*pgain;
  cmd_right = (pitch_right_adc.scaled)*pgain;
  // trim command
  cmd_trimmed_left = TRIM_PPRZ(cmd_left);
  cmd_trimmed_right = TRIM_PPRZ(cmd_right);
  // send command to ailerons individually
  ap_state->commands[COMMAND_TURB_LEFT] = cmd_trimmed_left;
  ap_state->commands[COMMAND_TURB_RIGHT] = cmd_trimmed_right;
  #endif

  RunOnceEvery(50, DOWNLINK_SEND_ADC_TURBULENCE_SCALED(DefaultChannel, DefaultDevice, &pitch_left_adc.filtered, &pitch_left_adc.scaled, &pitch_right_adc.filtered, &pitch_right_adc.scaled));
  //DOWNLINK_SEND_ADC_TURBULENCE_RAW(DefaultChannel, DefaultDevice, &airspeed_left_adc.calibration, &pitch_left_adc.calibration, &airspeed_right_adc.calibration, &pitch_right_adc.calibration);
  //DOWNLINK_SEND_ADC_TURBULENCE(DefaultChannel, DefaultDevice, &cmd_trimmed_left, &cmd_left, &cmd_trimmed_right, &cmd_right);

}