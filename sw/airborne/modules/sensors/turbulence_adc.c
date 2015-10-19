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
#define ADC_CHANNEL_TURBULENCE_NB_SAMPLES DEFAULT_AV_NB_SAMPLE
#endif

static struct adc_buf buf_airspeed_left;
static struct adc_buf buf_pitch_left;
static struct adc_buf buf_airspeed_right;
static struct adc_buf buf_pitch_right;

uint16_t airspeed_left_raw;
uint16_t pitch_left_raw;
uint16_t airspeed_right_raw;
uint16_t pitch_right_raw;
float pgain;
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
}

void turbulence_adc_update(void)
{
  airspeed_left_raw = buf_airspeed_left.sum / buf_airspeed_left.av_nb_sample;
  pitch_left_raw = buf_pitch_left.sum / buf_pitch_left.av_nb_sample;
  airspeed_right_raw = buf_airspeed_right.sum / buf_airspeed_right.av_nb_sample;
  pitch_right_raw = buf_pitch_right.sum / buf_pitch_right.av_nb_sample;
  pgain = 40000;
  cmd_left = (ANGLE_FLOAT_OF_BFP(pitch_left_raw)-0.5) * pgain;
  cmd_right = (ANGLE_FLOAT_OF_BFP(pitch_right_raw)-0.5) * pgain;
  //Bound(cmd_left, -2000, 2000);
  //Bound(cmd_right, -2000, 2000); /*command stays zero, figure out why */
  cmd_trimmed_left = TRIM_PPRZ(cmd_left);
  cmd_trimmed_right = TRIM_PPRZ(cmd_right);
  ap_state->commands[COMMAND_TURB_LEFT] = cmd_trimmed_left;
  ap_state->commands[COMMAND_TURB_RIGHT] = cmd_trimmed_right;

  DOWNLINK_SEND_ADC_TURBULENCE(DefaultChannel, DefaultDevice, &cmd_trimmed_left, &cmd_left, &cmd_trimmed_right, &cmd_right);

}
