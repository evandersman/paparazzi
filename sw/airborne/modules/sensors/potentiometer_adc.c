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
#include "mcu_periph/adc.h"
#include "mcu_periph/uart.h"
#include BOARD_CONFIG
#include "generated/airframe.h"
#include "state.h"
#include <stdio.h>
#include "std.h"
#include "inter_mcu.h"

#ifndef ADC_CHANNEL_POTENTIOMETER_NB_SAMPLES
#define ADC_CHANNEL_POTENTIOMETER_NB_SAMPLES 10
#endif


static struct adc_buf buf_potentiometer_left;
static struct adc_buf buf_potentiometer_right;

uint16_t potentiometer_adc_raw_left;
uint16_t potentiometer_adc_raw_right;

void potentiometer_adc_init(void)
{
  adc_buf_channel(ADC_CHANNEL_POTENTIOMETER_LEFT, &buf_potentiometer_left, ADC_CHANNEL_POTENTIOMETER_NB_SAMPLES);
  adc_buf_channel(ADC_CHANNEL_POTENTIOMETER_RIGHT, &buf_potentiometer_right, ADC_CHANNEL_POTENTIOMETER_NB_SAMPLES);
}

void potentiometer_adc_update(void)
{
  // 12bit adc values
  potentiometer_adc_raw_left = buf_potentiometer_left.sum / buf_potentiometer_left.av_nb_sample;
  potentiometer_adc_raw_right = buf_potentiometer_right.sum / buf_potentiometer_right.av_nb_sample;
}
