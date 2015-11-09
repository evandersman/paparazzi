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

/** @file modules/sensors/airspeed_adc.h
 * Read an airspeed or differential pressure sensor via onboard ADC.
 */

#ifndef TURBULENCE_ADC_H
#define TURBULENCE_ADC_H

#include <inttypes.h>

struct TurbulenceAdc {
  uint16_t raw;
  uint16_t offset;
  uint16_t calibration;
  float scaled;
  float scaled_dx;
  float scaled_ddx;
  float filtered;
  float filtered_dx;
  float filtered_ddx;
};

extern struct TurbulenceAdc airspeed_left_adc;
extern struct TurbulenceAdc pitch_left_adc;
extern struct TurbulenceAdc airspeed_right_adc;
extern struct TurbulenceAdc pitch_right_adc;
extern float pgain;
extern float pitch_omega;
extern float pitch_zeta;

void turbulence_adc_init(void);
void turbulence_adc_update(void);

#endif /* TURBULENCE_ADC_H */
