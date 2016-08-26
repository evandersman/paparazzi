/*
 * Copyright (C) 2005-2013 The Paparazzi Team
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
 *
 */

#ifndef HIGH_SPEED_LOGGER_SPI_LINK_H_
#define HIGH_SPEED_LOGGER_SPI_LINK_H_

#include "std.h"

extern void high_speed_logger_spi_link_init(void);
extern void high_speed_logger_spi_link_periodic(void);

#define PACKED __attribute__((__packed__))

struct PACKED high_speed_logger_spi_link_data {



  int32_t id;           // 1

  /*int32_t cmd_cw_l;
  int32_t cmd_ccw_l;
  int32_t motor_dyn;
  int32_t filt_speed;
  //int32_t cmd_cw_r;
  //int32_t cmd_ccw_r;

  int32_t potscaled_l;
  int32_t potscaled_r;
  int32_t potraw_l;
  int32_t potraw_r;

  int32_t servoerr_l;
  int32_t servoerr_r;

  int32_t pwm_cw_l;
  int32_t pwm_ccw_l;
  int32_t pwm_cw_r;
  int32_t pwm_ccw_r;

  int32_t cmd_roll;*/

  /*int32_t phi;
  int32_t theta;
  //int32_t psi;*/

  /* Test 1 determine G matrix */
  /*int32_t p;
  int32_t q;
  int32_t r;
  int32_t roll_setpoint;
  int32_t pitch_setpoint;
  int32_t cmd_throttle;
  int32_t cmd_roll;
  int32_t cmd_pitch;
  int32_t cmd_yaw;
  int32_t x;
  int32_t y;
  int32_t z;*/

  /* Test 2 INDI and PID reference tracking */
  /*int32_t p;
  int32_t q;
  //int32_t r;
  int32_t roll_setpoint;
  int32_t pitch_setpoint;
  int32_t cmd_indi;
  int32_t cmd_roll;
  int32_t cmd_pitch;
  int32_t ref_acc_pdot;
  int32_t filt_acc_pdot;
  int32_t ref_acc_qdot;
  int32_t filt_acc_qdot;
  int32_t probe_cmd_l;
  int32_t probe_cmd_r;*/

  /* Test 3 OUTER LOOP reference tracking */
  /*int32_t cmd_indi;
  int32_t probe_cmd_l;
  int32_t probe_cmd_r;
  int32_t altitude_setpoint;
  int32_t altitude;
  int32_t climb_setpoint;
  int32_t EnuSpeedZ;
  int32_t throttle_controlled;
  int32_t pitch_setpoint;
  int32_t course_setpoint;
  int32_t des_x;
  int32_t des_y;
  int32_t x;
  int32_t y;
  int32_t roll_setpoint;*/

  int32_t airspeed_left_adc_scaled;
  int32_t pitch_left_adc_scaled;
  int32_t airspeed_right_adc_scaled;
  int32_t pitch_right_adc_scaled;

  int32_t airspeed_left_adc_raw;
  int32_t pitch_left_adc_raw;
  int32_t airspeed_right_adc_raw;
  int32_t pitch_right_adc_raw;

  //int32_t cmd_indi;
  int32_t theta;
  int32_t course_setpoint;
  int32_t des_x;
  int32_t des_y;
  int32_t x;
  int32_t y;
  int32_t roll_setpoint;
};

#endif /* HIGH_SPEED_LOGGER_SPI_LINK_H_ */
