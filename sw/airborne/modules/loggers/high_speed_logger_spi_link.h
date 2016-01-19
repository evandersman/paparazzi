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
  int32_t pot;          // 2
  int32_t roll_setpoint;
  int32_t phi;
/*
  int32_t phi;          // 2
  int32_t p;
  int32_t roll_setpoint;
*/
/*
  int32_t pgain;        // 5
  int32_t dgain;
  int32_t ref_acc;
  int32_t filt_acc;     // 8
  int32_t act_dyn_p;
  int32_t g;
  int32_t du_p;         // 11
  int32_t u_p;
  int32_t u_in_p;
  int32_t u_act_dyn;    // 14
  int32_t probes_acc;   // 15
  int32_t command_roll; // 16
*/

  int32_t offset_pl;      // 5
  int32_t offset_al;
  int32_t offset_pr;
  int32_t offset_ar;
  int32_t pprobes;      // 9
  int32_t pgain;
  int32_t dgain;
  int32_t probe_press_l;    // 12
  int32_t probe_press_r;
  int32_t command_roll;     // 14
  int32_t command_turb_l;   // 15
  int32_t command_turb_r;   // 16

};

#endif /* HIGH_SPEED_LOGGER_SPI_LINK_H_ */
