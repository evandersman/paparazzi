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

#ifndef NAV_THROTTLE_WT_H
#define NAV_THROTTLE_WT_H

extern float h_ctl_throttle_pgain;
extern float h_ctl_throttle_igain;
extern float h_ctl_throttle_dgain;

extern void nav_nom_throttle_init(void);
extern void nav_nom_throttle_calc(void);

#endif
