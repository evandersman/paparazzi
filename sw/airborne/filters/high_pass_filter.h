/*
 * Copyright (C) 2013 Gautier Hattenberger
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

/** @file filters/high_pass_filter.h
 *  @brief Simple high pass filter
 *
 */

#ifndef HIGH_PASS_FILTER_H
#define HIGH_PASS_FILTER_H

#include "std.h"
#include "math/pprz_algebra_int.h"


/** Fourth order filter structure.
 *
 * Polynomial discrete form:
 *
 *        b0 + b1 z^-1 + b2 z^-2 etc
 * H(z) = ----------------------
 *        a0 + a1 z^-1 + a2 z^-2 etc
 */
struct FourthOrderHighPass {
  float a[4]; ///< denominator gains
  float b[4]; ///< numerator gains
  float i[4]; ///< input history
  float o[4]; ///< output history
};


/** Init fourth order high pass filter.
 *
 * @param filter fourth order high pass filter structure
 * @param value initial value of the filter
 */
static inline void init_fourth_order_high_pass(struct FourthOrderHighPass *filter, float *a, float *b, float value)
{
  filter->a[0] = a[0];
  filter->a[1] = a[1];
  filter->a[2] = a[2];
  filter->a[3] = a[3];
  filter->b[0] = b[0];
  filter->b[1] = b[1];
  filter->b[2] = b[2];
  filter->b[3] = b[3];
  filter->i[0] = filter->i[1] = filter->i[2] = filter->i[3] = filter->o[0] = filter->o[1] = filter->o[2] = filter->o[3] = value;
}

/** Update fourth order high pass filter state with a new value.
 *
 * @param filter fourth order high pass filter structure
 * @param value new input value of the filter
 * @return new filtered value
 */
static inline float update_fourth_order_high_pass(struct FourthOrderHighPass *filter, float value)
{
  float out = filter->b[0] * value
              + filter->b[1] * filter->i[0]
              + filter->b[2] * filter->i[1]
              + filter->b[3] * filter->i[2]
              + filter->b[0] * filter->i[3]
              - filter->a[0] * filter->o[0]
              - filter->a[1] * filter->o[1]
              - filter->a[2] * filter->o[2]
              - filter->a[3] * filter->o[3];

  filter->i[3] = filter->i[2];
  filter->i[2] = filter->i[1];
  filter->i[1] = filter->i[0];
  filter->i[0] = value;
  filter->o[3] = filter->o[2];
  filter->o[2] = filter->o[1];
  filter->o[1] = filter->o[0];
  filter->o[0] = out;
  return out;
}

/** Get current value of the fourth order high pass filter.
 *
 * @param filter fourth order high pass filter structure
 * @return current value of the filter
 */
static inline float get_fourth_order_high_pass(struct FourthOrderHighPass *filter)
{
  return filter->o[0];
}

#endif

